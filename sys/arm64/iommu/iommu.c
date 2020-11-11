/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/memdesc.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/tree.h>
#include <sys/taskqueue.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>
#include <vm/vm.h>
#include <vm/uma.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_page.h>
#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpi_pcibvar.h>
#endif
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/iommu/busdma_iommu.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>
#include <machine/vmparam.h>

#include "iommu.h"
#include "iommu_if.h"

static MALLOC_DEFINE(M_IOMMU, "IOMMU", "IOMMU framework");

#define	IOMMU_LIST_LOCK()		mtx_lock(&iommu_mtx)
#define	IOMMU_LIST_UNLOCK()		mtx_unlock(&iommu_mtx)
#define	IOMMU_LIST_ASSERT_LOCKED()	mtx_assert(&iommu_mtx, MA_OWNED)

#define dprintf(fmt, ...)

static struct mtx iommu_mtx;
static LIST_HEAD(, iommu_unit) iommu_list = LIST_HEAD_INITIALIZER(iommu_list);

static int
iommu_domain_unmap_buf(struct iommu_domain *iodom, iommu_gaddr_t base,
    iommu_gaddr_t size, int flags)
{
	struct iommu_unit *iommu;
	int error;

	iommu = iodom->iommu;

	error = IOMMU_UNMAP(iommu->dev, iodom, base, size);

	return (error);
}

static int
iommu_domain_map_buf(struct iommu_domain *iodom, iommu_gaddr_t base,
    iommu_gaddr_t size, vm_page_t *ma, uint64_t eflags, int flags)
{
	struct iommu_unit *iommu;
	vm_prot_t prot;
	vm_offset_t va;
	int error;

	dprintf("%s: base %lx, size %lx\n", __func__, base, size);

	prot = 0;
	if (eflags & IOMMU_MAP_ENTRY_READ)
		prot |= VM_PROT_READ;
	if (eflags & IOMMU_MAP_ENTRY_WRITE)
		prot |= VM_PROT_WRITE;

	va = base;

	iommu = (struct iommu_unit *)iodom->iommu;

	error = IOMMU_MAP(iommu->dev, iodom, va, ma, size, prot);

	return (0);
}

static const struct iommu_domain_map_ops domain_map_ops = {
	.map = iommu_domain_map_buf,
	.unmap = iommu_domain_unmap_buf,
};

static struct iommu_domain *
iommu_domain_alloc(struct iommu_unit *iommu)
{
	struct iommu_domain *iodom;

	iodom = IOMMU_DOMAIN_ALLOC(iommu->dev, iommu);
	if (iodom == NULL)
		return (NULL);

	iommu_domain_init(iommu, iodom, &domain_map_ops);
	iodom->end = VM_MAXUSER_ADDRESS;
	iodom->iommu = iommu;
	iommu_gas_init_domain(iodom);

	return (iodom);
}

static int
iommu_domain_free(struct iommu_domain *domain)
{
	struct iommu_unit *iommu;
	int error;

	iommu = domain->iommu;

	IOMMU_LOCK(iommu);
	error = IOMMU_DOMAIN_FREE(iommu->dev, domain);
	if (error) {
		IOMMU_UNLOCK(iommu);
		return (error);
	}
	IOMMU_UNLOCK(iommu);

	return (0);
}

static struct iommu_ctx *
iommu_ctx_lookup(device_t dev)
{
	struct iommu_unit *iommu;
	struct iommu_ctx *ctx;

	ctx = NULL;

	IOMMU_LIST_LOCK();
	LIST_FOREACH(iommu, &iommu_list, next) {
		ctx = IOMMU_CTX_LOOKUP(iommu->dev, iommu, dev);
		if (ctx != NULL)
			break;
	}
	IOMMU_LIST_UNLOCK();

	return (ctx);
}

static void
iommu_tag_init(struct bus_dma_tag_iommu *t)
{
	bus_addr_t maxaddr;

	maxaddr = BUS_SPACE_MAXADDR;

	t->common.ref_count = 0;
	t->common.impl = &bus_dma_iommu_impl;
	t->common.alignment = 1;
	t->common.boundary = 0;
	t->common.lowaddr = maxaddr;
	t->common.highaddr = maxaddr;
	t->common.maxsize = maxaddr;
	t->common.nsegments = BUS_SPACE_UNRESTRICTED;
	t->common.maxsegsz = maxaddr;
}

static struct iommu_ctx *
iommu_ctx_alloc(device_t dev, struct iommu_domain *iodom)
{
	struct iommu_unit *iommu;
	struct iommu_ctx *ctx;

	iommu = iodom->iommu;

	ctx = IOMMU_CTX_ALLOC(iommu->dev, iodom, dev);
	if (ctx == NULL)
		return (NULL);

	ctx->rid = pci_get_rid(dev);

	return (ctx);
}
/*
 * Attach a consumer device to a domain.
 */
static int
iommu_ctx_attach(struct iommu_domain *iodom, struct iommu_ctx *ctx,
    bool disabled)
{
	struct iommu_unit *iommu;
	int error;

	iommu = iodom->iommu;

	error = IOMMU_CTX_ATTACH(iommu->dev, iodom, ctx, disabled);
	if (error) {
		device_printf(iommu->dev, "Failed to add ctx\n");
		return (error);
	}

	return (error);
}

struct iommu_ctx *
iommu_get_ctx(struct iommu_unit *iommu, device_t requester,
    uint16_t rid, bool disabled, bool rmrr)
{
	struct iommu_ctx *ctx;
	struct iommu_domain *domain;
	struct bus_dma_tag_iommu *tag;
	int error;

	ctx = iommu_ctx_lookup(requester);
	if (ctx)
		return (ctx);

	/*
	 * In our current configuration we have a domain per each ctx.
	 * So allocate a domain first.
	 */
	domain = iommu_domain_alloc(iommu);
	if (domain == NULL)
		return (NULL);

	ctx = iommu_ctx_alloc(requester, domain);
	if (ctx == NULL)
		return (NULL);

	tag = ctx->tag = malloc(sizeof(struct bus_dma_tag_iommu),
	    M_IOMMU, M_WAITOK | M_ZERO);
	tag->owner = requester;
	tag->ctx = ctx;
	tag->ctx->domain = domain;

	iommu_tag_init(tag);

	error = iommu_ctx_attach(domain, ctx, disabled);
	if (error) {
		iommu_domain_free(domain);
		return (NULL);
	}

	ctx->domain = domain;

	return (ctx);
}

void
iommu_free_ctx_locked(struct iommu_unit *iommu, struct iommu_ctx *ctx)
{
	int error;

	IOMMU_ASSERT_LOCKED(iommu);

	error = IOMMU_CTX_DETACH(iommu->dev, ctx);
	if (error) {
		device_printf(iommu->dev, "Failed to remove device\n");
		return;
	}

	free(ctx->tag, M_IOMMU);

	IOMMU_UNLOCK(iommu);

	/* Since we have a domain per each ctx, remove the domain too. */
	error = iommu_domain_free(ctx->domain);
	if (error)
		device_printf(iommu->dev, "Could not free a domain\n");
}

void
iommu_free_ctx(struct iommu_ctx *ctx)
{
	struct iommu_unit *iommu;
	struct iommu_domain *domain;

	domain = ctx->domain;
	iommu = domain->iommu;

	IOMMU_LOCK(iommu);
	iommu_free_ctx_locked(iommu, ctx);
}

static void
iommu_domain_free_entry(struct iommu_map_entry *entry, bool free)
{
	struct iommu_domain *domain;

	domain = entry->domain;

	IOMMU_DOMAIN_LOCK(domain);
	iommu_gas_free_space(domain, entry);
	IOMMU_DOMAIN_UNLOCK(domain);

	if (free)
		iommu_gas_free_entry(domain, entry);
	else
		entry->flags = 0;
}

void
iommu_domain_unload(struct iommu_domain *domain,
    struct iommu_map_entries_tailq *entries, bool cansleep)
{
	struct iommu_unit *iommu;
	struct iommu_map_entry *entry, *entry1;
	int error;

	iommu = (struct iommu_unit *)domain->iommu;

	TAILQ_FOREACH_SAFE(entry, entries, dmamap_link, entry1) {
		KASSERT((entry->flags & IOMMU_MAP_ENTRY_MAP) != 0,
		    ("not mapped entry %p %p", domain, entry));
		error = domain->ops->unmap(domain, entry->start, entry->end -
		    entry->start, cansleep ? IOMMU_PGF_WAITOK : 0);
		KASSERT(error == 0, ("unmap %p error %d", domain, error));
		TAILQ_REMOVE(entries, entry, dmamap_link);
		iommu_domain_free_entry(entry, true);
        }

	if (TAILQ_EMPTY(entries))
		return;

	panic("entries map is not empty");
}

int
iommu_register(struct iommu_unit *iommu)
{

	mtx_init(&iommu->lock, "IOMMU", NULL, MTX_DEF);

	IOMMU_LIST_LOCK();
	LIST_INSERT_HEAD(&iommu_list, iommu, next);
	IOMMU_LIST_UNLOCK();

	iommu_init_busdma(iommu);

	return (0);
}

int
iommu_unregister(struct iommu_unit *iommu)
{

	IOMMU_LIST_LOCK();
	LIST_REMOVE(iommu, next);
	IOMMU_LIST_UNLOCK();

	return (0);
}

struct iommu_unit *
iommu_find(device_t dev, bool verbose)
{
	struct iommu_unit *iommu, *iommu1;

	LIST_FOREACH(iommu, &iommu_list, next) {
		iommu1 = IOMMU_FIND(iommu->dev, dev);
		if (iommu1 != NULL)
			return (iommu1);
	}

	return (NULL);
}

void
iommu_domain_unload_entry(struct iommu_map_entry *entry, bool free)
{

	dprintf("%s\n", __func__);

	iommu_domain_free_entry(entry, free);
}

static void
iommu_init(void)
{

	mtx_init(&iommu_mtx, "IOMMU", NULL, MTX_DEF);
}

SYSINIT(iommu, SI_SUB_DRIVERS, SI_ORDER_FIRST, iommu_init, NULL);
