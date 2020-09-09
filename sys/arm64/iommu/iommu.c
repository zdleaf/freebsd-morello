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

#include "iommu.h"
#include "smmu_if.h"

static MALLOC_DEFINE(M_IOMMU, "IOMMU", "IOMMU framework");
static MALLOC_DEFINE(M_BUSDMA, "SMMU", "ARM64 busdma SMMU");

#define	IOMMU_LIST_LOCK()		mtx_lock(&iommu_mtx)
#define	IOMMU_LIST_UNLOCK()		mtx_unlock(&iommu_mtx)
#define	IOMMU_LIST_ASSERT_LOCKED()	mtx_assert(&iommu_mtx, MA_OWNED)

#define dprintf(fmt, ...)

static struct mtx iommu_mtx;
static LIST_HEAD(, smmu_unit) iommu_list = LIST_HEAD_INITIALIZER(iommu_list);

static int
domain_unmap_buf(struct iommu_domain *iodom, iommu_gaddr_t base,
    iommu_gaddr_t size, int flags)
{
	struct smmu_domain *domain;
	struct smmu_unit *unit;
	int error;

	unit = (struct smmu_unit *)iodom->iommu;
	domain = (struct smmu_domain *)iodom;

	error = SMMU_UNMAP(unit->dev, domain, base, size);

	return (error);
}

static int
domain_map_buf(struct iommu_domain *iodom, iommu_gaddr_t base,
    iommu_gaddr_t size, vm_page_t *ma, uint64_t eflags, int flags)
{
	struct smmu_unit *unit;
	struct smmu_domain *domain;
	vm_prot_t prot;
	vm_offset_t va;
	int error;

	domain = (struct smmu_domain *)iodom;

	dprintf("%s: base %lx, size %lx\n", __func__, base, size);

	prot = 0;
	if (eflags & IOMMU_MAP_ENTRY_READ)
		prot |= VM_PROT_READ;
	if (eflags & IOMMU_MAP_ENTRY_WRITE)
		prot |= VM_PROT_WRITE;

	va = base;

	unit = (struct smmu_unit *)iodom->iommu;
	error = SMMU_MAP(unit->dev, domain, va, ma, size, prot);

	return (0);
}

static const struct iommu_domain_map_ops smmu_domain_map_ops = {
	.map = domain_map_buf,
	.unmap = domain_unmap_buf,
};

static struct smmu_domain *
smmu_domain_alloc(struct iommu_unit *unit)
{
	struct iommu_domain *iodom;
	struct smmu_unit *iommu;
	struct smmu_domain *domain;

	iommu = (struct smmu_unit *)unit;

	domain = SMMU_DOMAIN_ALLOC(iommu->dev);
	if (domain == NULL)
		return (NULL);

	iodom = (struct iommu_domain *)domain;

	LIST_INIT(&domain->ctx_list);
	iommu_domain_init(unit, iodom, &smmu_domain_map_ops);

	domain->domain.end = BUS_SPACE_MAXADDR;
	iommu_gas_init_domain(&domain->domain);

	IOMMU_LOCK(unit);
	LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
	IOMMU_UNLOCK(unit);

	return (domain);
}

static int
smmu_domain_free(struct smmu_domain *domain)
{
	struct iommu_unit *unit;
	struct smmu_unit *iommu;
	int error;

	unit = domain->domain.iommu;
	iommu = (struct smmu_unit *)unit;

	IOMMU_LOCK(unit);
	LIST_REMOVE(domain, next);
	error = SMMU_DOMAIN_FREE(iommu->dev, domain);
	if (error) {
		LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
		IOMMU_UNLOCK(unit);
		return (error);
	}

	IOMMU_UNLOCK(unit);

	return (0);
}

static struct smmu_ctx *
smmu_ctx_lookup(device_t dev)
{
	struct smmu_domain *domain;
	struct smmu_ctx *ctx;
	struct smmu_unit *iommu;

	LIST_FOREACH(iommu, &iommu_list, next) {
		LIST_FOREACH(domain, &iommu->domain_list, next) {
			LIST_FOREACH(ctx, &domain->ctx_list, next) {
				if (ctx->dev == dev)
					return (ctx);
			}
		}
	}

	return (NULL);
}

static void
smmu_tag_init(struct bus_dma_tag_iommu *t)
{
	bus_addr_t maxaddr;

	maxaddr = BUS_SPACE_MAXADDR;

	t->common.ref_count = 0;
	t->common.impl = &bus_dma_iommu_impl;
	t->common.boundary = 0;
	t->common.lowaddr = maxaddr;
	t->common.highaddr = maxaddr;
	t->common.maxsize = maxaddr;
	t->common.nsegments = BUS_SPACE_UNRESTRICTED;
	t->common.maxsegsz = maxaddr;
}

static struct smmu_ctx *
smmu_ctx_alloc(device_t dev)
{
	struct smmu_ctx *ctx;

	ctx = malloc(sizeof(struct smmu_ctx), M_IOMMU, M_WAITOK | M_ZERO);
	ctx->ctx.rid = pci_get_rid(dev);
	ctx->dev = dev;

	return (ctx);
}
/*
 * Attach a consumer device to a domain.
 */
static int
smmu_ctx_attach(struct smmu_domain *domain, struct smmu_ctx *ctx)
{
	struct iommu_domain *iodom;
	struct smmu_unit *iommu;
	int error;

	iommu = (struct smmu_unit *)domain->domain.iommu;

	error = SMMU_CTX_ATTACH(iommu->dev, domain, ctx);
	if (error) {
		device_printf(iommu->dev, "Failed to add ctx\n");
		return (error);
	}

	ctx->domain = domain;

	iodom = (struct iommu_domain *)domain;

	IOMMU_DOMAIN_LOCK(iodom);
	LIST_INSERT_HEAD(&domain->ctx_list, ctx, next);
	IOMMU_DOMAIN_UNLOCK(iodom);

	return (error);
}

static int
iommu_map_gic_page(struct smmu_domain *domain, uint64_t gic_page)
{
	int error;

	/* Reserve the GIC page */
	error = iommu_gas_reserve_region(&domain->domain, gic_page,
	    gic_page + PAGE_SIZE);
	if (error != 0)
		return (error);

	/* Map the GICv3 ITS page so the device could send MSI interrupts. */
	iommu_map_page(domain, gic_page, gic_page, VM_PROT_WRITE);

	return (0);
}

struct iommu_ctx *
iommu_get_ctx(struct iommu_unit *iommu, device_t requester,
    uint16_t rid, bool disabled, bool rmrr)
{
	struct smmu_ctx *ctx;
	struct smmu_domain *domain;
	struct bus_dma_tag_iommu *tag;
	int error;

	ctx = smmu_ctx_lookup(requester);
	if (ctx)
		return (&ctx->ctx);

	ctx = smmu_ctx_alloc(requester);
	if (ctx == NULL)
		return (NULL);

	if (disabled)
		ctx->bypass = true;

	/* In our current configuration we have a domain per each ctx. */
	domain = smmu_domain_alloc(iommu);
	if (domain == NULL)
		return (NULL);

	tag = ctx->ctx.tag = malloc(sizeof(struct bus_dma_tag_iommu),
	    M_IOMMU, M_WAITOK | M_ZERO);
	tag->owner = requester;
	tag->ctx = (struct iommu_ctx *)ctx;
	tag->ctx->domain = (struct iommu_domain *)domain;

	smmu_tag_init(tag);

	ctx->domain = domain;

	error = smmu_ctx_attach(domain, ctx);
	if (error) {
		smmu_domain_free(domain);
		return (NULL);
	}

	return (&ctx->ctx);
}

void
iommu_free_ctx_locked(struct iommu_unit *unit, struct iommu_ctx *ctx)
{
	struct smmu_domain *domain;
	struct smmu_unit *iommu;
	struct smmu_ctx *context;
	int error;

	IOMMU_ASSERT_LOCKED(unit);

	domain = (struct smmu_domain *)ctx->domain;
	iommu = (struct smmu_unit *)unit;
	context = (struct smmu_ctx *)ctx;

	error = SMMU_CTX_DETACH(iommu->dev, (struct smmu_ctx *)ctx);
	if (error) {
		device_printf(iommu->dev, "Failed to remove device\n");
		return;
	}

	LIST_REMOVE((struct smmu_ctx *)ctx, next);
	free(ctx->tag, M_IOMMU);

	IOMMU_UNLOCK(unit);

	/* Since we have a domain per each ctx, remove the domain too. */
	error = smmu_domain_free(domain);
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

int
iommu_map_page(struct smmu_domain *domain,
    vm_offset_t va, vm_paddr_t pa, vm_prot_t prot)
{
	struct smmu_unit *iommu;
	int error;

	iommu = (struct smmu_unit *)domain->domain.iommu;

	error = SMMU_MAP_PAGE(iommu->dev, domain, va, pa, prot);
	if (error)
		return (error);

	return (0);
}

int
iommu_unmap_page(struct smmu_domain *domain, vm_offset_t va)
{
	struct smmu_unit *iommu;
	int error;

	iommu = (struct smmu_unit *)domain->domain.iommu;

	error = SMMU_UNMAP(iommu->dev, domain, va, PAGE_SIZE);
	if (error)
		return (error);

	return (0);
}

static void
smmu_domain_free_entry(struct iommu_map_entry *entry, bool free)
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
	struct smmu_unit *unit;
	struct iommu_map_entry *entry, *entry1;
	int error;

	unit = (struct smmu_unit *)domain->iommu;

	TAILQ_FOREACH_SAFE(entry, entries, dmamap_link, entry1) {
		KASSERT((entry->flags & IOMMU_MAP_ENTRY_MAP) != 0,
		    ("not mapped entry %p %p", domain, entry));
		error = domain->ops->unmap(domain, entry->start, entry->end -
		    entry->start, cansleep ? IOMMU_PGF_WAITOK : 0);
		KASSERT(error == 0, ("unmap %p error %d", domain, error));
		TAILQ_REMOVE(entries, entry, dmamap_link);
		smmu_domain_free_entry(entry, true);
        }

	if (TAILQ_EMPTY(entries))
		return;

	panic("here");
}

int
iommu_register(device_t dev, struct smmu_unit *iommu, intptr_t xref)
{

	iommu->dev = dev;
	iommu->xref = xref;

	LIST_INIT(&iommu->domain_list);
	mtx_init(&iommu->unit.lock, "IOMMU", NULL, MTX_DEF);

	IOMMU_LIST_LOCK();
	LIST_INSERT_HEAD(&iommu_list, iommu, next);
	IOMMU_LIST_UNLOCK();

	iommu_init_busdma((struct iommu_unit *)iommu);

	return (0);
}

int
iommu_unregister(device_t dev)
{
	struct smmu_unit *iommu;
	bool found;

	found = false;

	IOMMU_LIST_LOCK();
	LIST_FOREACH(iommu, &iommu_list, next) {
		if (iommu->dev == dev) {
			found = true;
			break;
		}
	}

	if (!found) {
		IOMMU_LIST_UNLOCK();
		return (ENOENT);
	}

	if (!LIST_EMPTY(&iommu->domain_list)) {
		IOMMU_LIST_UNLOCK();
		return (EBUSY);
	}

	LIST_REMOVE(iommu, next);
	IOMMU_LIST_UNLOCK();

	free(iommu, M_IOMMU);

	return (0);
}

static struct smmu_unit *
smmu_lookup(intptr_t xref)
{
	struct smmu_unit *iommu;

	LIST_FOREACH(iommu, &iommu_list, next) {
		if (iommu->xref == xref)
			return (iommu);
	}

	return (NULL);
}

struct iommu_unit *
iommu_find(device_t dev, bool verbose)
{
	struct smmu_unit *iommu;
	u_int xref, sid;
	uint16_t rid;
	int error;
	int seg;

	rid = pci_get_rid(dev);
	seg = pci_get_domain(dev);

	/*
	 * Find an xref of an IOMMU controller that serves traffic for dev.
	 */
#ifdef DEV_ACPI
	error = acpi_iort_map_pci_smmuv3(seg, rid, &xref, &sid);
	if (error) {
		/* Could not find reference to an SMMU device. */
		return (NULL);
	}
#else
	/* TODO: add FDT support. */
	return (NULL);
#endif

	/*
	 * Find a registered IOMMU controller by xref.
	 */
	iommu = smmu_lookup(xref);
	if (iommu == NULL) {
		/* SMMU device is not registered in the IOMMU framework. */
		return (NULL);
	}

	return ((struct iommu_unit *)iommu);
}

void
iommu_domain_unload_entry(struct iommu_map_entry *entry, bool free)
{

	dprintf("%s\n", __func__);

	smmu_domain_free_entry(entry, free);
}

int
smmu_map_msi(device_t child, uint64_t msi_addr)
{
	struct smmu_ctx *ctx;
	uint64_t gic_page;
	int error;

	ctx = smmu_ctx_lookup(child);
	if (!ctx || ctx->bypass)
		return (0);

	gic_page = trunc_page(msi_addr);

	error = iommu_map_gic_page(ctx->domain, gic_page);

	return (error);
}

static void
iommu_init(void)
{

	mtx_init(&iommu_mtx, "IOMMU", NULL, MTX_DEF);
}

SYSINIT(iommu, SI_SUB_DRIVERS, SI_ORDER_FIRST, iommu_init, NULL);
