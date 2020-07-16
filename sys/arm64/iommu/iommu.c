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
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/uma.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_page.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/pci/pcivar.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpi_pcibvar.h>
#endif

#include <x86/iommu/busdma_dmar.h>

#include "iommu.h"
#include "iommu_if.h"

static MALLOC_DEFINE(M_IOMMU, "IOMMU", "IOMMU framework");
static MALLOC_DEFINE(M_BUSDMA, "SMMU", "ARM64 busdma SMMU");

static struct mtx iommu_mtx;

#define	IOMMU_LIST_LOCK()		mtx_lock(&iommu_mtx)
#define	IOMMU_LIST_UNLOCK()		mtx_unlock(&iommu_mtx)
#define	IOMMU_LIST_ASSERT_LOCKED()	mtx_assert(&iommu_mtx, MA_OWNED)

#define IOMMU_DEBUG
#undef IOMMU_DEBUG

#ifdef IOMMU_DEBUG
#define DPRINTF(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif

#define GICV3_ITS_PAGE  0x300b0000

static LIST_HEAD(, smmu_unit) iommu_list = LIST_HEAD_INITIALIZER(iommu_list);
static uma_zone_t iommu_map_entry_zone;

static int
iommu_domain_add_va_range(struct smmu_domain *domain,
    vm_offset_t va, vm_size_t size)
{
	int error;

	KASSERT(size > 0, ("wrong size"));

	error = vmem_add(domain->vmem, va, size, M_WAITOK);

	return (error);
}

static struct smmu_domain *
iommu_domain_alloc(struct iommu_unit *unit)
{
	struct smmu_unit *iommu;
	struct smmu_domain *domain;

	iommu = (struct smmu_unit *)unit;

	domain = IOMMU_DOMAIN_ALLOC(iommu->dev);
	if (domain == NULL)
		return (NULL);

	LIST_INIT(&domain->ctx_list);
	mtx_init(&domain->domain.lock, "IOMMU domain", NULL, MTX_DEF);

	domain->vmem = vmem_create("IOMMU vmem", 0, 0, PAGE_SIZE,
	    PAGE_SIZE, M_FIRSTFIT | M_WAITOK);
	if (domain->vmem == NULL)
		return (NULL);

	domain->domain.iommu = unit;

	IOMMU_LOCK(unit);
	LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
	IOMMU_UNLOCK(unit);

	return (domain);
}

static int
iommu_domain_free(struct smmu_domain *domain)
{
	struct iommu_unit *unit;
	struct smmu_unit *iommu;
	vmem_t *vmem;
	int error;

	unit = domain->domain.iommu;
	iommu = (struct smmu_unit *)unit;
	vmem = domain->vmem;

	IOMMU_LOCK(unit);
	LIST_REMOVE(domain, next);
	error = IOMMU_DOMAIN_FREE(iommu->dev, domain);
	if (error) {
		LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
		IOMMU_UNLOCK(unit);
		return (error);
	}

	IOMMU_UNLOCK(unit);

	vmem_destroy(vmem);

	return (0);
}

static struct smmu_ctx *
iommu_ctx_lookup(device_t dev)
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
iommu_tag_init(struct bus_dma_tag_iommu *t)
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
iommu_ctx_alloc(device_t dev)
{
	struct smmu_ctx *ctx;

	ctx = malloc(sizeof(struct smmu_ctx), M_IOMMU, M_WAITOK | M_ZERO);
	ctx->rid = pci_get_rid(dev);
	ctx->dev = dev;

	return (ctx);
}
/*
 * Attach a consumer device to a domain.
 */
static int
iommu_ctx_attach(struct smmu_domain *domain, struct smmu_ctx *ctx)
{
	struct iommu_domain *iodom;
	struct smmu_unit *iommu;
	int error;

	iommu = (struct smmu_unit *)domain->domain.iommu;

	error = IOMMU_CTX_ATTACH(iommu->dev, domain, ctx);
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


struct iommu_ctx *
iommu_get_ctx(struct iommu_unit *iommu, device_t requester,
    uint16_t rid, bool disabled, bool rmrr)
{
	struct smmu_ctx *ctx;
	struct smmu_domain *domain;
	struct bus_dma_tag_iommu *tag;
	int error;

	ctx = iommu_ctx_lookup(requester);
	if (ctx)
		return (&ctx->ctx);

	ctx = iommu_ctx_alloc(requester);
	if (ctx == NULL)
		return (NULL);

	if (disabled)
		ctx->bypass = true;

	/* In our current configuration we have a domain per each ctx. */
	domain = iommu_domain_alloc(iommu);
	if (domain == NULL)
		return (NULL);

	tag = ctx->ctx.tag = malloc(sizeof(struct bus_dma_tag_iommu),
	    M_IOMMU, M_WAITOK | M_ZERO);
	tag->owner = requester;
	tag->ctx = (struct iommu_ctx *)ctx;
	tag->ctx->domain = (struct iommu_domain *)domain;

	iommu_tag_init(tag);

	ctx->domain = domain;

	error = iommu_ctx_attach(domain, ctx);
	if (error) {
		iommu_domain_free(domain);
		return (NULL);
	}

	/* Add some virtual address range for this domain. */
	iommu_domain_add_va_range(domain, 0x40000000, 0x40000000);

	/* Map the GICv3 ITS page so the device could send MSI interrupts. */
	iommu_map_page(domain, GICV3_ITS_PAGE, GICV3_ITS_PAGE, VM_PROT_WRITE);

	return (&ctx->ctx);
}

void
iommu_free_ctx_locked(struct iommu_unit *unit, struct iommu_ctx *ctx)
{
	struct smmu_domain *domain;
	struct smmu_unit *iommu;
	int error;

	IOMMU_ASSERT_LOCKED(unit);

	domain = (struct smmu_domain *)ctx->domain;
	iommu = (struct smmu_unit *)unit;

	error = IOMMU_CTX_DETACH(iommu->dev, (struct smmu_ctx *)ctx);
	if (error) {
		device_printf(iommu->dev, "Failed to remove device\n");
		return;
	}

	LIST_REMOVE((struct smmu_ctx *)ctx, next);
	free(ctx->tag, M_IOMMU);

	IOMMU_UNLOCK(unit);

	/* Since we have a domain per each ctx, remove the domain too. */
	iommu_unmap_page(domain, GICV3_ITS_PAGE);
	error = iommu_domain_free(domain);
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

	error = IOMMU_MAP(iommu->dev, domain, va, pa, PAGE_SIZE, prot);
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

	error = IOMMU_UNMAP(iommu->dev, domain, va, PAGE_SIZE);
	if (error)
		return (error);

	return (0);
}

struct iommu_map_entry *
iommu_map_alloc_entry(struct iommu_domain *domain, u_int flags)
{
	struct iommu_map_entry *res;

	KASSERT((flags & ~(IOMMU_PGF_WAITOK)) == 0,
	    ("unsupported flags %x", flags));

	res = uma_zalloc(iommu_map_entry_zone, ((flags & IOMMU_PGF_WAITOK) !=
	    0 ? M_WAITOK : M_NOWAIT) | M_ZERO);
	if (res != NULL) {
		res->domain = domain;
		atomic_add_int(&domain->entries_cnt, 1);
	}
	return (res);
}

void iommu_map_free_entry(struct iommu_domain *domain,
    struct iommu_map_entry *entry)
{

	KASSERT(domain == entry->domain,
	    ("mismatched free domain %p entry %p entry->domain %p", domain,
	    entry, entry->domain));
	atomic_subtract_int(&domain->entries_cnt, 1);
	uma_zfree(iommu_map_entry_zone, entry);
}

int
iommu_map(struct iommu_domain *iodom,
    const struct bus_dma_tag_common *common, iommu_gaddr_t size, int offset,
    u_int eflags, u_int flags, vm_page_t *ma, struct iommu_map_entry **res)
{
	struct iommu_map_entry *entry;
	struct smmu_unit *iommu;
	struct smmu_domain *domain;
	vm_prot_t prot;
	vm_offset_t va;
	vm_paddr_t pa;
	int error;

	domain = (struct smmu_domain *)iodom;
	iommu = (struct smmu_unit *)iodom->iommu;

	entry = iommu_map_alloc_entry(iodom, 0);
	if (entry == NULL)
		return (ENOMEM);

	error = vmem_alloc(domain->vmem, size,
	    M_FIRSTFIT | M_NOWAIT, &va);
	if (error) {
		iommu_map_free_entry(iodom, entry);
		return (error);
	}

	pa = VM_PAGE_TO_PHYS(ma[0]);

	entry->start = va;
	entry->end = va + size;

	prot = 0;
	if (eflags & IOMMU_MAP_ENTRY_READ)
		prot |= VM_PROT_READ;
	if (eflags & IOMMU_MAP_ENTRY_WRITE)
		prot |= VM_PROT_WRITE;

	error = IOMMU_MAP(iommu->dev, domain, va, pa, size, prot);
	if (error) {
		iommu_map_free_entry(iodom, entry);
		return (error);
	}

	*res = entry;

	return (0);
}

void
iommu_domain_unload(struct iommu_domain *iodom,
    struct iommu_map_entries_tailq *entries, bool free)
{
	struct smmu_unit *iommu;
	struct smmu_domain *domain;
	struct iommu_map_entry *entry, *entry1;
	size_t size;
	int error;

	iommu = (struct smmu_unit *)iodom->iommu;
	domain = (struct smmu_domain *)iodom;

	TAILQ_FOREACH_SAFE(entry, entries, dmamap_link, entry1) {
		TAILQ_REMOVE(entries, entry, dmamap_link);
		size = entry->end - entry->start;
		error = IOMMU_UNMAP(iommu->dev, domain,
		    entry->start, size);
		if (error == 0)
			vmem_free(domain->vmem, entry->start, size);
		iommu_map_free_entry(iodom, entry);
	};
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
iommu_lookup(intptr_t xref)
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
	iommu = iommu_lookup(xref);
	if (iommu == NULL) {
		/* SMMU device is not registered in the IOMMU framework. */
		return (NULL);
	}

	return ((struct iommu_unit *)iommu);
}

static void
iommu_init(void)
{

	mtx_init(&iommu_mtx, "IOMMU", NULL, MTX_DEF);

	iommu_map_entry_zone = uma_zcreate("IOMMU_MAP_ENTRY",
	    sizeof(struct iommu_map_entry), NULL, NULL,
	    NULL, NULL, UMA_ALIGN_PTR, UMA_ZONE_NODUMP);
}

SYSINIT(iommu, SI_SUB_DRIVERS, SI_ORDER_FIRST, iommu_init, NULL);
