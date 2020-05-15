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
#include <arm64/iommu/busdma_iommu.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpi_pcibvar.h>
#endif

#include "iommu.h"
#include "iommu_if.h"

static MALLOC_DEFINE(M_IOMMU, "IOMMU", "IOMMU framework");
static MALLOC_DEFINE(M_BUSDMA, "SMMU", "ARM64 busdma SMMU");

static struct mtx iommu_mtx;

#define	IOMMU_LOCK(iommu)		mtx_lock(&(iommu)->mtx_lock)
#define	IOMMU_UNLOCK(iommu)		mtx_unlock(&(iommu)->mtx_lock)
#define	IOMMU_ASSERT_LOCKED(iommu)	mtx_assert(&(iommu)->mtx_lock, MA_OWNED)

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

static LIST_HEAD(, iommu_unit) iommu_list = LIST_HEAD_INITIALIZER(iommu_list);

int
iommu_domain_add_va_range(struct iommu_domain *domain,
    vm_offset_t va, vm_size_t size)
{
	struct iommu_unit *iommu;
	int error;

	KASSERT(size > 0, ("wrong size"));

	iommu = domain->iommu;

	error = vmem_add(domain->vmem, va, size, M_WAITOK);

	return (error);
}

struct iommu_domain *
iommu_domain_alloc(struct iommu_unit *iommu)
{
	struct iommu_domain *domain;

	domain = IOMMU_DOMAIN_ALLOC(iommu->dev);
	if (domain == NULL)
		return (NULL);

	LIST_INIT(&domain->device_list);
	mtx_init(&domain->mtx_lock, "IOMMU domain", NULL, MTX_DEF);
	domain->iommu = iommu;

	domain->vmem = vmem_create("IOMMU vmem", 0, 0, PAGE_SIZE,
	    PAGE_SIZE, M_FIRSTFIT | M_WAITOK);
	if (domain->vmem == NULL)
		return (NULL);

	IOMMU_LOCK(iommu);
	LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
	IOMMU_UNLOCK(iommu);

	return (domain);
}

int
iommu_domain_free(struct iommu_domain *domain)
{
	struct iommu_unit *iommu;
	vmem_t *vmem;
	int error;

	iommu = domain->iommu;
	vmem = domain->vmem;

	IOMMU_LOCK(iommu);
	LIST_REMOVE(domain, next);
	error = IOMMU_DOMAIN_FREE(iommu->dev, domain);
	if (error) {
		LIST_INSERT_HEAD(&iommu->domain_list, domain, next);
		IOMMU_UNLOCK(iommu);
		return (error);
	}

	IOMMU_UNLOCK(iommu);

	vmem_destroy(vmem);

	return (0);
}

struct iommu_domain *
iommu_get_domain_for_dev(device_t dev)
{
	struct iommu_domain *domain;
	struct iommu_device *device;
	struct iommu_unit *iommu;

	LIST_FOREACH(iommu, &iommu_list, next) {
		LIST_FOREACH(domain, &iommu->domain_list, next) {
			LIST_FOREACH(device, &domain->device_list, next) {
				if (device->dev == dev)
					return (domain);
			}
		}
	}

	return (NULL);
}

struct iommu_device *
iommu_get_device_for_dev(device_t dev)
{
	struct iommu_domain *domain;
	struct iommu_device *device;
	struct iommu_unit *iommu;

	LIST_FOREACH(iommu, &iommu_list, next) {
		LIST_FOREACH(domain, &iommu->domain_list, next) {
			LIST_FOREACH(device, &domain->device_list, next) {
				if (device->dev == dev)
					return (device);
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

struct iommu_device *
iommu_get_ctx_for_dev(struct iommu_unit *iommu, device_t requester,
    uint16_t rid, bool disabled, bool rmrr)
{
	struct iommu_device *device;
	struct iommu_domain *domain;
	struct bus_dma_tag_iommu *tag;
	int error;

	device = iommu_get_device_for_dev(requester);
	if (device)
		return (device);

	device = iommu_device_alloc(requester);
	if (device == NULL)
		return (NULL);

	domain = iommu_domain_alloc(iommu);
	if (domain == NULL)
		return (NULL);

	tag = &device->ctx_tag;
	tag->owner = requester;
	tag->device = device;

	iommu_tag_init(tag);

	device->domain = domain;

	error = iommu_device_attach(domain, device);
	if (error) {
		free(tag, M_BUSDMA);
		iommu_domain_free(domain);
		return (NULL);
	}

	/* Add some virtual address range for this domain. */
	iommu_domain_add_va_range(domain, 0x40000000, 0x40000000);

	/* Map the GICv3 ITS page so the device could send MSI interrupts. */
	iommu_map_page(domain, GICV3_ITS_PAGE, GICV3_ITS_PAGE, VM_PROT_WRITE);

	return (device);
}

struct iommu_device *
iommu_device_alloc(device_t dev)
{
	struct iommu_device *device;

	device = malloc(sizeof(*device), M_IOMMU, M_WAITOK | M_ZERO);
	device->rid = pci_get_rid(dev);
	device->dev = dev;

	return (device);
}

/*
 * Attach a consumer device to a domain.
 */
int
iommu_device_attach(struct iommu_domain *domain, struct iommu_device *device)
{
	struct iommu_unit *iommu;
	int err;

	iommu = domain->iommu;

	err = IOMMU_DEVICE_ATTACH(iommu->dev, domain, device);
	if (err) {
		device_printf(iommu->dev, "Failed to add device\n");
		free(device, M_IOMMU);
		return (err);
	}

	device->domain = domain;

	DOMAIN_LOCK(domain);
	LIST_INSERT_HEAD(&domain->device_list, device, next);
	DOMAIN_UNLOCK(domain);

	return (err);
}

int
iommu_free_ctx_locked(struct iommu_unit *iommu, struct iommu_device *device)
{
	struct iommu_domain *domain;
	int err;

	domain = device->domain;

	err = IOMMU_DEVICE_DETACH(iommu->dev, device);
	if (err) {
		device_printf(iommu->dev, "Failed to remove device\n");
		return (err);
	}

	LIST_REMOVE(device, next);

	return (0);
}

int
iommu_free_ctx(struct iommu_device *device)
{
	struct iommu_domain *domain;
	int error;

	printf("%s\n", __func__);

	domain = device->domain;

	DOMAIN_LOCK(domain);
	error = iommu_free_ctx_locked(domain->iommu, device);
	DOMAIN_UNLOCK(domain);

	return (error);
}

int
iommu_map_page(struct iommu_domain *domain,
    vm_offset_t va, vm_paddr_t pa, vm_prot_t prot)
{
	struct iommu_unit *iommu;
	int error;

	iommu = domain->iommu;

	error = IOMMU_MAP(iommu->dev, domain, va, pa, PAGE_SIZE, prot);
	if (error)
		return (error);

	return (0);
}

int
iommu_unmap_page(struct iommu_domain *domain, vm_offset_t va)
{
	struct iommu_unit *iommu;
	int error;

	iommu = domain->iommu;

	error = IOMMU_UNMAP(iommu->dev, domain, va, PAGE_SIZE);
	if (error)
		return (error);

	return (0);
}

static uma_zone_t iommu_map_entry_zone;

static void
intel_gas_init(void)
{

	iommu_map_entry_zone = uma_zcreate("DMAR_MAP_ENTRY",
	    sizeof(struct iommu_map_entry), NULL, NULL,
	    NULL, NULL, UMA_ALIGN_PTR, UMA_ZONE_NODUMP);
}
SYSINIT(intel_gas, SI_SUB_DRIVERS, SI_ORDER_FIRST, intel_gas_init, NULL);

static struct iommu_map_entry *
iommu_gas_alloc_entry(struct iommu_domain *domain, u_int flags)
{
	struct iommu_map_entry *res;

	KASSERT((flags & ~(DMAR_PGF_WAITOK)) == 0,
	    ("unsupported flags %x", flags));

	res = uma_zalloc(iommu_map_entry_zone, ((flags & DMAR_PGF_WAITOK) !=
	    0 ? M_WAITOK : M_NOWAIT) | M_ZERO);
	if (res != NULL) {
		res->domain = domain;
		atomic_add_int(&domain->entries_cnt, 1);
	}
	return (res);
}

static void
iommu_gas_free_entry(struct iommu_domain *domain, struct iommu_map_entry *entry)
{

	KASSERT(domain == entry->domain,
	    ("mismatched free domain %p entry %p entry->domain %p", domain,
	    entry, entry->domain));
	atomic_subtract_int(&domain->entries_cnt, 1);
	uma_zfree(iommu_map_entry_zone, entry);
}

int
iommu_map1(struct iommu_domain *domain, vm_size_t size, vm_offset_t offset,
    vm_prot_t prot, vm_page_t *ma, struct iommu_map_entry **res)
{
	struct iommu_map_entry *entry;
	struct iommu_unit *iommu;
	vm_offset_t va;
	vm_paddr_t pa;
	int error;

	iommu = domain->iommu;

	entry = iommu_gas_alloc_entry(domain, 0);

	error = vmem_alloc(domain->vmem, size,
	    M_FIRSTFIT | M_NOWAIT, &va);

	pa = VM_PAGE_TO_PHYS(ma[0]);

	entry->start = va;
	entry->end = va + size;

	error = IOMMU_MAP(iommu->dev, domain, va, pa, size, prot);

	*res = entry;

	return (0);
}

int
iommu_unmap1(struct iommu_domain *domain,
    struct iommu_map_entries_tailq *entries, bool free)
{
	struct iommu_map_entry *entry, *entry1;

	TAILQ_FOREACH_SAFE(entry, entries, dmamap_link, entry1) {
		TAILQ_REMOVE(entries, entry, dmamap_link);
		iommu_gas_free_entry(domain, entry);
	};

	return (0);
}

int
iommu_register(device_t dev, struct iommu_unit *iommu, intptr_t xref)
{

	iommu->dev = dev;
	iommu->xref = xref;

	LIST_INIT(&iommu->domain_list);
	mtx_init(&iommu->mtx_lock, "IOMMU", NULL, MTX_DEF);

	IOMMU_LIST_LOCK();
	LIST_INSERT_HEAD(&iommu_list, iommu, next);
	IOMMU_LIST_UNLOCK();

	iommu_init_busdma(iommu);

	return (0);
}

int
iommu_unregister(device_t dev)
{
	struct iommu_unit *iommu;
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

struct iommu_unit *
iommu_lookup(intptr_t xref, int flags)
{
	struct iommu_unit *iommu;

	LIST_FOREACH(iommu, &iommu_list, next) {
		if (iommu->xref == xref)
			return (iommu);
	}

	return (NULL);
}

struct iommu_unit *
iommu_find(device_t dev, bool verbose)
{
	struct iommu_unit *iommu;
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
	 * Find the registered IOMMU controller by xref.
	 */
	iommu = iommu_lookup(xref, 0);
	if (iommu == NULL) {
		/* SMMU device is not registered in the IOMMU framework. */
		return (NULL);
	}

	return (iommu);
}

static void
iommu_init(void)
{

	mtx_init(&iommu_mtx, "IOMMU", NULL, MTX_DEF);
}

SYSINIT(iommu, SI_SUB_DRIVERS, SI_ORDER_FIRST, iommu_init, NULL);
