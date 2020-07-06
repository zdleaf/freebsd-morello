/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2013 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Konstantin Belousov <kib@FreeBSD.org>
 * under sponsorship from the FreeBSD Foundation.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/domainset.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/lock.h>
#include <sys/proc.h>
#include <sys/memdesc.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/rman.h>
#include <sys/taskqueue.h>
#include <sys/tree.h>
#include <sys/uio.h>
#include <sys/vmem.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_map.h>
#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>
#include <x86/include/busdma_impl.h>
#include <x86/iommu/intel_reg.h>
#include <x86/iommu/busdma_dmar.h>
#include <x86/iommu/intel_dmar.h>

bool
bus_dma_dmar_set_buswide(device_t dev)
{
	struct iommu_unit *dmar;
	device_t parent;
	u_int busno, slot, func;

	parent = device_get_parent(dev);
	if (device_get_devclass(parent) != devclass_find("pci"))
		return (false);
	dmar = dmar_find(dev, bootverbose);
	if (dmar == NULL)
		return (false);
	busno = pci_get_bus(dev);
	slot = pci_get_slot(dev);
	func = pci_get_function(dev);
	if (slot != 0 || func != 0) {
		if (bootverbose) {
			device_printf(dev,
			    "dmar%d pci%d:%d:%d requested buswide busdma\n",
			    dmar->unit, busno, slot, func);
		}
		return (false);
	}
	dmar_set_buswide_ctx(dmar, busno);
	return (true);
}

int
bus_dma_dmar_load_ident(bus_dma_tag_t dmat, bus_dmamap_t map1,
    vm_paddr_t start, vm_size_t length, int flags)
{
	struct bus_dma_tag_common *tc;
	struct bus_dma_tag_iommu *tag;
	struct bus_dmamap_iommu *map;
	struct iommu_device *ctx;
	struct iommu_domain *domain;
	struct iommu_map_entry *entry;
	vm_page_t *ma;
	vm_size_t i;
	int error;
	bool waitok;

	MPASS((start & PAGE_MASK) == 0);
	MPASS((length & PAGE_MASK) == 0);
	MPASS(length > 0);
	MPASS(start + length >= start);
	MPASS((flags & ~(BUS_DMA_NOWAIT | BUS_DMA_NOWRITE)) == 0);

	tc = (struct bus_dma_tag_common *)dmat;
	if (tc->impl != &bus_dma_iommu_impl)
		return (0);

	tag = (struct bus_dma_tag_iommu *)dmat;
	ctx = tag->device;
	domain = ctx->domain;
	map = (struct bus_dmamap_iommu *)map1;
	waitok = (flags & BUS_DMA_NOWAIT) != 0;

	entry = dmar_gas_alloc_entry(domain, waitok ? 0 : DMAR_PGF_WAITOK);
	if (entry == NULL)
		return (ENOMEM);
	entry->start = start;
	entry->end = start + length;
	ma = malloc(sizeof(vm_page_t) * atop(length), M_TEMP, waitok ?
	    M_WAITOK : M_NOWAIT);
	if (ma == NULL) {
		dmar_gas_free_entry(domain, entry);
		return (ENOMEM);
	}
	for (i = 0; i < atop(length); i++) {
		ma[i] = vm_page_getfake(entry->start + PAGE_SIZE * i,
		    VM_MEMATTR_DEFAULT);
	}
	error = dmar_gas_map_region(domain, entry, IOMMU_MAP_ENTRY_READ |
	    ((flags & BUS_DMA_NOWRITE) ? 0 : IOMMU_MAP_ENTRY_WRITE),
	    waitok ? IOMMU_MF_CANWAIT : 0, ma);
	if (error == 0) {
		IOMMU_DOMAIN_LOCK(domain);
		TAILQ_INSERT_TAIL(&map->map_entries, entry, dmamap_link);
		entry->flags |= IOMMU_MAP_ENTRY_MAP;
		IOMMU_DOMAIN_UNLOCK(domain);
	} else {
		dmar_domain_unload_entry(entry, true);
	}
	for (i = 0; i < atop(length); i++)
		vm_page_putfake(ma[i]);
	free(ma, M_TEMP);
	return (error);
}

int
iommu_unmap(struct iommu_domain *domain,
    struct iommu_map_entries_tailq *entries, bool cansleep)
{

	dmar_domain_unload(domain, entries, cansleep);

	return (0);
}

int
iommu_map(struct iommu_domain *domain,
    const struct bus_dma_tag_common *common,
    bus_size_t size, int offset,
    int eflags, int iommu_flags,
    vm_page_t *ma, struct iommu_map_entry **entry)
{
	int ret;

	ret = dmar_gas_map(domain, common, (dmar_gaddr_t)size, offset,
	    eflags, iommu_flags, ma, entry);

	return (ret);
}

struct iommu_device *
iommu_get_device(struct iommu_unit *iommu, device_t dev,
    uint16_t rid, bool id_mapped, bool rmrr_init)
{

	return (dmar_get_ctx_for_dev(iommu, dev, rid, id_mapped, rmrr_init));
}


int
iommu_free_device(struct iommu_device *device)
{

	dmar_free_ctx(device);

	return (0);
}

int
iommu_free_device_locked(struct iommu_unit *dmar, struct iommu_device *ctx)
{

	dmar_free_ctx_locked(dmar, ctx);

	return (0);
}

struct iommu_unit *
iommu_find(device_t dev, bool verbose)
{

	return (dmar_find(dev, verbose));
}
