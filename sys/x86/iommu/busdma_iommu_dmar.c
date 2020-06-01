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

/*
 * Given original device, find the requester ID that will be seen by
 * the DMAR unit and used for page table lookup.  PCI bridges may take
 * ownership of transactions from downstream devices, so it may not be
 * the same as the BSF of the target device.  In those cases, all
 * devices downstream of the bridge must share a single mapping
 * domain, and must collectively be assigned to use either DMAR or
 * bounce mapping.
 */
device_t
dmar_get_requester(device_t dev, uint16_t *rid)
{
	devclass_t pci_class;
	device_t l, pci, pcib, pcip, pcibp, requester;
	int cap_offset;
	uint16_t pcie_flags;
	bool bridge_is_pcie;

	pci_class = devclass_find("pci");
	l = requester = dev;

	*rid = pci_get_rid(dev);

	/*
	 * Walk the bridge hierarchy from the target device to the
	 * host port to find the translating bridge nearest the DMAR
	 * unit.
	 */
	for (;;) {
		pci = device_get_parent(l);
		KASSERT(pci != NULL, ("dmar_get_requester(%s): NULL parent "
		    "for %s", device_get_name(dev), device_get_name(l)));
		KASSERT(device_get_devclass(pci) == pci_class,
		    ("dmar_get_requester(%s): non-pci parent %s for %s",
		    device_get_name(dev), device_get_name(pci),
		    device_get_name(l)));

		pcib = device_get_parent(pci);
		KASSERT(pcib != NULL, ("dmar_get_requester(%s): NULL bridge "
		    "for %s", device_get_name(dev), device_get_name(pci)));

		/*
		 * The parent of our "bridge" isn't another PCI bus,
		 * so pcib isn't a PCI->PCI bridge but rather a host
		 * port, and the requester ID won't be translated
		 * further.
		 */
		pcip = device_get_parent(pcib);
		if (device_get_devclass(pcip) != pci_class)
			break;
		pcibp = device_get_parent(pcip);

		if (pci_find_cap(l, PCIY_EXPRESS, &cap_offset) == 0) {
			/*
			 * Do not stop the loop even if the target
			 * device is PCIe, because it is possible (but
			 * unlikely) to have a PCI->PCIe bridge
			 * somewhere in the hierarchy.
			 */
			l = pcib;
		} else {
			/*
			 * Device is not PCIe, it cannot be seen as a
			 * requester by DMAR unit.  Check whether the
			 * bridge is PCIe.
			 */
			bridge_is_pcie = pci_find_cap(pcib, PCIY_EXPRESS,
			    &cap_offset) == 0;
			requester = pcib;

			/*
			 * Check for a buggy PCIe/PCI bridge that
			 * doesn't report the express capability.  If
			 * the bridge above it is express but isn't a
			 * PCI bridge, then we know pcib is actually a
			 * PCIe/PCI bridge.
			 */
			if (!bridge_is_pcie && pci_find_cap(pcibp,
			    PCIY_EXPRESS, &cap_offset) == 0) {
				pcie_flags = pci_read_config(pcibp,
				    cap_offset + PCIER_FLAGS, 2);
				if ((pcie_flags & PCIEM_FLAGS_TYPE) !=
				    PCIEM_TYPE_PCI_BRIDGE)
					bridge_is_pcie = true;
			}

			if (bridge_is_pcie) {
				/*
				 * The current device is not PCIe, but
				 * the bridge above it is.  This is a
				 * PCIe->PCI bridge.  Assume that the
				 * requester ID will be the secondary
				 * bus number with slot and function
				 * set to zero.
				 *
				 * XXX: Doesn't handle the case where
				 * the bridge is PCIe->PCI-X, and the
				 * bridge will only take ownership of
				 * requests in some cases.  We should
				 * provide context entries with the
				 * same page tables for taken and
				 * non-taken transactions.
				 */
				*rid = PCI_RID(pci_get_bus(l), 0, 0);
				l = pcibp;
			} else {
				/*
				 * Neither the device nor the bridge
				 * above it are PCIe.  This is a
				 * conventional PCI->PCI bridge, which
				 * will use the bridge's BSF as the
				 * requester ID.
				 */
				*rid = pci_get_rid(pcib);
				l = pcib;
			}
		}
	}
	return (requester);
}

#if 0
struct iommu_device *
dmar_instantiate_ctx(struct iommu_unit *dmar, device_t dev, bool rmrr)
{
	device_t requester;
	struct iommu_device *ctx;
	bool disabled;
	uint16_t rid;

	requester = dmar_get_requester(dev, &rid);

	/*
	 * If the user requested the IOMMU disabled for the device, we
	 * cannot disable the DMAR, due to possibility of other
	 * devices on the same DMAR still requiring translation.
	 * Instead provide the identity mapping for the device
	 * context.
	 */
	disabled = dmar_bus_dma_is_dev_disabled(pci_get_domain(requester), 
	    pci_get_bus(requester), pci_get_slot(requester), 
	    pci_get_function(requester));
	ctx = dmar_get_ctx_for_dev(dmar, requester, rid, disabled, rmrr);
	if (ctx == NULL)
		return (NULL);
	if (disabled) {
		/*
		 * Keep the first reference on context, release the
		 * later refs.
		 */
		IOMMU_LOCK(dmar);
		if ((ctx->flags & IOMMU_CTX_DISABLED) == 0) {
			ctx->flags |= IOMMU_CTX_DISABLED;
			IOMMU_UNLOCK(dmar);
		} else {
			dmar_free_ctx_locked(dmar, ctx);
		}
		ctx = NULL;
	}
	return (ctx);
}

bus_dma_tag_t
acpi_iommu_get_dma_tag(device_t dev, device_t child)
{
	struct iommu_unit *dmar;
	struct iommu_device *ctx;
	bus_dma_tag_t res;

	dmar = iommu_find(child, bootverbose);
	/* Not in scope of any DMAR ? */
	if (dmar == NULL)
		return (NULL);
	if (!dmar->dma_enabled)
		return (NULL);
	dmar_quirks_pre_use(dmar);
	dmar_instantiate_rmrr_ctxs(dmar);

	ctx = dmar_instantiate_ctx(dmar, child, false);
	res = ctx == NULL ? NULL : (bus_dma_tag_t)&ctx->ctx_tag;
	return (res);
}
#endif

bool
bus_dma_dmar_set_buswide(device_t dev)
{
	struct iommu_unit *dmar;
	device_t parent;
	u_int busno, slot, func;

	parent = device_get_parent(dev);
	if (device_get_devclass(parent) != devclass_find("pci"))
		return (false);
	dmar = iommu_find(dev, bootverbose);
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
    size_t size, vm_offset_t offset,
    int eflags, int iommu_flags,
    vm_page_t *ma, struct iommu_map_entry **entry)
{
	int ret;

	ret = dmar_gas_map(domain, common, (iommu_gaddr_t)size, (int)offset,
	    eflags, iommu_flags, ma, entry);

	return (ret);
}
