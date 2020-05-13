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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/kernel.h>

#include <machine/bus.h>
#include <arm64/include/bus_dma_impl.h>

#include <dev/iommu/iommu.h>
#include <dev/pci/pcivar.h>

#include <dev/iommu/busdma_iommu.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpi_pcibvar.h>
#endif

#define	GICV3_ITS_PAGE	0x300b0000

static MALLOC_DEFINE(M_BUSDMA, "SMMU", "ARM64 busdma SMMU");

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

int
busdma_smmu_domain_free(struct bus_dma_tag_iommu *dmat)
{
	struct iommu_domain *domain;
	int error;

	domain = iommu_get_domain_for_dev(dmat->owner);
	if (domain == NULL)
		return (0);

	error = iommu_device_detach(domain, dmat->owner);
	if (error) {
		device_printf(dmat->owner,
		    "Could not detach a device from IOMMU domain.\n");
		return (error);
	}

	/* Unmap the GICv3 ITS page. */
	struct iommu_device *device;
	device = dmat->device;
	error = iommu_unmap_page(device->domain, GICV3_ITS_PAGE);
	if (error) {
		device_printf(dmat->owner,
		    "Could not unmap GICv3 ITS page.\n");
		return (error);
	}

	error = iommu_domain_free(domain);
	if (error) {
		device_printf(dmat->owner,
		    "Could not deallocate IOMMU domain.\n");
		return (error);
	}

	return (0);
}

static struct iommu_domain *
smmu_domain_alloc(device_t dev)
{
	struct iommu_domain *domain;
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

	domain = iommu_domain_alloc(iommu);
	if (domain == NULL)
		return (NULL);

	/* Add some virtual address range for this domain. */
	iommu_domain_add_va_range(domain, 0x40000000, 0x40000000);

	/* Map the GICv3 ITS page so the device could send MSI interrupts. */
	iommu_map_page(domain, GICV3_ITS_PAGE, GICV3_ITS_PAGE, VM_PROT_WRITE);

	return (domain);
}

#if 0
bus_dma_tag_t
smmu_get_dma_tag(device_t dev, device_t child)
{
	struct iommu_domain *domain;
	devclass_t pci_class;
	struct bus_dma_tag_iommu *tag;
	int error;

	pci_class = devclass_find("pci");
	if (device_get_devclass(device_get_parent(child)) != pci_class)
		return (NULL);

	struct iommu_device *device;
	device = iommu_get_device_for_dev(child);
	if (device)
		return (device->ctx_tag);

#if 0
	domain = iommu_get_domain_for_dev(child);
	if (domain)
		return (domain->tag);
#endif

	/* A single device per domain. */

	domain = smmu_domain_alloc(child);
	if (!domain)
		return (NULL);

	tag = malloc(sizeof(*tag), M_BUSDMA, M_WAITOK | M_ZERO);
	if (!tag) {
		iommu_domain_free(domain);
		return (NULL);
	}

	smmu_tag_init(tag);
	tag->owner = child;
	device->ctx_tag = (bus_dma_tag_t)tag;

	error = iommu_device_attach(domain, child);
	if (error) {
		free(tag, M_BUSDMA);
		iommu_domain_free(domain);
		return (NULL);
	}

	tag->device = iommu_get_device_for_dev(child);
	tag->device->domain = domain;

	return ((bus_dma_tag_t)(tag));
}
#endif

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
