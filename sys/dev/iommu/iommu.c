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
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/iommu/smmu_var.h>
#include <dev/pci/pcivar.h>

#include "iommu.h"
#include "iommu_if.h"

static MALLOC_DEFINE(M_IOMMU, "iommu", "IOMMU");

static device_t iommu_dev = NULL;

static struct mtx iommu_mtx;

#define	IOMMU_LOCK()			mtx_lock(&iommu_mtx)
#define	IOMMU_UNLOCK()			mtx_unlock(&iommu_mtx)
#define	IOMMU_ASSERT_LOCKED()		mtx_assert(&iommu_mtx, MA_OWNED)

static LIST_HEAD(, iommu_domain) domain_list =
    LIST_HEAD_INITIALIZER(domain_list);

struct iommu_domain *
iommu_domain_alloc(void)
{
	struct iommu_domain *domain;

	domain = IOMMU_DOMAIN_ALLOC(iommu_dev);
	if (domain == NULL)
		return (NULL);

	LIST_INIT(&domain->device_list);
	mtx_init(&domain->mtx_lock, "IOMMU domain", NULL, MTX_DEF);

	IOMMU_LOCK();
	LIST_INSERT_HEAD(&domain_list, domain, next);
	IOMMU_UNLOCK();

	domain->vmem = vmem_create("IOMMU vmem", 0, 0, PAGE_SIZE,
	    PAGE_SIZE, M_FIRSTFIT | M_WAITOK);
	if (domain->vmem == NULL)
		return (NULL);

	/* 1GB of VA space starting from 0x40000000. */
	vmem_add(domain->vmem, 0x40000000, 0x40000000, 0);

	printf("%s: vmem initialized at addr %p\n", __func__, domain->vmem);

	return (domain);
}

void
iommu_domain_free(struct iommu_domain *domain)
{

	/* TODO */
}

struct iommu_domain *
iommu_get_domain_for_dev(device_t dev)
{
	struct iommu_domain *domain;
	struct iommu_device *device;

	LIST_FOREACH(domain, &domain_list, next) {
		LIST_FOREACH(device, &domain->device_list, next) {
			if (device->dev == dev)
				return (domain);
		}
	}

	return (NULL);
}

/*
 * Add a consumer device to a domain.
 */
int
iommu_add_device(struct iommu_domain *domain, device_t dev)
{
	struct iommu_device *device;
	int err;

	device = malloc(sizeof(*device), M_IOMMU, M_WAITOK | M_ZERO);
	device->rid = pci_get_rid(dev);
	device->dev = dev;

	err = IOMMU_ADD_DEVICE(iommu_dev, domain, device);
	if (err) {
		printf("Failed to add device\n");
		free(device, M_IOMMU);
		return (err);
	}

	DOMAIN_LOCK(domain);
	LIST_INSERT_HEAD(&domain->device_list, device, next);
	DOMAIN_UNLOCK(domain);

	return (err);
}

void
iommu_unmap(struct iommu_domain *domain, bus_dma_segment_t *segs, int nsegs)
{
	vm_offset_t offset;
	vm_offset_t va;
	vm_size_t size;
	int err;
	int i;

	for (i = 0; i < nsegs; i++) {
		va = segs[i].ds_addr & ~0xfff;
		offset = segs[i].ds_addr & 0xfff;
		size = roundup2(offset + segs[i].ds_len, PAGE_SIZE);

		err = IOMMU_UNMAP(iommu_dev, domain, va, size);
		if (err == 0)
			vmem_free(domain->vmem, va, size);
	}
}

void
iommu_map(struct iommu_domain *domain, bus_dma_segment_t *segs, int nsegs)
{
	vm_offset_t offset;
	vm_offset_t va;
	vm_paddr_t pa;
	vm_size_t size;
	int i;

	for (i = 0; i < nsegs; i++) {
		pa = segs[i].ds_addr & ~(PAGE_SIZE - 1);
		offset = segs[i].ds_addr & (PAGE_SIZE - 1);
		size = roundup2(offset + segs[i].ds_len, PAGE_SIZE);

		if ((offset + segs[i].ds_len) > PAGE_SIZE)
			printf("offset %lx len %lx size %lx\n",
			    offset, segs[i].ds_len, size);

		if (vmem_alloc(domain->vmem, size,
		    M_FIRSTFIT | M_NOWAIT, &va))
			panic("Could not allocate virtual address.\n");

		IOMMU_MAP(iommu_dev, domain, pa, va, size);

		segs[i].ds_addr = va | offset;
	}
}

int
iommu_capable(device_t dev)
{
	int err;

	err = IOMMU_CAPABLE(iommu_dev, dev);

	return (err);
}

int
iommu_register(device_t dev)
{

	iommu_dev = dev;

	return (0);
}

static void
iommu_init(void)
{

	mtx_init(&iommu_mtx, "IOMMU", NULL, MTX_DEF);
}

SYSINIT(iommu, SI_SUB_DRIVERS, SI_ORDER_FIRST, iommu_init, NULL);
