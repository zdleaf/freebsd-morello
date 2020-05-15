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
 *
 * $FreeBSD$
 */

#ifndef _DEV_IOMMU_BUSDMA_IOMMU_H_
#define _DEV_IOMMU_BUSDMA_IOMMU_H_

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

#include <arm64/iommu/iommu.h>

struct bus_dmamap_iommu {
	struct bus_dma_tag_iommu *tag;
	struct memdesc mem;
	bus_dmamap_callback_t *callback;
	void *callback_arg;
	struct iommu_map_entries_tailq map_entries;
	TAILQ_ENTRY(bus_dmamap_iommu) delay_link;
	bool locked;
	bool cansleep;
	int flags;
};

#define	BUS_DMAMAP_IOMMU_MALLOC		0x0001
#define	BUS_DMAMAP_IOMMU_KMEM_ALLOC	0x0002

extern struct bus_dma_impl bus_dma_iommu_impl;

typedef uint64_t iommu_gaddr_t;

bus_dma_tag_t acpi_iommu_get_dma_tag(device_t dev, device_t child);

struct iommu_map_entry {
	iommu_gaddr_t start;
	iommu_gaddr_t end;
	u_int flags;
	TAILQ_ENTRY(iommu_map_entry) dmamap_link; /* Link for dmamap entries */
	TAILQ_ENTRY(iommu_map_entry) unroll_link; /* Link for unroll after
						    dmamap_load failure */
	struct iommu_domain *domain;
};

static inline bool
iommu_test_boundary(iommu_gaddr_t start, iommu_gaddr_t size,
    iommu_gaddr_t boundary)
{

	if (boundary == 0)
		return (true);
	return (start + size <= ((start + boundary) & ~(boundary - 1)));
}

#endif /* !_DEV_IOMMU_BUSDMA_IOMMU_H_*/
