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
 *
 * $FreeBSD$
 */

#ifndef _DEV_IOMMU_IOMMU_H_
#define _DEV_IOMMU_IOMMU_H_

#include <machine/bus.h>

#include <sys/mutex.h>
#include <sys/bus_dma.h>
#include <sys/vmem.h>

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

#include <vm/vm.h>
#include <vm/pmap.h>

#if 0
#include <dev/iommu/busdma_iommu.h>
#endif
#include <machine/bus.h>
#include <machine/bus_dma_impl.h>
#include <sys/iommu.h>
#include <arm64/iommu/iommu.h>

#define	IOMMU_PAGE_SIZE		4096
#define	IOMMU_PAGE_MASK		(IOMMU_PAGE_SIZE - 1)

#define	IOMMU_MF_CANWAIT	0x0001
#define	IOMMU_MF_CANSPLIT	0x0002

#define	IOMMU_PGF_WAITOK	0x0001
#define	IOMMU_PGF_ZERO		0x0002
#define	IOMMU_PGF_ALLOC		0x0004
#define	IOMMU_PGF_NOALLOC	0x0008
#define	IOMMU_PGF_OBJL		0x0010

#if 0
#define	IOMMU_MAP_ENTRY_PLACE	0x0001	/* Fake entry */
#define	IOMMU_MAP_ENTRY_RMRR	0x0002	/* Permanent, not linked by
					   dmamap_link */
#define	IOMMU_MAP_ENTRY_MAP	0x0004	/* Busdma created, linked by
					   dmamap_link */
#define	IOMMU_MAP_ENTRY_UNMAPPED 0x0010	/* No backing pages */
#define	IOMMU_MAP_ENTRY_QI_NF	0x0020	/* qi task, do not free entry */
#define	IOMMU_MAP_ENTRY_READ	0x1000	/* Read permitted */
#define	IOMMU_MAP_ENTRY_WRITE	0x2000	/* Write permitted */
#define	IOMMU_MAP_ENTRY_SNOOP	0x4000	/* Snoop */
#define	IOMMU_MAP_ENTRY_TM	0x8000	/* Transient */

#define	IOMMU_LOCK(iommu)		mtx_lock(&(iommu)->mtx_lock)
#define	IOMMU_UNLOCK(iommu)		mtx_unlock(&(iommu)->mtx_lock)
#define	IOMMU_ASSERT_LOCKED(iommu)	\
    mtx_assert(&(iommu)->mtx_lock, MA_OWNED)

#define	IOMMU_DOMAIN_LOCK(domain)		mtx_lock(&(domain)->mtx_lock)
#define	IOMMU_DOMAIN_UNLOCK(domain)		mtx_unlock(&(domain)->mtx_lock)
#define	IOMMU_DOMAIN_ASSERT_LOCKED(domain)	\
    mtx_assert(&(domain)->mtx_lock, MA_OWNED)

typedef uint64_t iommu_gaddr_t;

extern struct bus_dma_impl bus_dma_iommu_impl;

struct iommu_map_entry {
	iommu_gaddr_t start;
	iommu_gaddr_t end;
	iommu_gaddr_t size;
	u_int flags;
	TAILQ_ENTRY(iommu_map_entry) dmamap_link; /* Link for dmamap entries */
	TAILQ_ENTRY(iommu_map_entry) unroll_link; /* Link for unroll after
						    dmamap_load failure */
	struct iommu_domain *domain;
};
#endif

struct iommu1_unit {
	struct iommu_unit		unit;
	LIST_HEAD(, iommu1_domain)	domain_list;
	LIST_ENTRY(iommu1_unit)		next;
	device_t			dev;
	intptr_t			xref;

#if 0
	int unit;
	int dma_enabled;

	/* Delayed freeing of map entries queue processing */
	struct iommu_map_entries_tailq tlb_flush_entries;
	struct task qi_task;
	struct taskqueue *qi_taskqueue;

	/* Busdma delayed map load */
	struct task dmamap_load_task;
	TAILQ_HEAD(, bus_dmamap_iommu) delayed_maps;
	struct taskqueue *delayed_taskqueue;
#endif
};

struct iommu1_domain {
	struct iommu_domain		domain;
	LIST_HEAD(, iommu1_ctx)		ctx_list;
	LIST_ENTRY(iommu1_domain)	next;
	vmem_t				*vmem;
	struct iommu1_unit		*iommu;
	u_int entries_cnt;
};

/* Consumer device. */
struct iommu1_ctx {
	struct iommu_ctx		ctx;
	LIST_ENTRY(iommu1_ctx)		next;
	device_t dev;
	uint16_t rid;
	bool bypass;
	struct iommu1_domain		*domain;
};

#if 0
static inline bool
iommu_test_boundary(iommu_gaddr_t start, iommu_gaddr_t size,
    iommu_gaddr_t boundary)
{

	if (boundary == 0)
		return (true);
	return (start + size <= ((start + boundary) & ~(boundary - 1)));
}
#endif

int iommu_map_page(struct iommu1_domain *domain,
    vm_offset_t va, vm_paddr_t pa, vm_prot_t prot);
int iommu_unmap_page(struct iommu1_domain *domain, vm_offset_t va);

int iommu_register(device_t dev, struct iommu1_unit *unit, intptr_t xref);
int iommu_unregister(device_t dev);

#endif /* _DEV_IOMMU_IOMMU_H_ */
