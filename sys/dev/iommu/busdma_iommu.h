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

#include <dev/iommu/iommu.h>

struct bus_dma_tag_iommu {
	struct bus_dma_tag_common common;
	struct iommu_device *device;
	device_t owner;
	int map_count;
	bus_dma_segment_t *segments;
	//struct iommu_domain *iommu_domain;
};

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

#if 0
struct iommu_ctx {
	struct iommu_domain *domain;
	struct bus_dma_tag_iommu ctx_tag;
	u_long loads;
	u_long unloads;
};
#endif

#define	DMAR_LOCK(n)
#define	DMAR_UNLOCK(n)
#define	DMAR_DOMAIN_LOCK(n)
#define	DMAR_DOMAIN_UNLOCK(n)
#define	DMAR_PAGE_SIZE	4096
#define	DMAR_PAGE_MASK	(DMAR_PAGE_SIZE - 1)

#define	BUS_DMAMAP_DMAR_MALLOC	0x0001
#define	BUS_DMAMAP_DMAR_KMEM_ALLOC 0x0002

extern struct bus_dma_impl bus_dma_iommu_impl;

typedef uint64_t iommu_gaddr_t;

bus_dma_tag_t acpi_iommu_get_dma_tag(device_t dev, device_t child);

struct iommu_qi_genseq {
	u_int gen;
	uint32_t seq;
};

struct iommu_map_entry {
	iommu_gaddr_t start;
	iommu_gaddr_t end;
	iommu_gaddr_t first;		/* Least start in subtree */
	iommu_gaddr_t last;		/* Greatest end in subtree */
	iommu_gaddr_t free_down;		/* Max free space below the
					   current R/B tree node */
	u_int flags;
	TAILQ_ENTRY(iommu_map_entry) dmamap_link; /* Link for dmamap entries */
	RB_ENTRY(iommu_map_entry) rb_entry;	 /* Links for domain entries */
	TAILQ_ENTRY(iommu_map_entry) unroll_link; /* Link for unroll after
						    dmamap_load failure */
	struct iommu_domain *domain;
	struct iommu_qi_genseq gseq;
};

RB_HEAD(iommu_gas_entries_tree, iommu_map_entry);
RB_PROTOTYPE(iommu_gas_entries_tree, iommu_map_entry, rb_entry,
    iommu_gas_cmp_entries);

#define	DMAR_MAP_ENTRY_PLACE	0x0001	/* Fake entry */
#define	DMAR_MAP_ENTRY_RMRR	0x0002	/* Permanent, not linked by
					   dmamap_link */
#define	DMAR_MAP_ENTRY_MAP	0x0004	/* Busdma created, linked by
					   dmamap_link */
#define	DMAR_MAP_ENTRY_UNMAPPED	0x0010	/* No backing pages */
#define	DMAR_MAP_ENTRY_QI_NF	0x0020	/* qi task, do not free entry */
#define	DMAR_MAP_ENTRY_READ	0x1000	/* Read permitted */
#define	DMAR_MAP_ENTRY_WRITE	0x2000	/* Write permitted */
#define	DMAR_MAP_ENTRY_SNOOP	0x4000	/* Snoop */
#define	DMAR_MAP_ENTRY_TM	0x8000	/* Transient */

static inline bool
iommu_test_boundary(iommu_gaddr_t start, iommu_gaddr_t size,
    iommu_gaddr_t boundary)
{

	if (boundary == 0)
		return (true);
	return (start + size <= ((start + boundary) & ~(boundary - 1)));
}

extern struct timespec iommu_hw_timeout;

#define	DMAR_WAIT_UNTIL(cond)					\
{								\
	struct timespec last, curr;				\
	bool forever;						\
								\
	if (iommu_hw_timeout.tv_sec == 0 &&			\
	    iommu_hw_timeout.tv_nsec == 0) {			\
		forever = true;					\
	} else {						\
		forever = false;				\
		nanouptime(&curr);				\
		timespecadd(&curr, &iommu_hw_timeout, &last);	\
	}							\
	for (;;) {						\
		if (cond) {					\
			error = 0;				\
			break;					\
		}						\
		nanouptime(&curr);				\
		if (!forever && timespeccmp(&last, &curr, <)) {	\
			error = ETIMEDOUT;			\
			break;					\
		}						\
		cpu_spinwait();					\
	}							\
}

#if 0
struct iommu_domain {
	int domain;			/* (c) DID, written in context entry */
	int mgaw;			/* (c) Real max address width */
	int agaw;			/* (c) Adjusted guest address width */
	int pglvl;			/* (c) The pagelevel */
	int awlvl;			/* (c) The pagelevel as the bitmask,
					   to set in context entry */
	iommu_gaddr_t end;		/* (c) Highest address + 1 in
					   the guest AS */
	u_int ctx_cnt;			/* (u) Number of contexts owned */
	u_int refs;			/* (u) Refs, including ctx */
	struct iommu_unit *iommu;		/* (c) */
	struct mtx lock;		/* (c) */
	LIST_ENTRY(iommu_domain) link;	/* (u) Member in the iommu list */
	LIST_HEAD(, iommu_ctx) contexts;	/* (u) */
	vm_object_t pgtbl_obj;		/* (c) Page table pages */
	u_int flags;			/* (u) */
	u_int entries_cnt;		/* (d) */
	struct iommu_gas_entries_tree rb_root; /* (d) */
	struct iommu_map_entries_tailq unload_entries; /* (d) Entries to
							 unload */
	struct iommu_map_entry *first_place, *last_place; /* (d) */
	struct task unload_task;	/* (c) */
	u_int batch_no;
};
#endif

int iommu_init_busdma(struct iommu_unit *unit);
void iommu_fini_busdma(struct iommu_unit *unit);

bus_dma_tag_t smmu_get_dma_tag(device_t dev, device_t child);
int busdma_smmu_domain_free(struct bus_dma_tag_iommu *dmat);

#endif /* !_DEV_IOMMU_BUSDMA_IOMMU_H_*/
