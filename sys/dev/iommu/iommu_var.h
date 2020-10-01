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

#ifndef _SYS_IOMMU_VAR_H_
#define _SYS_IOMMU_VAR_H_

struct iommu_unit {
	struct mtx lock;
	int unit;

	int dma_enabled;

	/* Busdma delayed map load */
	struct task dmamap_load_task;
	TAILQ_HEAD(, bus_dmamap_iommu) delayed_maps;
	struct taskqueue *delayed_taskqueue;

	/*
	 * Bitmap of buses for which context must ignore slot:func,
	 * duplicating the page table pointer into all context table
	 * entries.  This is a client-controlled quirk to support some
	 * NTBs.
	 */
	uint32_t buswide_ctxs[(PCI_BUSMAX + 1) / NBBY / sizeof(uint32_t)];
};

/*
 * Locking annotations:
 * (u) - Protected by iommu unit lock
 * (d) - Protected by domain lock
 * (c) - Immutable after initialization
 */

struct iommu_domain {
	struct iommu_unit *iommu;	/* (c) */
	const struct iommu_domain_map_ops *ops;
	struct mtx lock;		/* (c) */
	struct task unload_task;	/* (c) */
	u_int entries_cnt;		/* (d) */
	struct iommu_map_entries_tailq unload_entries; /* (d) Entries to
							 unload */
	struct iommu_gas_entries_tree rb_root; /* (d) */
	iommu_gaddr_t end;		/* (c) Highest address + 1 in
					   the guest AS */
	struct iommu_map_entry *first_place, *last_place; /* (d) */
	u_int flags;			/* (u) */
};

struct iommu_ctx {
	struct iommu_domain *domain;	/* (c) */
	struct bus_dma_tag_iommu *tag;	/* (c) Root tag */
	u_long loads;			/* atomic updates, for stat only */
	u_long unloads;			/* same */
	u_int flags;			/* (u) */
	uint16_t rid;			/* (c) pci RID */
};

#endif /* !_SYS_IOMMU_VAR_H_ */
