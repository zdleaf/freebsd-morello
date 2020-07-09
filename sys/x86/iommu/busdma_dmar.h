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

#ifndef __X86_IOMMU_BUSDMA_DMAR_H
#define __X86_IOMMU_BUSDMA_DMAR_H

struct iommu_map_entry;
TAILQ_HEAD(iommu_map_entries_tailq, iommu_map_entry);

struct bus_dma_tag_iommu {
	struct bus_dma_tag_common common;
	struct iommu_ctx *ctx;
	device_t owner;
	int map_count;
	bus_dma_segment_t *segments;
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

#define	BUS_DMAMAP_IOMMU_MALLOC	0x0001
#define	BUS_DMAMAP_IOMMU_KMEM_ALLOC 0x0002

extern struct bus_dma_impl bus_dma_iommu_impl;

struct iommu_unit {
	struct mtx lock;
	int unit;

	int dma_enabled;

	/* Busdma delayed map load */
	struct task dmamap_load_task;
	TAILQ_HEAD(, bus_dmamap_iommu) delayed_maps;
	struct taskqueue *delayed_taskqueue;
};

struct iommu_domain {
	struct iommu_unit *iommu;	/* (c) */
	struct mtx lock;		/* (c) */
	struct task unload_task;	/* (c) */
	struct iommu_map_entries_tailq unload_entries; /* (d) Entries to
							 unload */
};

struct iommu_ctx {
	struct iommu_domain *domain;	/* (c) */
	struct bus_dma_tag_iommu tag;	/* (c) Root tag */
	u_long loads;			/* atomic updates, for stat only */
	u_long unloads;			/* same */
	u_int flags;			/* (u) */
};

#define	IOMMU_LOCK(unit)		mtx_lock(&(unit)->lock)
#define	IOMMU_UNLOCK(unit)		mtx_unlock(&(unit)->lock)
#define	IOMMU_ASSERT_LOCKED(unit)	mtx_assert(&(unit)->lock, MA_OWNED)

#define	IOMMU_DOMAIN_LOCK(dom)		mtx_lock(&(dom)->lock)
#define	IOMMU_DOMAIN_UNLOCK(dom)	mtx_unlock(&(dom)->lock)
#define	IOMMU_DOMAIN_ASSERT_LOCKED(dom)	mtx_assert(&(dom)->lock, MA_OWNED)

bus_dma_tag_t acpi_iommu_get_dma_tag(device_t dev, device_t child);

struct iommu_ctx *iommu_get_ctx(struct iommu_unit *, device_t dev,
    uint16_t rid, bool id_mapped, bool rmrr_init);
struct iommu_unit *iommu_find(device_t dev, bool verbose);
void iommu_domain_unload_entry(struct iommu_map_entry *entry, bool free);
void iommu_domain_unload(struct iommu_domain *domain,
    struct iommu_map_entries_tailq *entries, bool cansleep);

#endif
