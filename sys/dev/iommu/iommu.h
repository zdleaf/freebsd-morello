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

#include <sys/mutex.h>
#include <sys/bus_dma.h>

/* Consumer device */
struct iommu_device {
	LIST_ENTRY(iommu_device)	next;
	device_t dev;
	uint16_t rid;
};

struct iommu_domain {
	LIST_HEAD(, iommu_device)	device_list;
	LIST_ENTRY(iommu_domain)	next;
	struct mtx			mtx_lock;
	bus_dma_tag_t			tag;
};

#define	DOMAIN_LOCK(domain)		mtx_lock(&(domain)->mtx_lock)
#define	DOMAIN_UNLOCK(domain)		mtx_unlock(&(domain)->mtx_lock)
#define	DOMAIN_ASSERT_LOCKED(domain)	\
    mtx_assert(&(domain)->mtx_lock, MA_OWNED)

void iommu_domain_free(struct iommu_domain *domain);
struct iommu_domain * iommu_domain_alloc(void);
struct iommu_domain * iommu_get_domain_for_dev(device_t dev);
void iommu_map(struct iommu_domain *, bus_dma_segment_t *segs, int nsegs);
void iommu_unmap(struct iommu_domain *, bus_dma_segment_t *segs, int nsegs);
int iommu_add_device(struct iommu_domain *domain, device_t dev);
int iommu_register(device_t dev);
int iommu_capable(device_t dev);

#endif /* _DEV_IOMMU_IOMMU_H_ */
