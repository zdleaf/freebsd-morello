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

#define	IOMMU_PAGE_SIZE		4096
#define	IOMMU_PAGE_MASK		(IOMMU_PAGE_SIZE - 1)

#define	IOMMU_MF_CANWAIT	0x0001
#define	IOMMU_MF_CANSPLIT	0x0002

#define	IOMMU_PGF_WAITOK	0x0001
#define	IOMMU_PGF_ZERO		0x0002
#define	IOMMU_PGF_ALLOC		0x0004
#define	IOMMU_PGF_NOALLOC	0x0008
#define	IOMMU_PGF_OBJL		0x0010

struct smmu_unit {
	struct iommu_unit		unit;
	LIST_HEAD(, smmu_domain)	domain_list;
	LIST_ENTRY(smmu_unit)		next;
	device_t			dev;
	intptr_t			xref;
};

struct smmu_domain {
	struct iommu_domain		domain;
	LIST_HEAD(, smmu_ctx)		ctx_list;
	LIST_ENTRY(smmu_domain)	next;
	u_int entries_cnt;
	struct smmu_cd			*cd;
	struct pmap			p;
	uint16_t			asid;
};

struct smmu_ctx {
	struct iommu_ctx		ctx;
	struct smmu_domain		*domain;
	LIST_ENTRY(smmu_ctx)		next;
	device_t dev;
	uint16_t rid;
	bool bypass;
	int sid;
};

int iommu_unregister(device_t dev);
int iommu_register(device_t dev, struct smmu_unit *unit, intptr_t xref);
int iommu_map_page(struct smmu_domain *domain, vm_offset_t va, vm_paddr_t pa,
    vm_prot_t prot);
int iommu_unmap_page(struct smmu_domain *domain, vm_offset_t va);
void smmu_map_msi(device_t, device_t, uint64_t);

#endif /* _DEV_IOMMU_IOMMU_H_ */
