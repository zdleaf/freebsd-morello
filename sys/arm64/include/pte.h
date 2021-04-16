/*-
 * Copyright (c) 2014 Andrew Turner
 * Copyright (c) 2014-2015 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Andrew Turner under
 * sponsorship from the FreeBSD Foundation.
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

#ifndef _MACHINE_PTE_H_
#define	_MACHINE_PTE_H_

#ifndef LOCORE
typedef	uint64_t	pd_entry_t;		/* page directory entry */
typedef	uint64_t	pt_entry_t;		/* page table entry */
#endif

/* Block and Page attributes */
#define	ATTR_MASK_H		UINT64_C(0xfffc000000000000)
#define	ATTR_MASK_L		UINT64_C(0x0000000000000fff)
#define	ATTR_MASK		(ATTR_MASK_H | ATTR_MASK_L)
/* Bits 58:55 are reserved for software */
#define	ATTR_SW_UNUSED2		(1UL << 58)
#define	ATTR_SW_UNUSED1		(1UL << 57)
#define	ATTR_SW_MANAGED		(1UL << 56)
#define	ATTR_SW_WIRED		(1UL << 55)

#define	ATTR_S1_UXN		(1UL << 54)
#define	ATTR_S1_PXN		(1UL << 53)
#define	ATTR_S1_XN		(ATTR_S1_PXN | ATTR_S1_UXN)

#define	ATTR_S2_XN(x)		((x) << 53)
#define	 ATTR_S2_XN_MASK	ATTR_S2_XN(3UL)
#define	 ATTR_S2_XN_NONE	0UL	/* Allow execution at EL0 & EL1 */
#define	 ATTR_S2_XN_EL1		1UL	/* Allow execution at EL0 */
#define	 ATTR_S2_XN_ALL		2UL	/* No execution */
#define	 ATTR_S2_XN_EL0		3UL	/* Allow execution at EL1 */

#define	ATTR_CONTIGUOUS		(1UL << 52)
#define	ATTR_DBM		(1UL << 51)
#define	ATTR_S1_nG		(1 << 11)
#define	ATTR_AF			(1 << 10)
#define	ATTR_SH(x)		((x) << 8)
#define	 ATTR_SH_MASK		ATTR_SH(3)
#define	 ATTR_SH_NS		0		/* Non-shareable */
#define	 ATTR_SH_OS		2		/* Outer-shareable */
#define	 ATTR_SH_IS		3		/* Inner-shareable */

#define	ATTR_S1_AP_RW_BIT	(1 << 7)
#define	ATTR_S1_AP(x)		((x) << 6)
#define	 ATTR_S1_AP_MASK	ATTR_S1_AP(3)
#define	 ATTR_S1_AP_RW		(0 << 1)
#define	 ATTR_S1_AP_RO		(1 << 1)
#define	 ATTR_S1_AP_USER	(1 << 0)
#define	ATTR_S1_NS		(1 << 5)
#define	ATTR_S1_IDX(x)		((x) << 2)
#define	ATTR_S1_IDX_MASK	(7 << 2)

#define	ATTR_S2_S2AP(x)		((x) << 6)
#define	 ATTR_S2_S2AP_MASK	3
#define	 ATTR_S2_S2AP_READ	1
#define	 ATTR_S2_S2AP_WRITE	2

#define	ATTR_S2_MEMATTR(x)		((x) << 2)
#define	 ATTR_S2_MEMATTR_MASK		ATTR_S2_MEMATTR(0xf)
#define	 ATTR_S2_MEMATTR_DEVICE_nGnRnE	0x0
#define	 ATTR_S2_MEMATTR_NC		0xf
#define	 ATTR_S2_MEMATTR_WT		0xa
#define	 ATTR_S2_MEMATTR_WB		0xf

#define	ATTR_DEFAULT	(ATTR_AF | ATTR_SH(ATTR_SH_IS))

#define	ATTR_DESCR_MASK		3
#define	ATTR_DESCR_VALID	1
#define	ATTR_DESCR_TYPE_MASK	2
#define	ATTR_DESCR_TYPE_TABLE	2
#define	ATTR_DESCR_TYPE_PAGE	2
#define	ATTR_DESCR_TYPE_BLOCK	0

#endif /* !_MACHINE_PTE_H_ */

/* End of pte.h */
