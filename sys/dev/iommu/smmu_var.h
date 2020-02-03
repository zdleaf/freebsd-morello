/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Ruslan Bukin <br@bsdpad.com>
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

#ifndef _DEV_IOMMU_SMMU_VAR_H_
#define _DEV_IOMMU_SMMU_VAR_H_

#define	SMMU_DEVSTR	"ARM System Memory Management Unit"

DECLARE_CLASS(smmu_driver);

struct smmu_softc {
	device_t		dev;
	struct resource		*res[4];
	void			*intr_cookie[3];
	uint32_t		ias; /* Intermediate Physical Address */
	uint32_t		oas; /* Physical Address */
	uint32_t		asid_bits;
	uint32_t		vmid_bits;
	uint32_t		sid_bits;
	uint32_t		ssid_bits;
	uint32_t		features;
#define	SMMU_FEATURE_2_LVL_STREAM_TABLE		(1 << 0)
#define	SMMU_FEATURE_2_LVL_CD			(1 << 1)
#define	SMMU_FEATURE_TT_LE			(1 << 2)
#define	SMMU_FEATURE_TT_BE			(1 << 3)
#define	SMMU_FEATURE_SEV			(1 << 4)
#define	SMMU_FEATURE_MSI			(1 << 5)
#define	SMMU_FEATURE_HYP			(1 << 6)
#define	SMMU_FEATURE_ATS			(1 << 7)
#define	SMMU_FEATURE_PRI			(1 << 8)
#define	SMMU_FEATURE_STALL_FORCE		(1 << 9)
#define	SMMU_FEATURE_STALL			(1 << 10)
#define	SMMU_FEATURE_S1P			(1 << 11)
#define	SMMU_FEATURE_S2P			(1 << 12)
};

struct smmu_devinfo {
	int smmu_domain;
};

MALLOC_DECLARE(M_SMMU);

/* Device methods */
int smmu_attach(device_t dev);
int smmu_detach(device_t dev);

#endif /* _DEV_IOMMU_SMMU_VAR_H_ */
