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

#ifndef _DEV_IOMMU_SMMU_REG_H_
#define	_DEV_IOMMU_SMMU_REG_H_

#define	SMMU_IDR0		0x000
#define	SMMU_IDR1		0x004
#define	SMMU_IDR2		0x008
#define	SMMU_IDR3		0x00C
#define	SMMU_IDR4		0x010
#define	SMMU_IDR5		0x014
#define	SMMU_IIDR		0x018
#define	SMMU_AIDR		0x01C
#define	SMMU_CR0		0x020
#define	SMMU_CR0ACK		0x024
#define	SMMU_CR1		0x028
#define	SMMU_CR2		0x02C
#define	SMMU_STATUSR		0x040
#define	SMMU_GBPA		0x044
#define	SMMU_AGBPA		0x048
#define	SMMU_IRQ_CTRL		0x050
#define	SMMU_IRQ_CTRLACK	0x054
#define	SMMU_GERROR		0x060
#define	SMMU_GERRORN		0x064
#define	SMMU_GERROR_IRQ_CFG0	0x068
#define	SMMU_GERROR_IRQ_CFG1	0x070
#define	SMMU_GERROR_IRQ_CFG2	0x074
#define	SMMU_STRTAB_BASE	0x080
#define	SMMU_STRTAB_BASE_CFG	0x088
#define	SMMU_CMDQ_BASE		0x090
#define	SMMU_CMDQ_PROD		0x098
#define	SMMU_CMDQ_CONS		0x09C
#define	SMMU_EVENTQ_BASE	0x0A0
#define	SMMU_EVENTQ_PROD	0x100A8
#define	SMMU_EVENTQ_CONS	0x100AC
#define	SMMU_EVENTQ_IRQ_CFG0	0x0B0
#define	SMMU_EVENTQ_IRQ_CFG1	0x0B8
#define	SMMU_EVENTQ_IRQ_CFG2	0x0BC
#define	SMMU_PRIQ_BASE		0x0C0
#define	SMMU_PRIQ_PROD		0x100C8
#define	SMMU_PRIQ_CONS		0x100CC
#define	SMMU_PRIQ_IRQ_CFG0	0x0D0
#define	SMMU_PRIQ_IRQ_CFG1	0x0D8
#define	SMMU_PRIQ_IRQ_CFG2	0x0DC
#define	SMMU_GATOS_CTRL		0x100
#define	SMMU_GATOS_SID		0x108
#define	SMMU_GATOS_ADDR		0x110
#define	SMMU_GATOS_PAR		0x118
#define	SMMU_VATOS_SEL		0x180
#define	SMMU_S_IDR0		0x8000
#define	SMMU_S_IDR1		0x8004
#define	SMMU_S_IDR2		0x8008
#define	SMMU_S_IDR3		0x800C
#define	SMMU_S_IDR4		0x8010
#define	SMMU_S_CR0		0x8020
#define	SMMU_S_CR0ACK		0x8024
#define	SMMU_S_CR1		0x8028
#define	SMMU_S_CR2		0x802C
#define	SMMU_S_INIT		0x803C
#define	SMMU_S_GBPA		0x8044
#define	SMMU_S_AGBPA		0x8048
#define	SMMU_S_IRQ_CTRL		0x8050
#define	SMMU_S_IRQ_CTRLACK	0x8054
#define	SMMU_S_GERROR		0x8060
#define	SMMU_S_GERRORN		0x8064
#define	SMMU_S_GERROR_IRQ_CFG0	0x8068
#define	SMMU_S_GERROR_IRQ_CFG1	0x8070
#define	SMMU_S_GERROR_IRQ_CFG2	0x8074
#define	SMMU_S_STRTAB_BASE	0x8080
#define	SMMU_S_STRTAB_BASE_CFG	0x8088
#define	SMMU_S_CMDQ_BASE	0x8090
#define	SMMU_S_CMDQ_PROD	0x8098
#define	SMMU_S_CMDQ_CONS	0x809C
#define	SMMU_S_EVENTQ_BASE	0x80A0
#define	SMMU_S_EVENTQ_PROD	0x80A8
#define	SMMU_S_EVENTQ_CONS	0x80AC
#define	SMMU_S_EVENTQ_IRQ_CFG0	0x80B0
#define	SMMU_S_EVENTQ_IRQ_CFG1	0x80B8
#define	SMMU_S_EVENTQ_IRQ_CFG2	0x80BC
#define	SMMU_S_GATOS_CTRL	0x8100
#define	SMMU_S_GATOS_SID	0x8108
#define	SMMU_S_GATOS_ADDR	0x8110
#define	SMMU_S_GATOS_PAR	0x8118

#endif /* _DEV_IOMMU_SMMU_REG_H_ */
