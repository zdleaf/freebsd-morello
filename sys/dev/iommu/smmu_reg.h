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
#define	 IDR0_ST_LVL_S		27
#define	 IDR0_ST_LVL_M		(0x3 << IDR0_ST_LVL_S)
#define	 IDR0_ST_LVL_LINEAR	(0x0 << IDR0_ST_LVL_S) /* Linear Stream table*/
#define	 IDR0_ST_LVL_2		(0x1 << IDR0_ST_LVL_S) /* 2-level Stream Table*/
#define	 IDR0_ST_TERM_MODEL	(1 << 26) /* Terminate model behavior */
#define	 IDR0_STALL_MODEL_S	24 /*  Stall model support */
#define	 IDR0_STALL_MODEL_M	(0x3 << IDR0_STALL_MODEL_S)
#define	 IDR0_STALL_MODEL_STALL	(0x0 << IDR0_STALL_MODEL_S) /* Stall and Term*/
#define	 IDR0_STALL_MODEL_FORCE	(0x2 << IDR0_STALL_MODEL_S) /* Stall is forced*/
#define	 IDR0_TTENDIAN_S	21 /* Endianness for translation table walks.*/
#define	 IDR0_TTENDIAN_M	(0x3 << IDR0_TTENDIAN_S)
#define	 IDR0_TTENDIAN_MIXED	(0x0 << IDR0_TTENDIAN_S)
#define	 IDR0_TTENDIAN_LITTLE	(0x2 << IDR0_TTENDIAN_S)
#define	 IDR0_TTENDIAN_BIG	(0x3 << IDR0_TTENDIAN_S)
#define	 IDR0_VATOS		(1 << 20) / * Virtual ATOS page interface */
#define	 IDR0_CD2L		(1 << 19) /* 2-level Context descriptor table*/
#define	 IDR0_VMID16		(1 << 18) /* 16-bit VMID supported */
#define	 IDR0_VMW		(1 << 17) /* VMID wildcard-matching */
#define	 IDR0_PRI		(1 << 16) /* Page Request Interface supported*/
#define	 IDR0_ATOS		(1 << 15) /* Address Translation Operations */
#define	 IDR0_SEV		(1 << 14) /* WFE wake-up events */
#define	 IDR0_MSI		(1 << 13) /* Message Signalled Interrupts */
#define	 IDR0_ASID16		(1 << 12) /* 16-bit ASID supported */
#define	 IDR0_NS1ATS		(1 << 11) /* Split-stage ATS not supported */
#define	 IDR0_ATS		(1 << 10) /* PCIe ATS supported by SMMU */
#define	 IDR0_HYP		(1 << 9) /* Hypervisor stage 1 contexts */
#define	 IDR0_DORMHINT		(1 << 8) /* Dormant hint supported */
#define	 IDR0_HTTU_S		6 /* H/W transl. table A-flag and Dirty state */
#define	 IDR0_HTTU_M		(0x3 << IDR0_HTTU_S)
#define	 IDR0_HTTU_A		(0x1 << IDR0_HTTU_S) /* Access flag (A-flag) */
#define	 IDR0_HTTU_AD		(0x2 << IDR0_HTTU_S) /* A-flag and Dirty State*/
#define	 IDR0_BTM		(1 << 5) /* Broadcast TLB Maintenance */
#define	 IDR0_COHACC		(1 << 4) /* Coherent access to translations*/
#define	 IDR0_TTF_S		2 /* Translation Table Formats supported */
#define	 IDR0_TTF_M		(0x3 << IDR0_TTF_S)
#define	 IDR0_TTF_AA32		(0x1 << IDR0_TTF_S) /* AArch32 (LPAE) */
#define	 IDR0_TTF_AA64		(0x2 << IDR0_TTF_S) /* AArch64 */
#define	 IDR0_TTF_ALL		(0x3 << IDR0_TTF_S) /* AArch32 and AArch64 */
#define	 IDR0_S1P		(1 << 1) / * Stage1 translation supported. */
#define	 IDR0_S2P		(1 << 0) / * Stage2 translation supported. */
#define	SMMU_IDR1		0x004
#define	 IDR1_TABLES_PRESET	(1 << 30) /* Table base addresses fixed. */
#define	 IDR1_QUEUES_PRESET	(1 << 29) /* Queue base addresses fixed. */
#define	 IDR1_REL		(1 << 28) /* Relative base pointers */
#define	 IDR1_ATTR_TYPES_OVR	(1 << 27) /* Incoming attrs can be overridden*/
#define	 IDR1_ATTR_PERMS_OVR	(1 << 26) /* Incoming attrs can be overridden*/
#define	 IDR1_CMDQS_S		21 /* Maximum number of Command queue entries*/
#define	 IDR1_CMDQS_M		(0x1f << IDR1_CMDQS_S)
#define	 IDR1_EVENTQS_S		16 /* Maximum number of Event queue entries */
#define	 IDR1_EVENTQS_M		(0x1f << IDR1_EVENTQS_S)
#define	 IDR1_PRIQS_S		11 /* Maximum number of PRI queue entries */
#define	 IDR1_PRIQS_M		(0x1f << IDR1_PRIQS_S)
#define	 IDR1_SSIDSIZE_S	6 /* Max bits of SubstreamID */
#define	 IDR1_SSIDSIZE_M	(0x1f << IDR1_SSIDSIZE_S)
#define	 IDR1_SIDSIZE_S		0 /* Max bits of StreamID */
#define	 IDR1_SIDSIZE_M		(0x3f << IDR1_SIDSIZE_S)
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
