/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Ruslan Bukin <br@bsdpad.com>
 *
 * This work was supported by Innovate UK project 105694, "Digital Security
 * by Design (DSbD) Technology Platform Prototype".
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

#ifndef	_DEV_DRM_KOMEDA_KOMEDA_REGS_H_
#define	_DEV_DRM_KOMEDA_KOMEDA_REGS_H_

/* Global Control Unit. */
#define	GLB_ARCH_ID		0x000
#define	GLB_CORE_ID		0x004
#define	GLB_CORE_INFO		0x008
#define	GLB_IRQ_STATUS		0x010
#define	GCU_IRQ_RAW_STATUS	0x0A0
#define	GCU_IRQ_CLEAR		0x0A4
#define	GCU_IRQ_MASK		0x0A8
#define	GCU_IRQ_STATUS		0x0AC
#define	GCU_STATUS		0x0B0
#define	GCU_CONTROL		0x0D0
#define	GCU_CONFIG_VALID0	0x0D4
#define	GCU_CONFIGURATION_ID0	0x100
#define	GCU_CONFIGURATION_ID1	0x104

/* DOU0 Backend Subsystem. */
#define	BS_INFO			0x1EC0
#define	BS_PROG_LINE		0x1ED4
#define	BS_PREFETCH_LINE	0x1ED8
#define	BS_BG_COLOR		0x1EDC
#define	BS_ACTIVESIZE		0x1EE0
#define	BS_HINTERVALS		0x1EE4
#define	BS_VINTERVALS		0x1EE8
#define	BS_SYNC			0x1EEC
#define	BS_DRIFT_TO		0x1F00
#define	BS_FRAME_TO		0x1F04
#define	BS_TE_TO		0x1F08
#define	BS_T0_INTERVAL		0x1F10
#define	BS_T1_INTERVAL		0x1F14
#define	BS_T2_INTERVAL		0x1F18
#define	BS_CRC0_LOW		0x1F20
#define	BS_CRC0_HIGH		0x1F24
#define	BS_CRC1_LOW		0x1F28
#define	BS_CRC1_HIGH		0x1F2C
#define	BS_USER			0x1F30

#endif /* !_DEV_DRM_KOMEDA_KOMEDA_REGS_H_ */
