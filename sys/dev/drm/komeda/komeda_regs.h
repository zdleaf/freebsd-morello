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

#endif /* !_DEV_DRM_KOMEDA_KOMEDA_REGS_H_ */
