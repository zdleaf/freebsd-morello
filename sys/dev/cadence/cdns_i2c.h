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

#ifndef	_DEV_CADENCE_CDNS_I2C_H_
#define	_DEV_CADENCE_CDNS_I2C_H_

#define	CDNS_I2C_CR		0x00	/* Control Register */
#define	CDNS_I2C_SR		0x04	/* (ro) Status Register */
#define	CDNS_I2C_ADDR		0x08	/* Address Register */
#define	CDNS_I2C_DATA		0x0C	/* Data Register */
#define	CDNS_I2C_ISR		0x10	/* Interrupt Status Register */
#define	CDNS_I2C_TRANS_SIZE	0x14	/* (8) Transfer Size Register */
#define	CDNS_I2C_SLV_PAUSE	0x18	/* (8) Slave Monitor Pause Register */
#define	CDNS_I2C_TIME_OUT	0x1C	/* (8) Time Out Register */
#define	CDNS_I2C_IMR		0x20	/* (ro) Interrupt Mask Register */
#define	CDNS_I2C_IER		0x24	/* Interrupt Enable Register */
#define	CDNS_I2C_IDR		0x28	/* Interrupt Disable Register */

#endif /* !_DEV_CADENCE_CDNS_I2C_H_ */
