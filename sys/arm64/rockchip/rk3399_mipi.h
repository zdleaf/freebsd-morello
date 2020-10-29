/*-
 * Copyright (c) 2018-2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
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
 */

#ifndef _ARM_STM_STM32F4_DSI_H_
#define _ARM_STM_STM32F4_DSI_H_

#define	DSI_VR		0x0000 /* DSI Host Version Register */
#define	DSI_CR		0x0004 /* DSI Host Control Register */
#define	 CR_EN		(1 << 0) /* DSI Host is enabled */
#define	DSI_CCR		0x0008 /* DSI Host Clock Control Register */
#define	 CCR_TOCKDIV_S	8 /* Timeout Clock Division */
#define	 CCR_TOCKDIV_M	(0xff << CCR_TOCKDIV_S)
#define	 CCR_TXECKDIV_S	0 /* TX Escape Clock Division */
#define	 CCR_TXECKDIV_M	(0xff << CCR_TXECKDIV_S)
#define	DSI_LVCIDR	0x000C /* DSI Host LTDC VCID Register */
#define	 LVCIDR_VCID_S	0 /* Virtual Channel ID */
#define	 LVCIDR_VCID_M	0x3
#define	DSI_LCOLCR	0x0010 /* DSI Host LTDC Color Coding Register */
#define	 LCOLCR_LPE	(1 << 8) /* Loosely Packet Enable */
#define	 LCOLCR_COLC_S	0 /* Color Coding */
#define	 LCOLCR_COLC_M	0xf
#define	 LCOLCR_COLC_16_1	0x0 /* 16-bit */
#define	 LCOLCR_COLC_16_2	0x1 /* 16-bit */
#define	 LCOLCR_COLC_16_3	0x2 /* 16-bit */
#define	 LCOLCR_COLC_18_1	0x3 /* 18-bit */
#define	 LCOLCR_COLC_18_2	0x4 /* 18-bit */
#define	 LCOLCR_COLC_24		0x5 /* 24-bit */
#define	DSI_LPCR	0x0014 /* DSI Host LTDC Polarity Configuration Register */
#define	 LPCR_HSP	(1 << 2) /* HSYNC Polarity */
#define	 LPCR_VSP	(1 << 1) /* VSYNC Polarity */
#define	 LPCR_DEP	(1 << 0) /* Data Enable Polarity */
#define	DSI_LPMCR	0x0018 /* DSI Host Low-Power mode Configuration Register */
#define	 LPMCR_LPSIZE_S		16 /* Largest Packet Size */
#define	 LPMCR_LPSIZE_M		0xff
#define	 LPMCR_VLPSIZE_S	0 /* VACT Largest Packet Size */
#define	 LPMCR_VLPSIZE_M	0xff
#define	DSI_PCR		0x002C /* DSI Host Protocol Configuration Register */
#define	DSI_GVCIDR	0x0030 /* DSI Host Generic VCID Register */
#define	DSI_MCR		0x0034 /* DSI Host mode Configuration Register */
#define	 MCR_CMDM	(1 << 0) /* Command mode. Reset value */
#define	DSI_VMCR	0x0038 /* DSI Host Video mode Configuration Register */
#define	 VMCR_LPCE	(1 << 15) /* Low-Power Command Enable */
#define	 VMCR_FBTAAE	(1 << 14) /* Frame Bus-Turn-Around Acknowledge Enable */
#define	 VMCR_LPHFPE	(1 << 13) /* Low-Power Horizontal Front-Porch Enable */
#define	 VMCR_LPHBPE	(1 << 12) /* Low-Power Horizontal Back-Porch Enable */
#define	 VMCR_LPVAE	(1 << 11) /* Low-Power Vertical Active Enable */
#define	 VMCR_LPVFPE	(1 << 10) /* Low-power Vertical Front-porch Enable */
#define	 VMCR_LPVBPE	(1 << 9) /* Low-power Vertical Back-Porch Enable */
#define	 VMCR_LPVSAE	(1 << 8) /* Low-Power Vertical Sync Active Enable */
#define	 VMCR_VMT_S	0 /* Video mode Type */
#define	 VMCR_VMT_M	(0x3 << VMCR_VMT_S)
#define	 VMCR_VMT_PULS	(0x0 << VMCR_VMT_S)
#define	 VMCR_VMT_EVNT	(0x1 << VMCR_VMT_S)
#define	 VMCR_VMT_BRST	(0x2 << VMCR_VMT_S)
#define	DSI_VPCR	0x003C /* DSI Host Video Packet Configuration Register */
#define	 VPCR_VPSIZE_S	0 /* Video Packet Size */
#define	 VPCR_VPSISE_M	(0x3fff << VPCR_VPSIZE_S)
#define	DSI_VCCR	0x0040 /* DSI Host Video Chunks Configuration Register */
#define	 VCCR_NUMC_S	0 /* Number of Chunks */
#define	 VCCR_NUMC_M	0x1fff
#define	DSI_VNPCR	0x0044 /* DSI Host Video Null Packet Configuration Register */
#define	 VNPCR_NPSIZE_S	0 /* Null Packet Size */
#define	 VNPCR_NPSIZE_M	0x1fff
#define	DSI_VHSACR	0x0048 /* DSI Host Video HSA Configuration Register */
#define	DSI_VHBPCR	0x004C /* DSI Host Video HBP Configuration Register */
#define	DSI_VLCR	0x0050 /* DSI Host Video Line Configuration Register */
#define	DSI_VVSACR	0x0054 /* DSI Host Video VSA Configuration Register */
#define	DSI_VVBPCR	0x0058 /* DSI Host Video VBP Configuration Register */
#define	DSI_VVFPCR	0x005C /* DSI Host Video VFP Configuration Register */
#define	DSI_VVACR	0x0060 /* DSI Host Video VA Configuration Register */
#define	DSI_LCCR	0x0064 /* DSI Host LTDC Command Configuration Register */
#define	DSI_CMCR	0x0068 /* DSI Host Command mode Configuration Register */
#define	DSI_GHCR	0x006C /* DSI Host Generic Header Configuration Register */
#define	 GHCR_WCMSB_S	16 /* WordCount MSB */
#define	 GHCR_WCMSB_M	0xff
#define	 GHCR_WCLSB_S	8 /* WordCount LSB */
#define	 GHCR_WCLSB_M	0xff
#define	 GHCR_VCID_S	6 /* Channel */
#define	 GHCR_VCID_M	0x3
#define	 GHCR_DT_S	0 /* Type */
#define	 GHCR_DT_M	0x3f
#define	DSI_GPDR	0x0070 /* DSI Host Generic Payload Data Register */
#define	DSI_GPSR	0x0074 /* DSI Host Generic Packet Status Register */
#define	 GPSR_PRDFE	(1 << 4) /* Payload Read FIFO Empty */
#define	 GPSR_CMDFF	(1 << 1) /* Command FIFO Full */
#define	 GPSR_CMDFE	(1 << 0) /* Command FIFO Empty */
#define	DSI_TCCR0	0x0078 /* DSI Host Timeout Counter Configuration Register 0 */
#define	DSI_TCCR1	0x007C /* DSI Host Timeout Counter Configuration Register 1 */
#define	DSI_TCCR2	0x0080 /* DSI Host Timeout Counter Configuration Register 2 */
#define	DSI_TCCR3	0x0084 /* DSI Host Timeout Counter Configuration Register 3 */
#define	DSI_TCCR4	0x0088 /* DSI Host Timeout Counter Configuration Register 4 */
#define	DSI_TCCR5	0x008C /* DSI Host Timeout Counter Configuration Register 5 */
#define	DSI_CLCR	0x0094 /* DSI Host Clock Lane Configuration Register */
#define	 CLCR_ACR	(1 << 1) /* Automatic Clock lane Control */
#define	 CLCR_DPCC	(1 << 0) /* D-PHY Clock Control */
#define	DSI_CLTCR	0x0098 /* DSI Host Clock Lane Timer Configuration Register */
#define	DSI_DLTCR	0x009C /* DSI Host Data Lane Timer Configuration Register */
#define	DSI_PCTLR	0x00A0 /* DSI Host PHY Control Register */
#define	 PCTLR_CKE	(1 << 2) /* Clock Enable */
#define	 PCTLR_DEN	(1 << 1) /* Digital Enable */
#define	 PCTLR_SHUTDOWN	(1 << 0) /* Shutdown */
#define	DSI_PCONFR	0x00A4 /* DSI Host PHY Configuration Register */
#define	 SW_TIME_S	8 /* Stop Wait Time */
#define	 SW_TIME_M	(0xff << SW_TIME_S)
#define	 PCONFR_NL_S	0 /* Number of Lanes */
#define	 PCONFR_NL_M	0x3
#define	 PCONFR_NL_1	(0 << PCONFR_NL_S)
#define	 PCONFR_NL_2	(1 << PCONFR_NL_S)	/* Reset value */
#define	DSI_PUCR	0x00A8 /* DSI Host PHY ULPS Control Register */
#define	DSI_PTTCR	0x00AC /* DSI Host PHY TX Triggers Configuration Register */
#define	DSI_PSR		0x00B0 /* DSI Host PHY Status Register */
#define	DSI_PHY_TST_CTRL0	0x00B4
#define	 TST_CTRL0_TESTCLK	(1 << 1)
#define	 TST_CTRL0_TESTCLR	(1 << 0)
#define	DSI_PHY_TST_CTRL1	0x00B8
#define	 TST_CTRL1_TESTEN	(1 << 16)
#define	 TST_CTRL1_TESTDIN_S	0
#define	 TST_CTRL1_TESTDIN_M	(0xff << TST_CTRL1_TESTDIN_S)
#define	 TST_CTRL1_TESTDOUT_S	8
#define	 TST_CTRL1_TESTDOUT_M	(0xff << TST_CTRL1_TESTDOUT_S)
#define	DSI_ISR0	0x00BC /* DSI Host Interrupt & Status Register 0 */
#define	DSI_ISR1	0x00C0 /* DSI Host Interrupt & Status Register 1 */
#define	DSI_IER0	0x00C4 /* DSI Host Interrupt Enable Register 0 */
#define	DSI_IER1	0x00C8 /* DSI Host Interrupt Enable Register 1 */
#define	DSI_FIR0	0x00D8 /* DSI Host Force Interrupt Register 0 */
#define	DSI_FIR1	0x00DC /* DSI Host Force Interrupt Register 1 */
#define	DSI_VSCR	0x0100 /* DSI Host Video Shadow Control Register */
#define	DSI_LCVCIDR	0x010C /* DSI Host LTDC Current VCID Register */
#define	DSI_LCCCR	0x0110 /* DSI Host LTDC Current Color Coding Register */
#define	DSI_LPMCCR	0x0118 /* DSI Host Low-Power mode Current Configuration */
#define	DSI_VMCCR	0x0138 /* DSI Host Video mode Current Configuration Register */
#define	DSI_VPCCR	0x013C /* DSI Host Video Packet Current Configuration Register */
#define	DSI_VCCCR	0x0140 /* DSI Host Video Chunks Current Configuration Register */
#define	DSI_VNPCCR	0x0144 /* DSI Host Video Null Packet Current Configuration Register */
#define	DSI_VHSACCR	0x0148 /* DSI Host Video HSA Current Configuration Register */
#define	DSI_VHBPCCR	0x014C /* DSI Host Video HBP Current Configuration Register */
#define	DSI_VLCCR	0x0150 /* DSI Host Video Line Current Configuration Register (DSI_VLCCR) */
#define	DSI_VVSACCR	0x0154 /* DSI Host Video VSA Current Configuration Register */
#define	DSI_VVBPCCR	0x0158 /* DSI Host Video VBP Current Configuration Register */
#define	DSI_VVFPCCR	0x015C /* DSI Host Video VFP Current Configuration Register */
#define	DSI_VVACCR	0x0160 /* DSI Host Video VA Current Configuration Register */

#define	CODE_PLL_VCORANGE_VCOCAP	0x10

struct stm32f4_dsi_config {
	uint8_t ndiv;
	uint8_t odf;
	uint8_t idf;
	uint8_t nlanes;
	uint8_t video_mode;
	uint32_t hse_val;
};

#endif /* !_ARM_STM_STM32F4_DSI_H_ */
