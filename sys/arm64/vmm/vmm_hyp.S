/*
 * Copyright (C) 2017 Alexandru Elisei <alexandru.elisei@gmail.com>
 * All rights reserved.
 *
 * This software was developed by Alexandru Elisei under sponsorship
 * from the FreeBSD Foundation.
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <sys/syscall.h>
#include <machine/armreg.h>
#include <machine/asm.h>
#include <machine/hypervisor.h>

#include "assym.inc"
#include "hyp_macros.h"
#include "hyp.h"
#include "hyp_assym.h"

	.text

	.globl	hyp_code_start
	.globl	hyp_code_end

	.align 12
hyp_code_start:

.macro	save_host_registers
	sub	sp, sp, #(32 * 8)
	str	x30,      [sp, #(30 * 8)]
	stp	x28, x29, [sp, #(28 * 8)]
	stp	x26, x27, [sp, #(26 * 8)]
	stp	x24, x25, [sp, #(24 * 8)]
	stp	x22, x23, [sp, #(22 * 8)]
	stp	x20, x21, [sp, #(20 * 8)]
	stp	x18, x19, [sp, #(18 * 8)]
	stp	x16, x17, [sp, #(16 * 8)]
	stp	x14, x15, [sp, #(14 * 8)]
	stp	x12, x13, [sp, #(12 * 8)]
	stp	x10, x11, [sp, #(10 * 8)]
	stp	x8,  x9,  [sp, #(8  * 8)]
	stp	x6,  x7,  [sp, #(6  * 8)]
	stp	x4,  x5,  [sp, #(4  * 8)]
	stp	x2,  x3,  [sp, #(2  * 8)]
	stp	x0,  x1,  [sp, #(0  * 8)]

	/* Store the special registers */
	mrs	x20, sp_el0
	mrs	x21, sp_el1
	mrs	x22, elr_el1
	mrs	x23, elr_el2
	mrs	x24, tpidr_el0
	mrs	x25, tpidrro_el0
	mrs	x26, tpidr_el1
	mrs	x27, vbar_el1

	sub	sp, sp, #(8 * 8)
	stp	x26, x27, [sp, #(6 * 8)]
	stp	x24, x25, [sp, #(4 * 8)]
	stp	x22, x23, [sp, #(2 * 8)]
	stp	x20, x21, [sp, #(0 * 8)]

	mrs	x20, actlr_el1
	mrs	x21, amair_el1
	mrs	x22, par_el1
	mrs	x23, mair_el1
	mrs	x24, tcr_el1
	mrs	x25, ttbr0_el1
	mrs	x26, ttbr1_el1
	mrs	x27, afsr0_el1

	sub	sp, sp, #(8 * 8)
	stp	x26, x27, [sp, #(6 * 8)]
	stp	x24, x25, [sp, #(4 * 8)]
	stp	x22, x23, [sp, #(2 * 8)]
	stp	x20, x21, [sp, #(0 * 8)]

	mrs	x20, afsr1_el1
	mrs	x21, contextidr_el1
	mrs	x22, cpacr_el1
	mrs	x23, esr_el1
	mrs	x24, far_el1
	mrs	x25, sctlr_el1
	mrs	x26, spsr_el1
	mrs	x27, hcr_el2

	sub	sp, sp, #(8 * 8)
	stp	x26, x27, [sp, #(6 * 8)]
	stp	x24, x25, [sp, #(4 * 8)]
	stp	x22, x23, [sp, #(2 * 8)]
	stp	x20, x21, [sp, #(0 * 8)]

	mrs	x20, vpidr_el2
	mrs	x21, vmpidr_el2
	mrs	x22, cptr_el2
	mrs	x23, spsr_el2
	mrs	x24, ich_hcr_el2
	mrs	x25, ich_vmcr_el2
	mrs	x26, cnthctl_el2
	mrs	x27, cntkctl_el1

	sub	sp, sp, #(8 * 8)
	stp	x26, x27, [sp, #(6 * 8)]
	stp	x24, x25, [sp, #(4 * 8)]
	stp	x22, x23, [sp, #(2 * 8)]
	stp	x20, x21, [sp, #(0 * 8)]

	mrs	x20, cntvoff_el2

	sub	sp, sp, #(2 * 8)
	str	x20, [sp]

#ifdef VFP
	/* Store the host VFP special registers */
	mrs	x27, fpcr
	mrs	x28, fpsr
	sub	sp, sp, #32
	stp	x27, x28, [sp]

	/* Store the host VFP registers */
	sub	sp, sp, #(32 * 16)
	stp	q30, q31, [sp, #(30 * 16)]
	stp	q28, q29, [sp, #(28 * 16)]
	stp	q26, q27, [sp, #(26 * 16)]
	stp	q24, q25, [sp, #(24 * 16)]
	stp	q22, q23, [sp, #(22 * 16)]
	stp	q20, q21, [sp, #(20 * 16)]
	stp	q18, q19, [sp, #(18 * 16)]
	stp	q16, q17, [sp, #(16 * 16)]
	stp	q14, q15, [sp, #(14 * 16)]
	stp	q12, q13, [sp, #(12 * 16)]
	stp	q10, q11, [sp, #(10 * 16)]
	stp	q8,  q9,  [sp, #(8  * 16)]
	stp	q6,  q7,  [sp, #(6  * 16)]
	stp	q4,  q5,  [sp, #(4  * 16)]
	stp	q2,  q3,  [sp, #(2  * 16)]
	stp	q0,  q1,  [sp, #(0  * 16)]
#endif

.endm

.macro	restore_host_registers
#ifdef VFP
	/* Restore the host VFP registers */
	ldp	q0,  q1,  [sp, #(0  * 16)]
	ldp	q2,  q3,  [sp, #(2  * 16)]
	ldp	q4,  q5,  [sp, #(4  * 16)]
	ldp	q6,  q7,  [sp, #(6  * 16)]
	ldp	q8,  q9,  [sp, #(8  * 16)]
	ldp	q10, q11, [sp, #(10 * 16)]
	ldp	q12, q13, [sp, #(12 * 16)]
	ldp	q14, q15, [sp, #(14 * 16)]
	ldp	q16, q17, [sp, #(16 * 16)]
	ldp	q18, q19, [sp, #(18 * 16)]
	ldp	q20, q21, [sp, #(20 * 16)]
	ldp	q22, q23, [sp, #(22 * 16)]
	ldp	q24, q25, [sp, #(24 * 16)]
	ldp	q26, q27, [sp, #(26 * 16)]
	ldp	q28, q29, [sp, #(28 * 16)]
	ldp	q30, q31, [sp, #(30 * 16)]
	add	sp, sp, #(32 * 16)

	/* Restore the host VFP special registers */
	ldp	x0, x1, [sp]
	add	sp, sp, #32
	msr	fpcr, x0
	msr	fpsr, x1
#endif

	ldr	x0, [sp]
	add	sp, sp, #(2 * 8)

	msr	cntvoff_el2, x0

	ldp	x6, x7, [sp, #(6 * 8)]
	ldp	x4, x5, [sp, #(4 * 8)]
	ldp	x2, x3, [sp, #(2 * 8)]
	ldp	x0, x1, [sp, #(0 * 8)]
	add	sp, sp, #(8 * 8)

	msr	vpidr_el2, x0
	msr	vmpidr_el2, x1
	msr	cptr_el2, x2
	msr	spsr_el2, x3
	msr	ich_hcr_el2, x4
	msr	ich_vmcr_el2, x5
	msr	cnthctl_el2, x6
	msr	cntkctl_el1, x7

	ldp	x6, x7, [sp, #(6 * 8)]
	ldp	x4, x5, [sp, #(4 * 8)]
	ldp	x2, x3, [sp, #(2 * 8)]
	ldp	x0, x1, [sp, #(0 * 8)]
	add	sp, sp, #(8 * 8)

	msr	afsr1_el1, x0
	msr	contextidr_el1, x1
	msr	cpacr_el1, x2
	msr	esr_el1, x3
	msr	far_el1, x4
	msr	sctlr_el1, x5
	msr	spsr_el1, x6
	msr	hcr_el2, x7

	ldp	x6, x7, [sp, #(6 * 8)]
	ldp	x4, x5, [sp, #(4 * 8)]
	ldp	x2, x3, [sp, #(2 * 8)]
	ldp	x0, x1, [sp, #(0 * 8)]
	add	sp, sp, #(8 * 8)

	msr	actlr_el1, x0
	msr	amair_el1, x1
	msr	par_el1, x2
	msr	mair_el1, x3
	msr	tcr_el1, x4
	msr	ttbr0_el1, x5
	msr	ttbr1_el1, x6
	msr	afsr0_el1, x7

	ldp	x6, x7, [sp, #(6 * 8)]
	ldp	x4, x5, [sp, #(4 * 8)]
	ldp	x2, x3, [sp, #(2 * 8)]
	ldp	x0, x1, [sp, #(0 * 8)]
	add	sp, sp, #(8 * 8)

	msr	sp_el0, x0
	msr	sp_el1, x1
	msr	elr_el1, x2
	msr	elr_el2, x3
	msr	tpidr_el0, x4
	msr	tpidrro_el0, x5
	msr	tpidr_el1, x6
	msr	vbar_el1, x7

	ldp	x0,  x1,  [sp, #(0  * 8)]
	ldp	x2,  x3,  [sp, #(2  * 8)]
	ldp	x4,  x5,  [sp, #(4  * 8)]
	ldp	x6,  x7,  [sp, #(6  * 8)]
	ldp	x8,  x9,  [sp, #(8  * 8)]
	ldp	x10, x11, [sp, #(10 * 8)]
	ldp	x12, x13, [sp, #(12 * 8)]
	ldp	x14, x15, [sp, #(14 * 8)]
	ldp	x16, x17, [sp, #(16 * 8)]
	ldp	x18, x19, [sp, #(18 * 8)]
	ldp	x20, x21, [sp, #(20 * 8)]
	ldp	x22, x23, [sp, #(22 * 8)]
	ldp	x24, x25, [sp, #(24 * 8)]
	ldp	x26, x27, [sp, #(26 * 8)]
	ldp	x28, x29, [sp, #(28 * 8)]
	ldr	x30,      [sp, #(30 * 8)]
	add	sp, sp, #(32 * 8)
.endm

.macro	save_guest_registers
	/* Back up x0 so we can use it as a temporary register */
	stp	x0,  x1,  [sp, #-(2 * 8)]!

	/* Restore the hypctx pointer */
	mrs	x0, tpidr_el2

	stp	x2,  x3,  [x0, #(TF_X + 2  * 8)]
	stp	x4,  x5,  [x0, #(TF_X + 4  * 8)]
	stp	x6,  x7,  [x0, #(TF_X + 6  * 8)]
	stp	x8,  x9,  [x0, #(TF_X + 8  * 8)]
	stp	x10, x11, [x0, #(TF_X + 10 * 8)]
	stp	x12, x13, [x0, #(TF_X + 12 * 8)]
	stp	x14, x15, [x0, #(TF_X + 14 * 8)]
	stp	x16, x17, [x0, #(TF_X + 16 * 8)]
	stp	x18, x19, [x0, #(TF_X + 18 * 8)]
	stp	x20, x21, [x0, #(TF_X + 20 * 8)]
	stp	x22, x23, [x0, #(TF_X + 22 * 8)]
	stp	x24, x25, [x0, #(TF_X + 24 * 8)]
	stp	x26, x27, [x0, #(TF_X + 26 * 8)]
	stp	x28, x29, [x0, #(TF_X + 28 * 8)]

	/* Load the guest stack pointer to save */
	mrs	x18, sp_el1
	stp	x18, x30, [x0, #(TF_SP)]

	/* Store the address to return to */
	mrs	x10, elr_el2
	mrs	x11, spsr_el2
	mrs	x12, esr_el2

	str	x10,      [x0, #(TF_ELR)]
	stp	w11, w12, [x0, #(TF_SPSR)]

	/* Restore the saved x0 & x1 and save them */
	ldp	x2,  x3,  [sp], #(2 * 8)
	stp	x2,  x3,  [x0, #(TF_X + 0  * 8)]

	mrs	x20, elr_el1
	mrs	x21, sp_el0
	mrs	x22, tpidr_el0
	mrs	x23, tpidrro_el0
	mrs	x24, tpidr_el1
	mrs	x25, vbar_el1

	stp	x20, x21, [x0, #(HYPCTX_ELR_EL1)]
	stp	x22, x23, [x0, #(HYPCTX_TPIDR_EL0)]
	stp	x24, x25, [x0, #(HYPCTX_TPIDR_EL1)]

	mrs	x20, actlr_el1
	mrs	x21, afsr0_el1
	mrs	x22, afsr1_el1
	mrs	x23, amair_el1
	mrs	x24, contextidr_el1
	mrs	x25, cpacr_el1
	mrs	x26, esr_el1
	mrs	x27, far_el1

	stp	x20, x21, [x0, #(HYPCTX_ACTLR_EL1)]
	stp	x22, x23, [x0, #(HYPCTX_AFSR1_EL1)]
	stp	x24, x25, [x0, #(HYPCTX_CONTEXTIDR_EL1)]
	stp	x26, x27, [x0, #(HYPCTX_ESR_EL1)]

	mrs	x20, mair_el1
	mrs	x21, par_el1
	mrs	x22, sctlr_el1
	mrs	x23, tcr_el1
	mrs	x24, ttbr0_el1
	mrs	x25, ttbr1_el1
	mrs	x26, spsr_el1
	mrs	x27, cptr_el2

	stp	x20, x21, [x0, #(HYPCTX_MAIR_EL1)]
	stp	x22, x23, [x0, #(HYPCTX_SCTLR_EL1)]
	stp	x24, x25, [x0, #(HYPCTX_TTBR0_EL1)]
	stp	x26, x27, [x0, #(HYPCTX_SPSR_EL1)]

	mrs	x20, hcr_el2
	mrs	x21, vpidr_el2
	mrs	x22, vmpidr_el2

	stp	x20, x21, [x0, #(HYPCTX_HCR_EL2)]
	str	x22,      [x0, #(HYPCTX_VMPIDR_EL2)]
.endm

.macro	restore_guest_registers
	/* x0 contains the hypctx pointer */

	ldp	x20, x21, [x0, #(HYPCTX_ELR_EL1)]
	ldp	x22, x23, [x0, #(HYPCTX_TPIDR_EL0)]
	ldp	x24, x25, [x0, #(HYPCTX_TPIDR_EL1)]

	msr	elr_el1, x20
	msr	sp_el0, x21
	msr	tpidr_el0, x22
	msr	tpidrro_el0, x23
	msr	tpidr_el1, x24
	msr	vbar_el1, x25

	ldp	x20, x21, [x0, #(HYPCTX_ACTLR_EL1)]
	ldp	x22, x23, [x0, #(HYPCTX_AFSR1_EL1)]
	ldp	x24, x25, [x0, #(HYPCTX_CONTEXTIDR_EL1)]
	ldp	x26, x27, [x0, #(HYPCTX_ESR_EL1)]

	msr	actlr_el1, x20
	msr	afsr0_el1, x21
	msr	afsr1_el1, x22
	msr	amair_el1, x23
	msr	contextidr_el1, x24
	msr	cpacr_el1, x25
	msr	esr_el1, x26
	msr	far_el1, x27

	ldp	x20, x21, [x0, #(HYPCTX_MAIR_EL1)]
	ldp	x22, x23, [x0, #(HYPCTX_SCTLR_EL1)]
	ldp	x24, x25, [x0, #(HYPCTX_TTBR0_EL1)]
	ldp	x26, x27, [x0, #(HYPCTX_SPSR_EL1)]

	msr	mair_el1, x20
	msr	par_el1, x21
	msr	sctlr_el1, x22
	msr	tcr_el1, x23
	msr	ttbr0_el1, x24
	msr	ttbr1_el1, x25
	msr	spsr_el1, x26
	msr	cptr_el2, x27

	ldp	x20, x21, [x0, #(HYPCTX_HCR_EL2)]
	ldr	x22,      [x0, #(HYPCTX_VMPIDR_EL2)]

	msr	hcr_el2, x20
	msr	vpidr_el2, x21
	msr	vmpidr_el2, x22

	/*
	 * Copy the guest x0 and x1 to the stack so we can restore them
	 * after loading the other registers.
	 */
	ldp	x2,  x3,  [x0, #(TF_X + 0  * 8)]
	stp	x2,  x3,  [sp, #-(2 * 8)]!

	/* Load the address to return to */
	ldr	w11, [x0, #(TF_SPSR)]
	ldr	x10, [x0, #(TF_ELR)]

	msr	spsr_el2, x11
	msr	elr_el2, x10

	/* Load the guest link register anmd stack pointer */
	ldp	x18, x30, [x0, #(TF_SP)]
	msr	sp_el1, x18

	ldp	x28, x29, [x0, #(TF_X + 28 * 8)]
	ldp	x26, x27, [x0, #(TF_X + 26 * 8)]
	ldp	x24, x25, [x0, #(TF_X + 24 * 8)]
	ldp	x22, x23, [x0, #(TF_X + 22 * 8)]
	ldp	x20, x21, [x0, #(TF_X + 20 * 8)]
	ldp	x18, x19, [x0, #(TF_X + 18 * 8)]
	ldp	x16, x17, [x0, #(TF_X + 16 * 8)]
	ldp	x14, x15, [x0, #(TF_X + 14 * 8)]
	ldp	x12, x13, [x0, #(TF_X + 12 * 8)]
	ldp	x10, x11, [x0, #(TF_X + 10 * 8)]
	ldp	x8,  x9,  [x0, #(TF_X + 8  * 8)]
	ldp	x6,  x7,  [x0, #(TF_X + 6  * 8)]
	ldp	x4,  x5,  [x0, #(TF_X + 4  * 8)]
	ldp	x2,  x3,  [x0, #(TF_X + 2  * 8)]

	ldp	x0,  x1,  [sp], #(2 * 8)
.endm

ENTRY(vmm_call_hyp)
	hvc	#0
	ret
END(vmm_call_hyp)


.macro vempty
	.align 7
	1: b	1b
.endm

.macro vector name
	.align 7
	b	handle_\name
.endm

	.align 11
	.globl	hyp_init_vectors
hyp_init_vectors:
	vempty		/* Synchronous EL2t */
	vempty		/* IRQ EL2t */
	vempty		/* FIQ EL2t */
	vempty		/* Error EL2t */

	vempty		/* Synchronous EL2h */
	vempty		/* IRQ EL2h */
	vempty		/* FIQ EL2h */
	vempty		/* Error EL2h */

	vector hyp_init	/* Synchronous 64-bit EL1 */
	vempty		/* IRQ 64-bit EL1 */
	vempty		/* FIQ 64-bit EL1 */
	vempty		/* Error 64-bit EL1 */

	vempty		/* Synchronous 32-bit EL1 */
	vempty		/* IRQ 32-bit EL1 */
	vempty		/* FIQ 32-bit EL1 */
	vempty		/* Error 32-bit EL1 */


/*
 * Initialize the hypervisor mode with a new exception vector table, translation
 * table and stack.
 *
 * Expecting:
 * x0 - the hypervisor exception vectors
 * x1 - translation tables physical address
 * x2 - stack top virtual address
 * x3 - TCR_EL2 value
 * x4 - SCTLR_EL2 value
 * x5 - VTCR_EL2 value
 */
ENTRY(handle_hyp_init)
	/* Install the new exception vectors */
	msr	vbar_el2, x0
	/* Set the stack top address */
	mov	sp, x2
	/* Use the host VTTBR_EL2 to tell the host and the guests apart */
	mov	x9, #VTTBR_HOST
	msr	vttbr_el2, x9
	/* Load the base address for the translation tables */
	msr	ttbr0_el2, x1
	/* Invalidate the TLB */
	tlbi	alle2
	/* Use the same memory attributes as EL1 */
	mrs	x9, mair_el1
	msr	mair_el2, x9
	/* Configure address translation */
	msr	tcr_el2, x3
	isb
	/* Set the system control register for EL2 */
	msr	sctlr_el2, x4
	/* Set the Stage 2 translation control register */
	msr	vtcr_el2, x5
	/* Return success */
	mov	x0, #0
	/* MMU is up and running */
	eret
END(handle_hyp_init)


	.align 11
	.globl	hyp_vectors
hyp_vectors:
	vempty			/* Synchronous EL2t */
	vempty			/* IRQ EL2t */
	vempty			/* FIQ EL2t */
	vempty			/* Error EL2t */

	vector el2_el2h_sync	/* Synchronous EL2h */
	vector el2_el2h_irq	/* IRQ EL2h */
	vector el2_el2h_fiq	/* FIQ EL2h */
	vector el2_el2h_error	/* Error EL2h */

	vector el2_el1_sync64	/* Synchronous 64-bit EL1 */
	vector el2_el1_irq64	/* IRQ 64-bit EL1 */
	vector el2_el1_fiq64	/* FIQ 64-bit EL1 */
	vector el2_el1_error64	/* Error 64-bit EL1 */

	vempty			/* Synchronous 32-bit EL1 */
	vempty			/* IRQ 32-bit EL1 */
	vempty			/* FIQ 32-bit EL1 */
	vempty			/* Error 32-bit EL1 */


.macro do_world_switch_to_host
	save_guest_registers
	SAVE_GUEST_REGS()
	SAVE_EXIT_INFO()
#ifdef VFP
	/*
	 * Saving the guest VFP registers needs to come after saving the rest of
	 * the registers because the process dirties the regular registers.
	 */
	SAVE_GUEST_VFP_REGS()
#endif
	restore_host_registers

	/* Restore host VTTBR */
	mov	x9, #VTTBR_HOST
	msr	vttbr_el2, x9
.endm


.macro handle_el2_excp type
	/* Save registers before modifying so we can restore them */
	str	x9, [sp, #-16]!

	/* Test if the exception happened when the host was running */
	mrs	x9, vttbr_el2
	cmp	x9, #VTTBR_HOST
	beq	1f

	/* We got the exception while the guest was running */
	ldr	x9, [sp], #16
	do_world_switch_to_host
	b	2f
1:
	/* We got the exception while the host was running */
	ldr	x9, [sp], #16
2:
	mov	x0, \type
	eret
.endm


ENTRY(handle_el2_el2h_sync)
	handle_el2_excp #EXCP_TYPE_EL2_SYNC
END(handle_el2_el2h_sync)

ENTRY(handle_el2_el2h_irq)
	handle_el2_excp #EXCP_TYPE_EL2_IRQ
END(handle_el2_el2h_sync)

ENTRY(handle_el2_el2h_fiq)
	handle_el2_excp #EXCP_TYPE_EL2_FIQ
END(handle_el2_el2h_sync)

ENTRY(handle_el2_el2h_error)
	handle_el2_excp #EXCP_TYPE_EL2_ERROR
END(handle_el2_el2h_sync)


ENTRY(handle_el2_el1_sync64)
	/* Save registers before modifying so we can restore them */
	str	x9, [sp, #-16]!

	/* Check for host hypervisor call */
	mrs	x9, vttbr_el2
	cmp	x9, #VTTBR_HOST
	beq	1f

	/* Restore register */
	ldr	x9, [sp], #16

	/* Guest exception taken to EL2 */
	do_world_switch_to_host
	mov	x0, #EXCP_TYPE_EL1_SYNC
	b	exit

1:
	/* Restore register */
	ldr	x9, [sp], #16

	cmp	x0, #HYP_GET_VECTOR_TABLE
	beq	2f
	b	call_function
2:
	/* Return the vector table base address */
	mrs	x0, vbar_el2
exit:
	eret
END(handle_el2_el1_sync64)


/*
 * Call a function in EL2 context
 *
 * Expecting:
 * x0 - function virtual address
 * x1-x7 - function parameters
 */
ENTRY(call_function)
	/* Save the function address before shuffling parameters */
	mov	x9, x0

	/* Shuffle function parameters */
	mov	x0, x1
	mov	x1, x2
	mov	x2, x3
	mov	x3, x4
	mov	x4, x5
	mov	x5, x6
	mov	x6, x7

	/* Call function */
	br	x9
END(call_function)


/*
 * We only trap IRQ, FIQ and SError exceptions when a guest is running. Do a
 * world switch to host to handle these exceptions.
 */


ENTRY(handle_el2_el1_irq64)
	do_world_switch_to_host
	str	x9, [sp, #-16]!
	mrs	x9, ich_misr_el2
	cmp	x9, xzr
	beq	1f
	mov	x0, #EXCP_TYPE_MAINT_IRQ
	b	2f
1:
	mov	x0, #EXCP_TYPE_EL1_IRQ
2:
	ldr	x9, [sp], #16
	eret
END(handle_el2_el1_irq)

ENTRY(handle_el2_el1_fiq64)
	do_world_switch_to_host
	mov	x0, #EXCP_TYPE_EL1_FIQ
	eret
END(handle_el2_el1_fiq64)

ENTRY(handle_el2_el1_error64)
	do_world_switch_to_host
	mov	x0, #EXCP_TYPE_EL1_ERROR
	eret
END(handle_el2_el1_error64)


/*
 * Usage:
 * void vmm_enter_guest(struct hypctx *hypctx)
 *
 * Expecting:
 * x0 - hypctx address
 */
ENTRY(vmm_enter_guest)
	/* Save hypctx address */
	msr	tpidr_el2, x0

	save_host_registers
#ifdef VFP
	/*
	 * Loading the guest VFP registers needs to come before loading the
	 * rest of the registers because this process dirties the regular
	 * registers.
	 */
	LOAD_GUEST_VFP_REGS()
#endif
	LOAD_GUEST_REGS()
	restore_guest_registers

	/* Enter guest */
	eret
END(vmm_enter_guest)


/*
 * Usage:
 * void vmm_cleanup(void *hyp_stub_vectors)
 *
 * Expecting:
 * x0 - physical address of hyp_stub_vectors
 */
ENTRY(vmm_cleanup)
	/* Restore the stub vectors */
	msr	vbar_el2, x0

	/* Disable the MMU */
	dsb	sy
	mrs	x2, sctlr_el2
	bic	x2, x2, #SCTLR_EL2_M
	msr	sctlr_el2, x2

	eret
END(vmm_cleanup)

.macro read_reg name
	mrs	x0, \name
.endm

/*
 * Return the value of the ICH_VTR_EL2 register.
 */
ENTRY(vmm_read_ich_vtr_el2)
	read_reg ich_vtr_el2
	eret
END(vmm_read_ich_vtr_el2)

/*
 * Return the value of the CNTHCTL_EL2 register.
 */
ENTRY(vmm_read_cnthctl_el2)
	read_reg cnthctl_el2
	eret
END(vmm_read_cnthctl_el2)

/*
 * Return the value of the TCR_EL2 register.
 */
ENTRY(vmm_read_tcr_el2)
	read_reg tcr_el2
	eret
END(vmm_read_tcr_el2)



hyp_code_end:
