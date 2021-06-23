/*
 * Copyright (C) 2017 Alexandru Elisei <alexandru.elisei@gmail.com>
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

#ifndef _VMM_HYP_MACROS_H_
#define	_VMM_HYP_MACROS_H_


#define PUSH_SYS_REG_PAIR(reg0, reg1)			\
	mrs	x1, reg0;				\
	mrs	x2, reg1;				\
	stp	x2, x1, [sp, #-16]!;


#define PUSH_SYS_REG(reg)				\
	mrs 	x1, reg;				\
	str	x1, [sp, #-16]!;


#define POP_SYS_REG_PAIR(reg0, reg1)			\
	ldp	x2, x1, [sp], #16;			\
	msr	reg1, x2;				\
	msr	reg0, x1;


#define POP_SYS_REG(reg)				\
	ldr	x1, [sp], #16;				\
	msr	reg, x1;


#define	SAVE_ARRAY_REG64(reg, dest, remaining)		\
	cmp	remaining, #0;				\
	beq	9f;					\
	mrs	x7, reg;				\
	str	x7, [dest];				\
	add	dest, dest, #8;				\
	sub	remaining, remaining, #1;


#define	SAVE_LR_REGS()					\
	/* Load the number of ICH_LR_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_LR_NUM;		\
	ldr	x3, [x0, x2];				\
	/* x1 holds the destination address */		\
	mov	x1, #HYPCTX_VGIC_ICH_LR_EL2;		\
	add	x1, x0, x1;				\
	SAVE_ARRAY_REG64(ich_lr0_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr1_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr2_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr3_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr4_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr5_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr6_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr7_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr8_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr9_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr10_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr11_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr12_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr13_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr14_el2, x1, x3);		\
	SAVE_ARRAY_REG64(ich_lr15_el2, x1, x3);		\
9:;							\
	;


#define	SAVE_ARRAY_REG32(reg, dest, remaining)		\
	cmp	remaining, #0;				\
	beq	9f;					\
	mrs	x7, reg;				\
	str	w7, [dest];				\
	add	dest, dest, #4;				\
	sub	remaining, remaining, #1;


#define	SAVE_AP0R_REGS()				\
	/* Load the number of ICH_AP0R_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_AP0R_NUM;		\
	ldr	x3, [x0, x2];				\
	/* x1 holds the destination address */		\
	mov	x1, #HYPCTX_VGIC_ICH_AP0R_EL2;		\
	add	x1, x0, x1;				\
	SAVE_ARRAY_REG32(ich_ap0r0_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap0r1_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap0r2_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap0r3_el2, x1, x3);	\
9:;							\
	;


#define	SAVE_AP1R_REGS()				\
	/* Load the number of ICH_AP1R_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_AP1R_NUM;		\
	ldr	x3, [x0, x2];				\
	/* x1 holds the destination address */		\
	mov	x1, #HYPCTX_VGIC_ICH_AP1R_EL2;		\
	add	x1, x0, x1;				\
	SAVE_ARRAY_REG32(ich_ap1r0_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap1r1_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap1r2_el2, x1, x3);	\
	SAVE_ARRAY_REG32(ich_ap1r3_el2, x1, x3);	\
9:;							\
	;


/*
 * The STR and LDR instructions take an offset between [-256, 255], but the
 * hypctx register offset can be larger than that. To get around this limitation
 * we use a temporary register to hold the offset.
 */
#define	SAVE_SYS_REG64(prefix, reg)			\
	mrs	x1, reg;				\
	mov	x2, prefix ##_ ##reg;			\
	str	x1, [x0, x2];


#define	SAVE_SYS_REG32(prefix, reg)			\
	mrs	x1, reg;				\
	mov	x2, prefix ##_ ##reg;			\
	str	w1, [x0, x2];


#define	SAVE_REG(prefix, reg)				\
	mov	x1, prefix ##_ ##reg;			\
	str	reg, [x0, x1];

/*
 * The STP and LDP instructions takes an immediate in the range of [-512, 504]
 * when using the post-indexed addressing mode, but the hypctx register offset
 * can be larger than that. To get around this limitation we compute the address
 * by adding the hypctx base address with the struct member offset.
 *
 * Using STP/LDP to save/load register pairs to the corresponding struct hypctx
 * variables works because the registers are declared as an array and they are
 * stored in contiguous memory addresses.
 */

#define	SAVE_REG_PAIR(prefix, reg0, reg1)		\
	mov	x1, prefix ##_ ##reg0;			\
	add	x1, x0, x1;				\
	stp	reg0, reg1, [x1];


/*
 * Save all the guest registers. Start by saving the regular registers first
 * because those will be used as temporary registers for accessing the hypctx
 * member addresses.
 *
 * Expecting:
 * TPIDR_EL2 - struct hypctx address
 *
 * After call:
 * x0 - struct hypctx address
 */
#define	SAVE_GUEST_REGS()				\
	SAVE_REG(HYPCTX, FP);				\
							\
	SAVE_SYS_REG32(HYPCTX_VTIMER_CPU, CNTKCTL_EL1);	\
	SAVE_SYS_REG64(HYPCTX_VTIMER_CPU, CNTV_CVAL_EL0); \
	SAVE_SYS_REG32(HYPCTX_VTIMER_CPU, CNTV_CTL_EL0);\
							\
	/*						\
	 * ICH_EISR_EL2, ICH_ELRSR_EL2 and ICH_MISR_EL2 are read-only and are
	 * saved because they are modified by the hardware as part of the
	 * interrupt virtualization process and we need to inspect them in
	 * the VGIC driver.
	 */						\
	SAVE_SYS_REG32(HYPCTX_VGIC, ICH_EISR_EL2);	\
	SAVE_SYS_REG32(HYPCTX_VGIC, ICH_ELRSR_EL2);	\
	SAVE_SYS_REG32(HYPCTX_VGIC, ICH_MISR_EL2);	\
	SAVE_SYS_REG32(HYPCTX_VGIC, ICH_HCR_EL2);	\
	SAVE_SYS_REG32(HYPCTX_VGIC, ICH_VMCR_EL2);	\
							\
	SAVE_LR_REGS();					\
	SAVE_AP0R_REGS();				\
	SAVE_AP1R_REGS();				\
							\
	/* Save the stack pointer. */			\
	mrs	x1, sp_el1;				\
	mov	x2, #HYPCTX_REGS_SP;			\
	str	x1, [x0, x2];				\
							\
	SAVE_SYS_REG64(HYPCTX, ACTLR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, AFSR0_EL1);		\
	SAVE_SYS_REG64(HYPCTX, AFSR1_EL1);		\
	SAVE_SYS_REG64(HYPCTX, AMAIR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, CONTEXTIDR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, CPACR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, ELR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, ESR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, FAR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, MAIR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, PAR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, SCTLR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, SP_EL0);			\
	SAVE_SYS_REG64(HYPCTX, TCR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, TPIDR_EL0);		\
	SAVE_SYS_REG64(HYPCTX, TPIDRRO_EL0);		\
	SAVE_SYS_REG64(HYPCTX, TPIDR_EL1);		\
	SAVE_SYS_REG64(HYPCTX, TTBR0_EL1);		\
	SAVE_SYS_REG64(HYPCTX, TTBR1_EL1);		\
	SAVE_SYS_REG64(HYPCTX, VBAR_EL1);		\
							\
	SAVE_SYS_REG32(HYPCTX, SPSR_EL1);		\
							\
	SAVE_SYS_REG64(HYPCTX, CPTR_EL2);		\
	SAVE_SYS_REG64(HYPCTX, ELR_EL2);		\
	SAVE_SYS_REG64(HYPCTX, HCR_EL2);		\
	SAVE_SYS_REG64(HYPCTX, VPIDR_EL2);		\
	SAVE_SYS_REG64(HYPCTX, VMPIDR_EL2);		\
	SAVE_SYS_REG32(HYPCTX, SPSR_EL2);


#define	SAVE_GUEST_VFP_REGS()				\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q0, Q1);		\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q2, Q3);		\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q4, Q5);		\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q6, Q7);		\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q8, Q9);		\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q10, Q11);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q12, Q13);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q14, Q15);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q16, Q17);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q18, Q19);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q20, Q21);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q22, Q23);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q24, Q25);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q26, Q27);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q28, Q29);	\
	SAVE_REG_PAIR(HYPCTX_VFPSTATE, Q30, Q31);	\
							\
	SAVE_SYS_REG32(HYPCTX_VFPSTATE, FPCR);		\
	SAVE_SYS_REG32(HYPCTX_VFPSTATE, FPSR);


/* See SAVE_SYS_REG */
#define	LOAD_SYS_REG64(prefix, reg)			\
	mov	x1, prefix ##_ ##reg;			\
	ldr	x2, [x0, x1];				\
	msr	reg, x2;


#define	LOAD_SYS_REG32(prefix, reg)			\
	mov	x1, prefix ##_ ##reg;			\
	ldr	w2, [x0, x1];				\
	msr	reg, x2;


/* See SAVE_REG_PAIR */
#define LOAD_REG_PAIR(prefix, reg0, reg1)		\
	mov	x1, prefix ##_ ##reg0;			\
	add	x1, x0, x1;				\
	ldp	reg0, reg1, [x1];


#define	LOAD_GUEST_VFP_REGS()				\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q0, Q1);		\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q2, Q3);		\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q4, Q5);		\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q6, Q7);		\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q8, Q9);		\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q10, Q11);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q12, Q13);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q14, Q15);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q16, Q17);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q18, Q19);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q20, Q21);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q22, Q23);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q24, Q25);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q26, Q27);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q28, Q29);	\
	LOAD_REG_PAIR(HYPCTX_VFPSTATE, Q30, Q31);	\
							\
	LOAD_SYS_REG32(HYPCTX_VFPSTATE, FPCR);		\
	LOAD_SYS_REG32(HYPCTX_VFPSTATE, FPSR);


#define	LOAD_REG(prefix, reg)				\
	mov	x1, prefix ##_ ##reg;			\
	ldr	reg, [x0, x1];


#define	LOAD_ARRAY_REG64(reg, src, remaining)		\
	cmp	remaining, #0;				\
	beq	9f;					\
	ldr	x2, [src];				\
	msr	reg, x2;				\
	add	src, src, #8;				\
	sub	remaining, remaining, #1;


#define	LOAD_LR_REGS();					\
	/* Load the number of ICH_LR_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_LR_NUM;		\
	ldr	x3, [x0, x2];				\
	mov	x1, #HYPCTX_VGIC_ICH_LR_EL2;		\
	/* x1 holds the load address */			\
	add	x1, x0, x1;				\
	LOAD_ARRAY_REG64(ich_lr0_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr1_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr2_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr3_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr4_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr5_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr6_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr7_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr8_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr9_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr10_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr11_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr12_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr13_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr14_el2, x1, x3);		\
	LOAD_ARRAY_REG64(ich_lr15_el2, x1, x3);		\
9:;							\
	;


#define	LOAD_ARRAY_REG32(reg, src, remaining)		\
	cmp	remaining, #0;				\
	beq	9f;					\
	ldr	w2, [src];				\
	msr	reg, x2;				\
	add	src, src, #4;				\
	sub	remaining, remaining, #1;


#define	LOAD_AP0R_REGS();				\
	/* Load the number of ICH_AP0R_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_AP0R_NUM;		\
	ldr	x3, [x0, x2];				\
	/* x1 holds the load address */			\
	mov	x1, #HYPCTX_VGIC_ICH_AP0R_EL2;		\
	add	x1, x0, x1;				\
	LOAD_ARRAY_REG32(ich_ap0r0_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap0r1_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap0r2_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap0r3_el2, x1, x3);	\
9:;							\
	;


#define	LOAD_AP1R_REGS();				\
	/* Load the number of ICH_AP1R_EL2 regs from memory */ \
	mov	x2, #HYPCTX_VGIC_ICH_AP1R_NUM;		\
	ldr	x3, [x0, x2];				\
	/* x1 holds the load address */			\
	mov	x1, #HYPCTX_VGIC_ICH_AP1R_EL2;		\
	add	x1, x0, x1;				\
	LOAD_ARRAY_REG32(ich_ap1r0_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap1r1_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap1r2_el2, x1, x3);	\
	LOAD_ARRAY_REG32(ich_ap1r3_el2, x1, x3);	\
9:;							\
	;



#define KTOHYP_REG(reg)					\
	mov	x7, HYP_KVA_MASK;			\
	and	reg, reg, x7;				\
	mov	x7, HYP_KVA_OFFSET;			\
	orr	reg, reg, x7;


/* Load a register from struct hyp *hyp member of hypctx. */
#define	LOAD_HYP_REG(prefix, reg)			\
	/* Compute VA of hyp member in x1 */ 		\
	mov	x1, #HYPCTX_HYP;			\
	add	x1, x1, x0;				\
	/* Get hyp address in x2 */			\
	ldr	x2, [x1];				\
	/* Transform hyp kernel VA into an EL2 VA */	\
	KTOHYP_REG(x2);					\
	/* Get register offset inside struct hyp */	\
	mov	x1, prefix ##_ ##reg;			\
	/* Compute regster address */			\
	add	x2, x2, x1;				\
	/* Load the register */				\
	ldr	x1, [x2];				\
	msr	reg, x1;				\


/*
 * Restore all the guest registers to their original values.
 *
 * Expecting:
 * x0 - struct hypctx address
 *
 * After call:
 * tpidr_el2 - struct hypctx address
 */
#define	LOAD_GUEST_REGS()				\
	LOAD_SYS_REG64(HYPCTX, ACTLR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, AFSR0_EL1);		\
	LOAD_SYS_REG64(HYPCTX, AFSR1_EL1);		\
	LOAD_SYS_REG64(HYPCTX, AMAIR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, CONTEXTIDR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, CPACR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, ELR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, ESR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, FAR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, MAIR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, PAR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, SCTLR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, SP_EL0);			\
	LOAD_SYS_REG64(HYPCTX, TCR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, TPIDR_EL0);		\
	LOAD_SYS_REG64(HYPCTX, TPIDRRO_EL0);		\
	LOAD_SYS_REG64(HYPCTX, TPIDR_EL1);		\
	LOAD_SYS_REG64(HYPCTX, TTBR0_EL1);		\
	LOAD_SYS_REG64(HYPCTX, TTBR1_EL1);		\
	LOAD_SYS_REG64(HYPCTX, VBAR_EL1);		\
	LOAD_SYS_REG32(HYPCTX, SPSR_EL1);		\
							\
	LOAD_SYS_REG64(HYPCTX, CPTR_EL2);		\
	LOAD_SYS_REG64(HYPCTX, ELR_EL2);		\
	LOAD_SYS_REG64(HYPCTX, HCR_EL2);		\
	LOAD_SYS_REG64(HYPCTX, VPIDR_EL2);		\
	LOAD_SYS_REG64(HYPCTX, VMPIDR_EL2);		\
	LOAD_SYS_REG32(HYPCTX, SPSR_EL2);		\
							\
	LOAD_SYS_REG32(HYPCTX_VGIC, ICH_HCR_EL2);	\
	LOAD_SYS_REG32(HYPCTX_VGIC, ICH_VMCR_EL2);	\
							\
	LOAD_SYS_REG32(HYPCTX_VTIMER_CPU, CNTKCTL_EL1);	\
	LOAD_SYS_REG64(HYPCTX_VTIMER_CPU, CNTV_CVAL_EL0); \
	LOAD_SYS_REG32(HYPCTX_VTIMER_CPU, CNTV_CTL_EL0); \
							\
	LOAD_REG(HYPCTX, FP);				\
							\
	LOAD_HYP_REG(HYP, VTTBR_EL2);			\
	LOAD_HYP_REG(HYP_VTIMER, CNTHCTL_EL2);		\
	LOAD_HYP_REG(HYP_VTIMER, CNTVOFF_EL2);		\
							\
	LOAD_LR_REGS();					\
	LOAD_AP0R_REGS();				\
	LOAD_AP1R_REGS();				\
							\
	/* Load the guest EL1 stack pointer */		\
	mov	x1, #HYPCTX_REGS_SP;			\
	add	x1, x1, x0;				\
	ldr	x2, [x1];				\
	msr	sp_el1, x2;				\


/*
 * Save exit information
 *
 * Expecting:
 * x0 - struct hypctx address
 */
#define	SAVE_EXIT_INFO()				\
	SAVE_SYS_REG64(HYPCTX_EXIT_INFO, ESR_EL2);	\
	SAVE_SYS_REG64(HYPCTX_EXIT_INFO, FAR_EL2);	\
	SAVE_SYS_REG64(HYPCTX_EXIT_INFO, HPFAR_EL2);	\

#endif /* !_VMM_HYP_MACROS_H_ */
