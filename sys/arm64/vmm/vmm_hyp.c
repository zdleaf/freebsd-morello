/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Andrew Turner
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
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/proc.h>

#include <machine/armreg.h>

#include "arm64.h"
#include "hyp.h"

struct hypctx;

uint64_t vmm_hyp_enter(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
    uint64_t, uint64_t, uint64_t);
uint64_t vmm_enter_guest(struct hypctx *);
void vmm_cleanup(void *);

/* TODO: Make this common between this & vfp.h */
static void
vfp_store(struct vfpstate *state)
{
	__uint128_t *vfp_state;
	uint64_t fpcr, fpsr;

	vfp_state = state->vfp_regs;
	__asm __volatile(
	    "mrs	%0, fpcr		\n"
	    "mrs	%1, fpsr		\n"
	    "stp	q0,  q1,  [%2, #16 *  0]\n"
	    "stp	q2,  q3,  [%2, #16 *  2]\n"
	    "stp	q4,  q5,  [%2, #16 *  4]\n"
	    "stp	q6,  q7,  [%2, #16 *  6]\n"
	    "stp	q8,  q9,  [%2, #16 *  8]\n"
	    "stp	q10, q11, [%2, #16 * 10]\n"
	    "stp	q12, q13, [%2, #16 * 12]\n"
	    "stp	q14, q15, [%2, #16 * 14]\n"
	    "stp	q16, q17, [%2, #16 * 16]\n"
	    "stp	q18, q19, [%2, #16 * 18]\n"
	    "stp	q20, q21, [%2, #16 * 20]\n"
	    "stp	q22, q23, [%2, #16 * 22]\n"
	    "stp	q24, q25, [%2, #16 * 24]\n"
	    "stp	q26, q27, [%2, #16 * 26]\n"
	    "stp	q28, q29, [%2, #16 * 28]\n"
	    "stp	q30, q31, [%2, #16 * 30]\n"
	    : "=&r"(fpcr), "=&r"(fpsr) : "r"(vfp_state));

	state->vfp_fpcr = fpcr;
	state->vfp_fpsr = fpsr;
}

static void
vfp_restore(struct vfpstate *state)
{
	__uint128_t *vfp_state;
	uint64_t fpcr, fpsr;

	vfp_state = state->vfp_regs;
	fpcr = state->vfp_fpcr;
	fpsr = state->vfp_fpsr;

	__asm __volatile(
	    "ldp	q0,  q1,  [%2, #16 *  0]\n"
	    "ldp	q2,  q3,  [%2, #16 *  2]\n"
	    "ldp	q4,  q5,  [%2, #16 *  4]\n"
	    "ldp	q6,  q7,  [%2, #16 *  6]\n"
	    "ldp	q8,  q9,  [%2, #16 *  8]\n"
	    "ldp	q10, q11, [%2, #16 * 10]\n"
	    "ldp	q12, q13, [%2, #16 * 12]\n"
	    "ldp	q14, q15, [%2, #16 * 14]\n"
	    "ldp	q16, q17, [%2, #16 * 16]\n"
	    "ldp	q18, q19, [%2, #16 * 18]\n"
	    "ldp	q20, q21, [%2, #16 * 20]\n"
	    "ldp	q22, q23, [%2, #16 * 22]\n"
	    "ldp	q24, q25, [%2, #16 * 24]\n"
	    "ldp	q26, q27, [%2, #16 * 26]\n"
	    "ldp	q28, q29, [%2, #16 * 28]\n"
	    "ldp	q30, q31, [%2, #16 * 30]\n"
	    "msr	fpcr, %0		\n"
	    "msr	fpsr, %1		\n"
	    : : "r"(fpcr), "r"(fpsr), "r"(vfp_state));
}
static uint64_t
vmm_hyp_call_guest(struct hyp *hyp, int vcpu)
{
	struct hypctx *hypctx;
	uint64_t cntvoff_el2, vpidr_el2, vmpidr_el2, cptr_el2, spsr_el2;
	uint64_t ich_hcr_el2, ich_vmcr_el2, cnthctl_el2, cntkctl_el1, afsr1_el1;
	uint64_t contextidr_el1, cpacr_el1, esr_el1, far_el1, sctlr_el1;
	uint64_t spsr_el1, hcr_el2, actlr_el1, amair_el1, par_el1, mair_el1;
	uint64_t tcr_el1, ttbr0_el1, ttbr1_el1, afsr0_el1, sp_el0, sp_el1;
	uint64_t elr_el1, elr_el2, tpidr_el0, tpidrro_el0, tpidr_el1, vbar_el1;
	uint64_t ret;

	/* TODO: Check cpuid is valid */
	hypctx = &hyp->ctx[vcpu];

	/* Save the host special registers */
	/* TODO: Check which of these we need to save */
	cntvoff_el2 = READ_SPECIALREG(cntvoff_el2);
	vpidr_el2 = READ_SPECIALREG(vpidr_el2);
	vmpidr_el2 = READ_SPECIALREG(vmpidr_el2);
	cptr_el2 = READ_SPECIALREG(cptr_el2);
	spsr_el2 = READ_SPECIALREG(spsr_el2);
	ich_hcr_el2 = READ_SPECIALREG(ich_hcr_el2);
	ich_vmcr_el2 = READ_SPECIALREG(ich_vmcr_el2);
	cnthctl_el2 = READ_SPECIALREG(cnthctl_el2);
	cntkctl_el1 = READ_SPECIALREG(cntkctl_el1);
	afsr1_el1 = READ_SPECIALREG(afsr1_el1);
	contextidr_el1 = READ_SPECIALREG(contextidr_el1);
	cpacr_el1 = READ_SPECIALREG(cpacr_el1);
	esr_el1 = READ_SPECIALREG(esr_el1);
	far_el1 = READ_SPECIALREG(far_el1);
	sctlr_el1 = READ_SPECIALREG(sctlr_el1);
	spsr_el1 = READ_SPECIALREG(spsr_el1);
	hcr_el2 = READ_SPECIALREG(hcr_el2);
	actlr_el1 = READ_SPECIALREG(actlr_el1);
	amair_el1 = READ_SPECIALREG(amair_el1);
	par_el1 = READ_SPECIALREG(par_el1);
	mair_el1 = READ_SPECIALREG(mair_el1);
	tcr_el1 = READ_SPECIALREG(tcr_el1);
	ttbr0_el1 = READ_SPECIALREG(ttbr0_el1);
	ttbr1_el1 = READ_SPECIALREG(ttbr1_el1);
	afsr0_el1 = READ_SPECIALREG(afsr0_el1);
	sp_el0 = READ_SPECIALREG(sp_el0);
	sp_el1 = READ_SPECIALREG(sp_el1);
	elr_el1 = READ_SPECIALREG(elr_el1);
	elr_el2 = READ_SPECIALREG(elr_el2);
	tpidr_el0 = READ_SPECIALREG(tpidr_el0);
	tpidrro_el0 = READ_SPECIALREG(tpidrro_el0);
	tpidr_el1 = READ_SPECIALREG(tpidr_el1);
	vbar_el1 = READ_SPECIALREG(vbar_el1);

	/* Restore the guest special registers */
	WRITE_SPECIALREG(elr_el1, hypctx->elr_el1);
	WRITE_SPECIALREG(sp_el0, hypctx->sp_el0);
	WRITE_SPECIALREG(tpidr_el0, hypctx->tpidr_el0);
	WRITE_SPECIALREG(tpidrro_el0, hypctx->tpidrro_el0);
	WRITE_SPECIALREG(tpidr_el1, hypctx->tpidr_el1);
	WRITE_SPECIALREG(vbar_el1, hypctx->vbar_el1);
	WRITE_SPECIALREG(actlr_el1, hypctx->actlr_el1);
	WRITE_SPECIALREG(afsr0_el1, hypctx->afsr0_el1);
	WRITE_SPECIALREG(afsr1_el1, hypctx->afsr1_el1);
	WRITE_SPECIALREG(amair_el1, hypctx->amair_el1);
	WRITE_SPECIALREG(contextidr_el1, hypctx->contextidr_el1);
	WRITE_SPECIALREG(cpacr_el1, hypctx->cpacr_el1);
	WRITE_SPECIALREG(esr_el1, hypctx->esr_el1);
	WRITE_SPECIALREG(far_el1, hypctx->far_el1);
	WRITE_SPECIALREG(mair_el1, hypctx->mair_el1);
	WRITE_SPECIALREG(par_el1, hypctx->par_el1);
	WRITE_SPECIALREG(sctlr_el1, hypctx->sctlr_el1);
	WRITE_SPECIALREG(tcr_el1, hypctx->tcr_el1);
	WRITE_SPECIALREG(ttbr0_el1, hypctx->ttbr0_el1);
	WRITE_SPECIALREG(ttbr1_el1, hypctx->ttbr1_el1);
	WRITE_SPECIALREG(spsr_el1, hypctx->spsr_el1);
	WRITE_SPECIALREG(cptr_el2, hypctx->cptr_el2);
	WRITE_SPECIALREG(hcr_el2, hypctx->hcr_el2);
	WRITE_SPECIALREG(vpidr_el2, hypctx->vpidr_el2);
	WRITE_SPECIALREG(vmpidr_el2, hypctx->vmpidr_el2);

	/* Load the special regs from the trapframe */
	WRITE_SPECIALREG(sp_el1, hypctx->tf.tf_sp);
	WRITE_SPECIALREG(elr_el2, hypctx->tf.tf_elr);
	WRITE_SPECIALREG(spsr_el2, hypctx->tf.tf_spsr);

	/* Load the timer registers */
	WRITE_SPECIALREG(cntkctl_el1, hypctx->vtimer_cpu.cntkctl_el1);
	WRITE_SPECIALREG(cntv_cval_el0, hypctx->vtimer_cpu.cntv_cval_el0);
	WRITE_SPECIALREG(cntv_ctl_el0, hypctx->vtimer_cpu.cntv_ctl_el0);
	WRITE_SPECIALREG(cnthctl_el2, hyp->vtimer.cnthctl_el2);
	WRITE_SPECIALREG(cntvoff_el2, hyp->vtimer.cntvoff_el2);

	/* Load the GICv3 registers */
	WRITE_SPECIALREG(ich_hcr_el2, hypctx->vgic_cpu_if.ich_hcr_el2);
	WRITE_SPECIALREG(ich_vmcr_el2, hypctx->vgic_cpu_if.ich_vmcr_el2);
	switch(hypctx->vgic_cpu_if.ich_lr_num - 1) {
#define	LOAD_LR(x)					\
	case x:						\
		WRITE_SPECIALREG(ich_lr ## x ##_el2,	\
		    hypctx->vgic_cpu_if.ich_lr_el2[x])
	LOAD_LR(15);
	LOAD_LR(14);
	LOAD_LR(13);
	LOAD_LR(12);
	LOAD_LR(11);
	LOAD_LR(10);
	LOAD_LR(9);
	LOAD_LR(8);
	LOAD_LR(7);
	LOAD_LR(6);
	LOAD_LR(5);
	LOAD_LR(4);
	LOAD_LR(3);
	LOAD_LR(2);
	LOAD_LR(1);
	default:
	LOAD_LR(0);
#undef LOAD_LR
	}

	switch(hypctx->vgic_cpu_if.ich_ap0r_num - 1) {
#define	LOAD_APR(x)						\
	case x:							\
		WRITE_SPECIALREG(ich_ap0r ## x ##_el2,		\
		    hypctx->vgic_cpu_if.ich_ap0r_el2[x]);		\
		WRITE_SPECIALREG(ich_ap1r ## x ##_el2,		\
		    hypctx->vgic_cpu_if.ich_ap1r_el2[x])
	LOAD_APR(3);
	LOAD_APR(2);
	LOAD_APR(1);
	default:
	LOAD_APR(0);
#undef LOAD_APR
	}

	/* Load the common hypervisor registers */
	WRITE_SPECIALREG(vttbr_el2, hyp->vttbr_el2);

	/* Load the guest VFP registers */
	vfp_restore(&hypctx->vfpstate);

	/* Call into the guest */
	ret = vmm_enter_guest(hypctx);

	/* Store the guest VFP registers */
	vfp_store(&hypctx->vfpstate);

	/* Store the exit info */
	hypctx->exit_info.far_el2 = READ_SPECIALREG(far_el2);
	hypctx->exit_info.hpfar_el2 = READ_SPECIALREG(hpfar_el2);

	/* Store the special to from the trapframe */
	hypctx->tf.tf_sp = READ_SPECIALREG(sp_el1);
	hypctx->tf.tf_elr = READ_SPECIALREG(elr_el2);
	hypctx->tf.tf_spsr = READ_SPECIALREG(spsr_el2);
	hypctx->tf.tf_esr = READ_SPECIALREG(esr_el2);

	/* Store the guest special registers */
	hypctx->elr_el1 = READ_SPECIALREG(elr_el1);
	hypctx->sp_el0 = READ_SPECIALREG(sp_el0);
	hypctx->tpidr_el0 = READ_SPECIALREG(tpidr_el0);
	hypctx->tpidrro_el0 = READ_SPECIALREG(tpidrro_el0);
	hypctx->tpidr_el1 = READ_SPECIALREG(tpidr_el1);
	hypctx->vbar_el1 = READ_SPECIALREG(vbar_el1);
	hypctx->actlr_el1 = READ_SPECIALREG(actlr_el1);
	hypctx->afsr0_el1 = READ_SPECIALREG(afsr0_el1);
	hypctx->afsr1_el1 = READ_SPECIALREG(afsr1_el1);
	hypctx->amair_el1 = READ_SPECIALREG(amair_el1);
	hypctx->contextidr_el1 = READ_SPECIALREG(contextidr_el1);
	hypctx->cpacr_el1 = READ_SPECIALREG(cpacr_el1);
	hypctx->esr_el1 = READ_SPECIALREG(esr_el1);
	hypctx->far_el1 = READ_SPECIALREG(far_el1);
	hypctx->mair_el1 = READ_SPECIALREG(mair_el1);
	hypctx->par_el1 = READ_SPECIALREG(par_el1);
	hypctx->sctlr_el1 = READ_SPECIALREG(sctlr_el1);
	hypctx->tcr_el1 = READ_SPECIALREG(tcr_el1);
	hypctx->ttbr0_el1 = READ_SPECIALREG(ttbr0_el1);
	hypctx->ttbr1_el1 = READ_SPECIALREG(ttbr1_el1);
	hypctx->spsr_el1 = READ_SPECIALREG(spsr_el1);
	hypctx->cptr_el2 = READ_SPECIALREG(cptr_el2);
	hypctx->hcr_el2 = READ_SPECIALREG(hcr_el2);
	hypctx->vpidr_el2 = READ_SPECIALREG(vpidr_el2);
	hypctx->vmpidr_el2 = READ_SPECIALREG(vmpidr_el2);

	/* Store the timer registers */
	hypctx->vtimer_cpu.cntkctl_el1 = READ_SPECIALREG(cntkctl_el1);
	hypctx->vtimer_cpu.cntv_cval_el0 = READ_SPECIALREG(cntv_cval_el0);
	hypctx->vtimer_cpu.cntv_ctl_el0 = READ_SPECIALREG(cntv_ctl_el0);

	/* Store the GICv3 registers */
	hypctx->vgic_cpu_if.ich_eisr_el2 = READ_SPECIALREG(ich_eisr_el2);
	hypctx->vgic_cpu_if.ich_elrsr_el2 = READ_SPECIALREG(ich_elrsr_el2);
	hypctx->vgic_cpu_if.ich_hcr_el2 = READ_SPECIALREG(ich_hcr_el2);
	hypctx->vgic_cpu_if.ich_misr_el2 = READ_SPECIALREG(ich_misr_el2);
	hypctx->vgic_cpu_if.ich_vmcr_el2 = READ_SPECIALREG(ich_vmcr_el2);
	switch(hypctx->vgic_cpu_if.ich_lr_num - 1) {
#define	STORE_LR(x)					\
	case x:						\
		hypctx->vgic_cpu_if.ich_lr_el2[x] =	\
		    READ_SPECIALREG(ich_lr ## x ##_el2)
	STORE_LR(15);
	STORE_LR(14);
	STORE_LR(13);
	STORE_LR(12);
	STORE_LR(11);
	STORE_LR(10);
	STORE_LR(9);
	STORE_LR(8);
	STORE_LR(7);
	STORE_LR(6);
	STORE_LR(5);
	STORE_LR(4);
	STORE_LR(3);
	STORE_LR(2);
	STORE_LR(1);
	default:
	STORE_LR(0);
#undef STORE_LR
	}

	switch(hypctx->vgic_cpu_if.ich_ap0r_num - 1) {
#define	STORE_APR(x)						\
	case x:							\
		hypctx->vgic_cpu_if.ich_ap0r_el2[x] =		\
		    READ_SPECIALREG(ich_ap0r ## x ##_el2);	\
		hypctx->vgic_cpu_if.ich_ap1r_el2[x] =		\
		    READ_SPECIALREG(ich_ap1r ## x ##_el2)
	STORE_APR(3);
	STORE_APR(2);
	STORE_APR(1);
	default:
	STORE_APR(0);
#undef STORE_APR
	}

	/* Restore the host special registers */
	WRITE_SPECIALREG(vpidr_el2, vpidr_el2);
	WRITE_SPECIALREG(vmpidr_el2, vmpidr_el2);
	WRITE_SPECIALREG(cptr_el2, cptr_el2);
	WRITE_SPECIALREG(spsr_el2, spsr_el2);
	WRITE_SPECIALREG(ich_hcr_el2, ich_hcr_el2);
	WRITE_SPECIALREG(ich_vmcr_el2, ich_vmcr_el2);
	WRITE_SPECIALREG(cnthctl_el2, cnthctl_el2);
	WRITE_SPECIALREG(cntkctl_el1, cntkctl_el1);
	WRITE_SPECIALREG(afsr1_el1, afsr1_el1);
	WRITE_SPECIALREG(contextidr_el1, contextidr_el1);
	WRITE_SPECIALREG(cpacr_el1, cpacr_el1);
	WRITE_SPECIALREG(esr_el1, esr_el1);
	WRITE_SPECIALREG(far_el1, far_el1);
	WRITE_SPECIALREG(sctlr_el1, sctlr_el1);
	WRITE_SPECIALREG(spsr_el1, spsr_el1);
	WRITE_SPECIALREG(hcr_el2, hcr_el2);
	WRITE_SPECIALREG(actlr_el1, actlr_el1);
	WRITE_SPECIALREG(amair_el1, amair_el1);
	WRITE_SPECIALREG(par_el1, par_el1);
	WRITE_SPECIALREG(mair_el1, mair_el1);
	WRITE_SPECIALREG(tcr_el1, tcr_el1);
	WRITE_SPECIALREG(ttbr0_el1, ttbr0_el1);
	WRITE_SPECIALREG(ttbr1_el1, ttbr1_el1);
	WRITE_SPECIALREG(afsr0_el1, afsr0_el1);
	WRITE_SPECIALREG(sp_el0, sp_el0);
	WRITE_SPECIALREG(sp_el1, sp_el1);
	WRITE_SPECIALREG(elr_el1, elr_el1);
	WRITE_SPECIALREG(elr_el2, elr_el2);
	WRITE_SPECIALREG(tpidr_el0, tpidr_el0);
	WRITE_SPECIALREG(tpidrro_el0, tpidrro_el0);
	WRITE_SPECIALREG(tpidr_el1, tpidr_el1);
	WRITE_SPECIALREG(vbar_el1, vbar_el1);

	return (ret);
}

static uint64_t
vmm_hyp_read_reg(uint64_t reg)
{
	switch(reg) {
	case HYP_REG_ICH_VTR:
		return (READ_SPECIALREG(ich_vtr_el2));
	case HYP_REG_CNTHCTL:
		return (READ_SPECIALREG(cnthctl_el2));
	}

	return (0);
}

static bool
vmm_is_vpipt_cache(void)
{
	/* TODO: Implement */
	return (0);
}

static int
vmm_clean_s2_tlbi(void)
{
	dsb(ishst);
	__asm __volatile("tlbi alle1is");

	/*
	 * If we have a VPIPT icache it will use the VMID to tag cachelines.
	 * As we are changing the allocated VMIDs we need to invalidate the
	 * icache lines containing all old values.
	 */
	if (vmm_is_vpipt_cache())
		__asm __volatile("ic ialluis");
	dsb(ish);

	return (0);
}

uint64_t
vmm_hyp_enter(uint64_t handle, uint64_t x1, uint64_t x2, uint64_t x3,
    uint64_t x4, uint64_t x5, uint64_t x6, uint64_t x7)
{

	switch (handle) {
	case HYP_CLEANUP:
		vmm_cleanup((void *)x1);
		/* NOTREACHED */
		break;
	case HYP_ENTER_GUEST:
		return (vmm_hyp_call_guest((struct hyp *)x1, x2));
	case HYP_READ_REGISTER:
		return (vmm_hyp_read_reg(x1));
	case HYP_CLEAN_S2_TLBI:
		return (vmm_clean_s2_tlbi());
	default:
		break;
	}

	return (0);
}
