/*
 * Copyright (C) 2018 Alexandru Elisei <alexandru.elisei@gmail.com>
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

#include <sys/cdefs.h>

#include <sys/types.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/smp.h>
#include <sys/bitstring.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/cpu.h>
#include <machine/machdep.h>
#include <machine/param.h>
#include <machine/pmap.h>
#include <machine/vmparam.h>
#include <machine/intr.h>
#include <machine/vmm.h>
#include <machine/vmm_instruction_emul.h>

#include <arm/arm/gic_common.h>
#include <arm/arm/generic_timer.h>
#include <arm64/arm64/gic_v3_reg.h>
#include <arm64/arm64/gic_v3_var.h>

#include <arm64/vmm/hyp.h>
#include <arm64/vmm/mmu.h>
#include <arm64/vmm/arm64.h>

#include "vgic_v3.h"
#include "vgic_v3_reg.h"

#define VGIC_V3_DEVNAME		"vgic"
#define VGIC_V3_DEVSTR		"ARM Virtual Generic Interrupt Controller v3"

#define	RES0			0UL

#define	IRQBUF_SIZE_MIN		32
#define	IRQBUF_SIZE_MAX		(1 << 10)

#define	IRQ_SCHEDULED		(GIC_LAST_SPI + 1)

#define	lr_pending(lr)		\
    (ICH_LR_EL2_STATE(lr) == ICH_LR_EL2_STATE_PENDING)
#define	lr_inactive(lr)		\
    (ICH_LR_EL2_STATE(lr) == ICH_LR_EL2_STATE_INACTIVE)
#define lr_active(lr)		\
    (ICH_LR_EL2_STATE(lr) == ICH_LR_EL2_STATE_ACTIVE)
#define lr_pending_active(lr)	\
    (ICH_LR_EL2_STATE(lr) == ICH_LR_EL2_STATE_PENDING_ACTIVE)
#define	lr_not_active(lr) (!lr_active(lr) && !lr_pending_active(lr))

#define	lr_clear_irq(lr) ((lr) &= ~ICH_LR_EL2_STATE_MASK)

MALLOC_DEFINE(M_VGIC_V3, "ARM VMM VGIC V3", "ARM VMM VGIC V3");

struct vgic_v3_virt_features {
	uint8_t min_prio;
	size_t ich_lr_num;
	size_t ich_ap0r_num;
	size_t ich_ap1r_num;
};

struct vgic_v3_ro_regs {
	uint32_t gicd_icfgr0;
	uint32_t gicd_pidr2;
	uint32_t gicd_typer;
};

#define	vip_to_lr(vip, lr)						\
do {									\
	lr = ICH_LR_EL2_STATE_PENDING;					\
	lr |= ICH_LR_EL2_GROUP1;					\
	lr |= (uint64_t)vip->priority << ICH_LR_EL2_PRIO_SHIFT;		\
	lr |= vip->irq;							\
} while (0)

#define	lr_to_vip(lr, vip)						\
do {									\
	(vip)->irq = ICH_LR_EL2_VINTID(lr);				\
	(vip)->priority = \
	    (uint8_t)(((lr) & ICH_LR_EL2_PRIO_MASK) >> ICH_LR_EL2_PRIO_SHIFT); \
} while (0)

static struct vgic_v3_virt_features virt_features;
static struct vgic_v3_ro_regs ro_regs;

static struct gic_v3_softc *gic_sc;

static struct vgic_v3_irq *vgic_v3_get_irq(struct hyp *, int, uint32_t);
static void vgic_v3_release_irq(struct vgic_v3_irq *);

void
vgic_v3_cpuinit(void *arg, bool last_vcpu)
{
	struct hypctx *hypctx = arg;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;
	struct vgic_v3_redist *redist = &hypctx->vgic_redist;
	struct vgic_v3_irq *irq;
	uint64_t aff, vmpidr_el2;
	int i, irqid;

	vmpidr_el2 = hypctx->vmpidr_el2;
	KASSERT(vmpidr_el2 != 0,
	    ("Trying to init this CPU's vGIC before the vCPU"));
	/*
	 * Get affinity for the current CPU. The guest CPU affinity is taken
	 * from VMPIDR_EL2. The Redistributor corresponding to this CPU is
	 * the Redistributor with the same affinity from GICR_TYPER.
	 */
	aff = (CPU_AFF3(vmpidr_el2) << 24) | (CPU_AFF2(vmpidr_el2) << 16) |
	    (CPU_AFF1(vmpidr_el2) << 8) | CPU_AFF0(vmpidr_el2);

	/* Set up GICR_TYPER. */
	redist->gicr_typer = aff << GICR_TYPER_AFF_SHIFT;
	/* Set the vcpu as the processsor ID */
	redist->gicr_typer |= hypctx->vcpu << GICR_TYPER_CPUNUM_SHIFT;
	/* Redistributor doesn't support virtual LPIS. */
	redist->gicr_typer &= ~GICR_TYPER_VLPIS;
	redist->gicr_typer |= GICR_TYPER_PLPIS;

	if (last_vcpu)
		/* Mark the last Redistributor */
		redist->gicr_typer |= GICR_TYPER_LAST;

	/*
	 * Configure the Redistributor Control Register.
	 *
	 * ~GICR_CTLR_LPI_ENABLE: LPIs are disabled
	 */
	redist->gicr_ctlr = 0 & ~GICR_CTLR_LPI_ENABLE;


	redist->gicr_propbaser =
	    (GICR_PROPBASER_SHARE_OS << GICR_PROPBASER_SHARE_SHIFT) |
	    (GICR_PROPBASER_CACHE_NIWAWB << GICR_PROPBASER_CACHE_SHIFT);
	redist->gicr_pendbaser =
	    (GICR_PENDBASER_SHARE_OS << GICR_PENDBASER_SHARE_SHIFT) |
	    (GICR_PENDBASER_CACHE_NIWAWB << GICR_PENDBASER_CACHE_SHIFT);

	/* TODO: We need a call to mtx_destroy */
	mtx_init(&cpu_if->lr_mtx, "VGICv3 ICH_LR_EL2 lock", NULL, MTX_SPIN);

	/* Set the SGI and PPI state */
	for (irqid = 0; irqid < VGIC_PRV_I_NUM; irqid++) {
		irq = &cpu_if->private_irqs[irqid];

		/* TODO: We need a call to mtx_destroy */
		mtx_init(&irq->irq_spinmtx, "VGIC IRQ spinlock", NULL,
		    MTX_SPIN);
		irq->irq = irqid;
		irq->mpidr = hypctx->vmpidr_el2 & GICD_AFF;
		irq->irqtype = VGIC_IRQ_MISC;
		if (irqid < VGIC_SGI_NUM) {
			/* SGIs */
			irq->enabled = 1;
			irq->config = VGIC_CONFIG_EDGE;
		} else {
			/* PPIs */
			irq->config = VGIC_CONFIG_LEVEL;
		}
		irq->priority = 0;
	}

	/*
	 * Configure the Interrupt Controller Hyp Control Register.
	 *
	 * ICH_HCR_EL2_En: enable virtual CPU interface.
	 *
	 * Maintenance interrupts are disabled.
	 */
	cpu_if->ich_hcr_el2 = ICH_HCR_EL2_En;

	/*
	 * Configure the Interrupt Controller Virtual Machine Control Register.
	 *
	 * ICH_VMCR_EL2_VPMR: lowest priority mask for the VCPU interface
	 * ICH_VMCR_EL2_VBPR1_NO_PREEMPTION: disable interrupt preemption for
	 * Group 1 interrupts
	 * ICH_VMCR_EL2_VBPR0_NO_PREEMPTION: disable interrupt preemption for
	 * Group 0 interrupts
	 * ~ICH_VMCR_EL2_VEOIM: writes to EOI registers perform priority drop
	 * and interrupt deactivation.
	 * ICH_VMCR_EL2_VENG0: virtual Group 0 interrupts enabled.
	 * ICH_VMCR_EL2_VENG1: virtual Group 1 interrupts enabled.
	 */
	cpu_if->ich_vmcr_el2 = \
	    (virt_features.min_prio << ICH_VMCR_EL2_VPMR_SHIFT) | \
	    ICH_VMCR_EL2_VBPR1_NO_PREEMPTION | ICH_VMCR_EL2_VBPR0_NO_PREEMPTION;
	cpu_if->ich_vmcr_el2 &= ~ICH_VMCR_EL2_VEOIM;
	cpu_if->ich_vmcr_el2 |= ICH_VMCR_EL2_VENG0 | ICH_VMCR_EL2_VENG1;

	cpu_if->ich_lr_num = virt_features.ich_lr_num;
	for (i = 0; i < cpu_if->ich_lr_num; i++)
		cpu_if->ich_lr_el2[i] = 0UL;
	cpu_if->ich_lr_used = 0;
	TAILQ_INIT(&cpu_if->irq_act_pend);

	cpu_if->ich_ap0r_num = virt_features.ich_ap0r_num;
	cpu_if->ich_ap1r_num = virt_features.ich_ap1r_num;
}

void
vgic_v3_vminit(void *arg)
{
	struct hyp *hyp = arg;
	struct vgic_v3_dist *dist = &hyp->vgic_dist;

	/*
	 * Configure the Distributor control register. The register resets to an
	 * architecturally UNKNOWN value, so we reset to 0 to disable all
	 * functionality controlled by the register.
	 *
	 * The exception is GICD_CTLR.DS, which is RA0/WI when the Distributor
	 * supports one security state (ARM GIC Architecture Specification for
	 * GICv3 and GICv4, p. 4-464)
	 */
	dist->gicd_ctlr = 0;

	dist->gicd_typer = ro_regs.gicd_typer;
	dist->nirqs = GICD_TYPER_I_NUM(dist->gicd_typer);
	dist->gicd_pidr2 = ro_regs.gicd_pidr2;

	mtx_init(&dist->dist_mtx, "VGICv3 Distributor lock", NULL, MTX_SPIN);
}

static uint64_t
read_enabler(struct hyp *hyp, int vcpuid, int n)
{
	struct vgic_v3_irq *irq;
	uint64_t ret;
	uint32_t irq_base;
	int i;

	ret = 0;
	irq_base = n * 32;
	for (i = 0; i < 32; i++) {
		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		if (irq->enabled != 0)
			ret |= 1u << i;
		vgic_v3_release_irq(irq);
	}

	return (ret);
}

static void
write_enabler(struct hyp *hyp, int vcpuid, int n, bool set, uint64_t val)
{
	struct vgic_v3_irq *irq;
	uint32_t irq_base;
	int i;

	irq_base = n * 32;
	for (i = 0; i < 32; i++) {
		/* We only change interrupts when the appropriate bit is set */
		if ((val & (1u << i)) == 0)
			continue;

		/* Find the interrupt this bit represents */
		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		/* TODO: Clear from lr if set == false? */
		/* TODO: Manage when irq is a level interrupt */
		irq->enabled = set ? 1 : 0;
		vgic_v3_release_irq(irq);
	}
}

static uint64_t
read_priorityr(struct hyp *hyp, int vcpuid, int n)
{
	struct vgic_v3_irq *irq;
	uint64_t ret;
	uint32_t irq_base;
	int i;

	ret = 0;
	irq_base = n * 4;
	for (i = 0; i < 4; i++) {
		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		ret |= ((uint64_t)irq->priority) << (i * 8);
		vgic_v3_release_irq(irq);
	}

	return (ret);
}

static void
write_priorityr(struct hyp *hyp, int vcpuid, int n, uint64_t val)
{
	struct vgic_v3_irq *irq;
	uint32_t irq_base;
	int i;

	irq_base = n * 4;
	for (i = 0; i < 4; i++) {
		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		/* Set the priority. We support 32 priority steps (5 bits) */
		irq->priority = (val >> (i * 8)) & 0xf8;
		vgic_v3_release_irq(irq);
	}
}

static uint64_t
read_config(struct hyp *hyp, int vcpuid, int n)
{
	struct vgic_v3_irq *irq;
	uint64_t ret;
	uint32_t irq_base;
	int i;

	ret = 0;
	irq_base = n * 16;
	for (i = 0; i < 16; i++) {
		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		ret |= ((uint64_t)irq->config) << (i * 2);
		vgic_v3_release_irq(irq);
	}

	return (ret);
}

static void
write_config(struct hyp *hyp, int vcpuid, int n, uint64_t val)
{
	struct vgic_v3_irq *irq;
	uint32_t irq_base;
	int i;

	irq_base = n * 16;
	for (i = 0; i < 16; i++) {
		/*
		 * The config can't be changed for SGIs and PPIs. SGIs have
		 * an edge-triggered behaviour, and the register is
		 * implementation defined to be read-only for PPIs.
		 */
		if (irq_base + i < VGIC_PRV_I_NUM)
			continue;

		irq = vgic_v3_get_irq(hyp, vcpuid, irq_base + i);
		if (irq == NULL)
			continue;

		/* Bit 0 is RES0 */
		irq->config = (val >> (i * 2)) & VGIC_CONFIG_MASK;
		vgic_v3_release_irq(irq);
	}
}

static uint64_t
read_route(struct hyp *hyp, int vcpuid, int n)
{
	struct vgic_v3_irq *irq;
	uint64_t mpidr;

	irq = vgic_v3_get_irq(hyp, vcpuid, n);
	if (irq == NULL)
		return (0);

	mpidr = irq->mpidr;
	vgic_v3_release_irq(irq);

	return (mpidr);
}

static void
write_route(struct hyp *hyp, int vcpuid, int n, uint64_t val)
{
	struct vgic_v3_irq *irq;

	irq = vgic_v3_get_irq(hyp, vcpuid, n);
	if (irq == NULL)
		return;

	/* TODO: Support Interrupt Routing Mode */
	irq->mpidr = val & GICD_AFF;
	vgic_v3_release_irq(irq);
}

static int
dist_read(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t *rval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_v3_dist *dist = &hyp->vgic_dist;
	bool *retu = arg;
	uint64_t reg;
	int n;

	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < dist->start || fault_ipa + size > dist->end ||
	    size != 4) {
		return (EINVAL);
	}

	reg = fault_ipa - dist->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch(reg) {
	default:
		break;
	case GICD_CTLR:
		mtx_lock_spin(&dist->dist_mtx);
		*rval = dist->gicd_ctlr;
		mtx_unlock_spin(&dist->dist_mtx);

		/* Writes are never pending */
		*rval &= ~GICD_CTLR_RWP;

		*retu = false;
		return (0);
	case GICD_TYPER:
		*rval = dist->gicd_typer;
		*retu = false;
		return (0);
	case GICD_PIDR2:
		*rval = dist->gicd_pidr2;
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IGROUPR(0) && /* 0x0080 */
	    reg < GICD_IGROUPR(1024)) {

		/*
		 * GIC Architecture specification, p 8-477: "For SGIs and PPIs:
		 * When ARE is 1 for the Security state of an interrupt, the
		 * field for that interrupt is RES0 and an implementation is
		 * permitted to make the field RAZ/WI in this case".
		 */
		if (reg == GICD_IGROUPR(0) && aff_routing_en(dist))
			*rval = 0;
		else
			*rval = 0xffffffff;
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ISENABLER(0) &&   /* 0x0100 */
	    reg < GICD_ISENABLER(1024)) { /* 0x0180 */
		n = (reg - GICD_ISENABLER(0)) / 4;
		/* The first register is RAZ as affinity routing is on */
		if (n == 0)
			*rval = 0;
		else
			*rval = read_enabler(hyp, vcpuid, n);
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICENABLER(0) &&   /* 0x0180 */
	    reg < GICD_ICENABLER(1024)) { /* 0x0200 */
		n = (reg - GICD_ICENABLER(0)) / 4;
		/* The first register is RAZ as affinity routing is on */
		if (n == 0)
			*rval = 0;
		else
			*rval = read_enabler(hyp, vcpuid, n);
		*retu = false;
		return (0);
	}

	/* TODO: GICD_ISPENDR 0x0200 */
	/* TODO: GICD_ICPENDR 0x0280 */
	/* TODO: GICD_ICACTIVER 0x0380 */

	if (reg >= GICD_IPRIORITYR(0) &&   /* 0x0400 */
	    reg < GICD_IPRIORITYR(1024)) { /* 0x0800 */
		n = (reg - GICD_IPRIORITYR(0)) / 4;
		/* The first 8 registers are RAZ as affinity routing is on */
		if (n <= 7) {
			*rval = 0;
		} else {
			*rval = read_priorityr(hyp, vcpuid, n);
		}
		*retu = false;
		return (0);
	}

	/* TODO: GICD_ITARGETSR 0x0800 */

	if (reg >= GICD_ICFGR(0) && /* 0x0C00 */
	    reg < (GICD_ICFGR(1024))) {
		n = (reg - GICD_ICFGR(0)) / 4;
		/* The first 2 registers are RAZ as affinity routing is on */
		if (n <= 1) {
			*rval = 0;
		} else {
			*rval = read_config(hyp, vcpuid, n);
		}
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IROUTER(0) && /* 0x6000 */
	    reg < GICD_IROUTER(1024)) {
		n = (reg - GICD_IROUTER(0)) / 8;
		if (n <= 31) {
			*rval = 0;
		} else {
			*rval = read_route(hyp, vcpuid, n);
		}
		*retu = false;
		return (0);
	}

	panic("%s: %lx\n", __func__, fault_ipa - dist->start);
	return (0);
}

static int
dist_write(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t wval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_v3_dist *dist = &hyp->vgic_dist;
	bool *retu = arg;
	uint64_t reg;
	int n;

	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < dist->start || fault_ipa + size > dist->end) {
		return (EINVAL);
	}

	reg = fault_ipa - dist->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch(reg) {
	default:
		break;
	case GICD_CTLR:
		/*
		 * GICD_CTLR.DS is RAO/WI when only one security
		 * state is supported.
		 */
		/* TODO: Add the GICD_CTLR_DS macro */
		wval |= GICD_CTLR_ARE_NS;
		mtx_lock_spin(&dist->dist_mtx);
		dist->gicd_ctlr = wval;
		/* TODO: Wake any vcpus that have interrupts pending */
		mtx_unlock_spin(&dist->dist_mtx);

		*retu = false;
		return (0);
	case GICD_TYPER:
		*retu = false;
		return (0);
	case GICD_PIDR2:
		*retu = false;
		return (0);
	}

	/* Only group 1 interrupts are supported. Treat IGROUPR as RA0/WI. */
	if (reg >= GICD_IGROUPR(0) && /* 0x0080 */
	    reg < (GICD_IGROUPR(1024))) {
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ISENABLER(0) && /* 0x0100 */
	    reg < GICD_ISENABLER(1024)) {
		n = (reg - GICD_ISENABLER(0)) / 4;
		write_enabler(hyp, vcpuid, n, true, wval);
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICENABLER(0) && /* 0x0180 */
	    reg < GICD_ICENABLER(1024)) {
		n = (reg - GICD_ICENABLER(0)) / 4;
		write_enabler(hyp, vcpuid, n, false, wval);
		*retu = false;
		return (0);
	}

	/* TODO: GICD_ISPENDR 0x0200 */
	/* TODO: GICD_ICPENDR 0x0280 */

	/* TODO: GICD_ICACTIVER 0x0380 */
	if (reg >= GICD_ICACTIVER(0) && /* 0x0380 */
	    reg < GICD_ICACTIVER(1024)) {
		/* TODO: Implement */
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IPRIORITYR(0) &&   /* 0x0400 */
	    reg < GICD_IPRIORITYR(1024)) { /* 0x0800 */
		n = (reg - GICD_IPRIORITYR(0)) / 4;
		/* The first 8 registers are WI as affinity routing is on */
		if (n > 7) {
			write_priorityr(hyp, vcpuid, n, wval);
		}
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICFGR(0) &&
	    reg < GICD_ICFGR(1024)) {
		n = (reg - GICD_ICFGR(0)) / 4;
		/* XXX */
		if (n > 1) {
			write_config(hyp, vcpuid, n, wval);
		}
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IROUTER(0) && /* 0x6000 */
	    reg < GICD_IROUTER(1024)) {
		n = (reg - GICD_IROUTER(0)) / 8;
		if (n > 31) {
			write_route(hyp, vcpuid, n, wval);
		}
		*retu = false;
		return (0);
	}

	panic("%s: %lx\n", __func__, fault_ipa - dist->start);
	return (0);
}

static int
redist_read(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t *rval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_v3_redist *redist = &hyp->ctx[vcpuid].vgic_redist;
	bool *retu = arg;
	uint64_t reg;
	int n;

	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < redist->start || fault_ipa + size > redist->end) {
		return (EINVAL);
	}

	reg = fault_ipa - redist->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch (reg) {
	case GICR_CTLR:
		*rval = redist->gicr_ctlr;
		*retu = false;
		return (0);
	case GICR_TYPER:
		*rval = redist->gicr_typer;
		*retu = false;
		return (0);
	case GICR_WAKER:
		*rval = 0;
		*retu = false;
		return (0);
	case GICR_PROPBASER:
		*rval = redist->gicr_propbaser;
		*retu = false;
		return (0);
	case GICR_PENDBASER:
		*rval = redist->gicr_pendbaser;
		*retu = false;
		return (0);
	case GICR_PIDR2:
		*rval = hyp->vgic_dist.gicd_pidr2;
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_IGROUPR0:
		*rval = ~0ul;
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ISENABLER0:
	case GICR_SGI_BASE_SIZE + GICR_ICENABLER0:
		*rval = read_enabler(hyp, vcpuid, 0);
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR0_BASE:
		*rval = read_config(hyp, vcpuid, 0);
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR1_BASE:
		*rval = read_config(hyp, vcpuid, 1);
		*retu = false;
		return (0);
	default:
		break;
	}

	if (reg >= (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) &&
	    reg < (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE + VGIC_PRV_I_NUM)){
		n = (reg - GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) / 4;
		*rval = read_priorityr(hyp, vcpuid, n);
		*retu = false;
		return (0);
	}

	panic("%s: %lx", __func__, reg);
}

static int
redist_write(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t wval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_v3_redist *redist = &hyp->ctx[vcpuid].vgic_redist;
	bool *retu = arg;
	uint64_t reg;
	int n;


	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < redist->start || fault_ipa + size > redist->end) {
		return (EINVAL);
	}

	reg = fault_ipa - redist->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch (reg) {
	case GICR_CTLR:
		redist->gicr_ctlr = wval;
		*retu = false;
		return (0);
	case GICR_PROPBASER:
		wval &= ~(GICR_PROPBASER_OUTER_CACHE_MASK |
		    GICR_PROPBASER_SHARE_MASK | GICR_PROPBASER_CACHE_MASK);
		wval |=
		    (GICR_PROPBASER_SHARE_OS << GICR_PROPBASER_SHARE_SHIFT) |
		    (GICR_PROPBASER_CACHE_NIWAWB << GICR_PROPBASER_CACHE_SHIFT);
		redist->gicr_propbaser = wval;
		*retu = false;
		return (0);
	case GICR_PENDBASER:
		wval &= ~(GICR_PENDBASER_OUTER_CACHE_MASK |
		    GICR_PENDBASER_SHARE_MASK | GICR_PENDBASER_CACHE_MASK);
		wval |=
		    (GICR_PENDBASER_SHARE_OS << GICR_PENDBASER_SHARE_SHIFT) |
		    (GICR_PENDBASER_CACHE_NIWAWB << GICR_PENDBASER_CACHE_SHIFT);
		redist->gicr_pendbaser = wval;
		*retu = false;
		return (0);
	case GICR_TYPER:
	case GICR_WAKER:
	case GICR_PIDR2:
	case GICR_SGI_BASE_SIZE + GICR_IGROUPR0:
		/* Ignore writes */
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ISENABLER0:
	case GICR_SGI_BASE_SIZE + GICR_ICENABLER0:
		write_enabler(hyp, vcpuid, 0,
		    reg == (GICR_SGI_BASE_SIZE + GICR_ISENABLER0), wval);
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICACTIVER0:
		/* TODO: Implement */
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR0_BASE:
		write_config(hyp, vcpuid, 0, wval);
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR1_BASE:
		write_config(hyp, vcpuid, 1, wval);
		*retu = false;
		return (0);
	default:
		break;
	}

	if (reg >= (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) &&
	    reg < (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE + VGIC_PRV_I_NUM)){
		n = (reg - GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) / 4;
		write_priorityr(hyp, vcpuid, n, wval);
		*retu = false;
		return (0);
	}

	panic("%s: %lx", __func__, reg);
}

int
vgic_v3_icc_sgi1r_read(void *vm, int vcpuid, uint64_t *rval, void *arg)
{
	bool *retu = arg;

	/*
	 * TODO: Inject an unknown exception.
	 */
	*rval = 0;
	*retu = false;
	return (0);
}

int
vgic_v3_icc_sgi1r_write(void *vm, int vcpuid, uint64_t rval, void *arg)
{
	struct hyp *hyp;
	cpuset_t active_cpus;
	uint32_t irqid;
	int cpus, vcpu;
	bool *retu = arg;

	hyp = vm_get_cookie(vm);
	active_cpus = vm_active_cpus(vm);
	if ((rval & ICC_SGI1R_EL1_IRM) == 0) {
		/*
		 * TODO: Support on mure than 16 CPUs. This is the mask for the
		 * affinity bits. These should be 0.
		 */
		if ((rval & 0xff00ff00ff000ul) != 0)
			return (0);
		irqid = (rval >> ICC_SGI1R_EL1_SGIID_SHIFT) &
		    ICC_SGI1R_EL1_SGIID_MASK;
		cpus = rval & 0xff;
		vcpu = 0;
		while (cpus > 0) {
			if (CPU_ISSET(vcpu, &active_cpus) && vcpu != vcpuid) {
				vgic_v3_inject_irq(hyp, vcpuid, irqid, true,
				    VGIC_IRQ_MISC);
			}
			vcpu++;
			cpus >>= 1;
		}
	}

	*retu = false;

	return (0);
}

static void
vgic_v3_mmio_init(struct hyp *hyp)
{
	struct vgic_v3_dist *dist;
	struct vgic_v3_irq *irq;
	int i;

	/* Allocate memory for the SPIs */
	dist = &hyp->vgic_dist;
	dist->irqs = malloc((dist->nirqs - VGIC_PRV_I_NUM) *
	    sizeof(*dist->irqs), M_VGIC_V3, M_WAITOK | M_ZERO);

	for (i = 0; i < dist->nirqs - VGIC_PRV_I_NUM; i++) {
		irq = &dist->irqs[i];

		/* TODO: We need a call to mtx_destroy */
		mtx_init(&irq->irq_spinmtx, "VGIC IRQ spinlock", NULL,
		    MTX_SPIN);

		irq->irq = i + VGIC_PRV_I_NUM;
		irq->irqtype = VGIC_IRQ_MISC;
	}
}

static void
vgic_v3_mmio_destroy(struct hyp *hyp)
{
	struct vgic_v3_dist *dist = &hyp->vgic_dist;

	free(dist->irqs, M_VGIC_V3);
}

int
vgic_v3_attach_to_vm(struct vm *vm, uint64_t dist_start, size_t dist_size,
    uint64_t redist_start, size_t redist_size)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_v3_dist *dist = &hyp->vgic_dist;
	struct vgic_v3_redist *redist;
	int i;

	/* Set the distributor address and size for trapping guest access. */
	dist->start = dist_start;
	dist->end = dist_start + dist_size;

	for (i = 0; i < VM_MAXCPU; i++) {
		redist = &hyp->ctx[i].vgic_redist;
		/* Set the redistributor address and size. */
		redist->start = redist_start;
		redist->end = redist_start + redist_size;
	}

	vm_register_inst_handler(vm, dist_start, dist_size, dist_read,
	    dist_write);
	vm_register_inst_handler(vm, redist_start, redist_size, redist_read,
	    redist_write);

	vgic_v3_mmio_init(hyp);

	hyp->vgic_attached = true;

	return (0);
}

void
vgic_v3_detach_from_vm(struct vm *vm)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct hypctx *hypctx;
	struct vgic_v3_cpu_if *cpu_if;
	int i;

	for (i = 0; i < VM_MAXCPU; i++) {
		hypctx = & hyp->ctx[i];
		cpu_if = &hypctx->vgic_cpu_if;
	}

	vgic_v3_mmio_destroy(hyp);
}

static struct vgic_v3_irq *
vgic_v3_get_irq(struct hyp *hyp, int vcpuid, uint32_t irqid)
{
	struct vgic_v3_cpu_if *cpu_if;
	struct vgic_v3_dist *dist;
	struct vgic_v3_irq *irq;

	if (irqid < VGIC_PRV_I_NUM) {
		if (vcpuid < 0 || vcpuid >= nitems(hyp->ctx))
			return (NULL);

		cpu_if = &hyp->ctx[vcpuid].vgic_cpu_if;
		irq = &cpu_if->private_irqs[irqid];
	} else if (irqid <= GIC_LAST_SPI) {
		dist = &hyp->vgic_dist;
		irqid -= VGIC_PRV_I_NUM;
		if (irqid >= dist->nirqs)
			return (NULL);
		irq = &dist->irqs[irqid];
	} else if (irqid < GIC_FIRST_LPI) {
		return (NULL);
	} else {
		panic("TODO: %s: Support LPIs (irq = %x)", __func__, irqid);
	}

	mtx_lock_spin(&irq->irq_spinmtx);
	return (irq);
}

static void
vgic_v3_release_irq(struct vgic_v3_irq *irq)
{

	mtx_unlock_spin(&irq->irq_spinmtx);
}

int
vgic_v3_vcpu_pending_irq(void *arg)
{
	panic("vgic_v3_vcpu_pending_irq");
}

int
vgic_v3_inject_irq(struct hyp *hyp, int vcpuid, uint32_t irqid, bool level,
    enum vgic_v3_irqtype irqtype)
{

	struct vgic_v3_cpu_if *cpu_if;
	struct vgic_v3_irq *irq;
	uint64_t irouter;


	KASSERT(vcpuid == -1 || irqid < VGIC_PRV_I_NUM,
	    ("%s: SPI/LPI with vcpuid set: irq %u vcpuid %u", __func__, irqid,
	    vcpuid));

	irq = vgic_v3_get_irq(hyp, vcpuid, irqid);
	if (irq == NULL) {
		eprintf("Malformed IRQ %u.\n", irqid);
		return (1);
	}

	/* Ignore already pending IRQs */
	if (irq->pending || irq->active) {
		vgic_v3_release_irq(irq);
		return (0);
	}

	irouter = irq->mpidr;
	KASSERT(vcpuid == -1 || vcpuid == irouter,
	    ("%s: Interrupt %u has bad cpu affinity: vcpuid %u affinity %#lx",
	    __func__, irqid, vcpuid, irouter));
	KASSERT(irouter < VM_MAXCPU,
	    ("%s: Interrupt %u sent to invalid vcpu %lu", __func__, irqid,
	    irouter));

	if (vcpuid == -1)
		vcpuid = irouter;
	if (vcpuid >= VM_MAXCPU) {
		vgic_v3_release_irq(irq);
		return (1);
	}

	cpu_if = &hyp->ctx[vcpuid].vgic_cpu_if;

	mtx_lock_spin(&cpu_if->lr_mtx);

	/*
	 * TODO: Only inject if:
	 *  - Level-triggered IRQ: level changes low -> high
	 *  -  Edge-triggered IRQ: level is high
	 */

	if (level) {
		MPASS(!irq->active);
		/* TODO: Insert in the correct place */
		TAILQ_INSERT_TAIL(&cpu_if->irq_act_pend, irq, act_pend_list);
		irq->pending = 1;
	}

	mtx_unlock_spin(&cpu_if->lr_mtx);
	vgic_v3_release_irq(irq);

	return (0);
}

int
vgic_v3_inject_msi(struct hyp *hyp, uint64_t msg, uint64_t addr)
{
	struct vgic_v3_dist *dist = &hyp->vgic_dist;
	uint64_t reg;

	/* This is a 4 byte register */
	if (addr < dist->start || addr + 4 > dist->end) {
		return (EINVAL);
	}

	reg = addr - dist->start;
	if (reg != GICD_SETSPI_NSR)
		return (EINVAL);

	return (vgic_v3_inject_irq(hyp, -1, msg, true, VGIC_IRQ_MISC));
}

void
vgic_v3_flush_hwstate(void *arg)
{
	struct hypctx *hypctx;
	struct vgic_v3_cpu_if *cpu_if;
	struct vgic_v3_irq *irq;
	int i;

	hypctx = arg;
	cpu_if = &hypctx->vgic_cpu_if;

	/*
	 * All Distributor writes have been executed at this point, do not
	 * protect Distributor reads with a mutex.
	 *
	 * This is callled with all interrupts disabled, so there is no need for
	 * a List Register spinlock either.
	 */
	mtx_lock_spin(&cpu_if->lr_mtx);

	cpu_if->ich_hcr_el2 &= ~ICH_HCR_EL2_UIE;

	/* Exit early if there are no buffered interrupts */
	if (TAILQ_EMPTY(&cpu_if->irq_act_pend))
		goto out;

	KASSERT(cpu_if->ich_lr_used == 0, ("%s: Used LR count not zero %u",
	    __func__, cpu_if->ich_lr_used));

	i = 0;
	cpu_if->ich_elrsr_el2 = (1 << cpu_if->ich_lr_num) - 1;
	TAILQ_FOREACH(irq, &cpu_if->irq_act_pend, act_pend_list) {
		/* No free list register, stop searching for IRQs */
		if (i == cpu_if->ich_lr_num)
			break;

		if (!irq->enabled)
			continue;

		cpu_if->ich_lr_el2[i] = ICH_LR_EL2_GROUP1 |
		    ((uint64_t)irq->priority << ICH_LR_EL2_PRIO_SHIFT) |
		    irq->irq;
		if (irq->pending)
			cpu_if->ich_lr_el2[i] |= ICH_LR_EL2_STATE_PENDING;
		if (irq->active)
			cpu_if->ich_lr_el2[i] |= ICH_LR_EL2_STATE_ACTIVE;

		i++;
	}
	cpu_if->ich_lr_used = i;

out:
	mtx_unlock_spin(&cpu_if->lr_mtx);
}

void
vgic_v3_sync_hwstate(void *arg)
{
	struct hypctx *hypctx;
	struct vgic_v3_cpu_if *cpu_if;
	struct vgic_v3_irq *irq;
	uint64_t lr;
	int i;

	hypctx = arg;
	cpu_if = &hypctx->vgic_cpu_if;

	/* Exit early if there are no buffered interrupts */
	if (cpu_if->ich_lr_used == 0)
		return;

	/*
	 * Check on the IRQ state after running the guest. ich_lr_used and
	 * ich_lr_el2 are only ever used within this thread so is safe to
	 * access unlocked.
	 */
	for (i = 0; i < cpu_if->ich_lr_used; i++) {
		lr = cpu_if->ich_lr_el2[i];
		cpu_if->ich_lr_el2[i] = 0;

		irq = vgic_v3_get_irq(hypctx->hyp, hypctx->vcpu,
		    ICH_LR_EL2_VINTID(lr));
		if (irq == NULL)
			continue;

		irq->active = (lr & ICH_LR_EL2_STATE_ACTIVE) != 0;
		irq->pending = !irq->active &&
		    (lr & ICH_LR_EL2_STATE_PENDING) != 0;

		/* Lock to update irq_act_pend */
		mtx_lock_spin(&cpu_if->lr_mtx);
		if (irq->active) {
			MPASS(!irq->pending);
			/* Ensure the active IRQ is at the head of the list */
			TAILQ_REMOVE(&cpu_if->irq_act_pend, irq, act_pend_list);
			TAILQ_INSERT_HEAD(&cpu_if->irq_act_pend, irq,
			    act_pend_list);
		} else if (!irq->pending) {
			/* If pending or active remove from the list */
			TAILQ_REMOVE(&cpu_if->irq_act_pend, irq, act_pend_list);
		}
		mtx_unlock_spin(&cpu_if->lr_mtx);
		vgic_v3_release_irq(irq);
	}

	cpu_if->ich_hcr_el2 &= ~ICH_HCR_EL2_EOICOUNT_MASK;
	cpu_if->ich_lr_used = 0;
}

static void
vgic_v3_get_ro_regs()
{
	/* GICD_ICFGR0 configures SGIs and it is read-only. */
	ro_regs.gicd_icfgr0 = gic_d_read(gic_sc, 4, GICD_ICFGR(0));

	/*
	 * Configure the GIC type register for the guest.
	 * All SPIs and max LPI of 64k - 1.
	 */
	ro_regs.gicd_typer = 31;
	ro_regs.gicd_typer |= 16ul << 19;
	ro_regs.gicd_typer |= GICD_TYPER_MBIS;

	/*
	 * XXX. Guest reads of GICD_PIDR2 should return the same ArchRev as
	 * specified in the guest FDT.
	 */
	ro_regs.gicd_pidr2 = gic_d_read(gic_sc, 4, GICD_PIDR2);
}

bool
vgic_attach(void)
{
	device_t gic_dev;
	uintptr_t ver;

	KASSERT(gic_sc == NULL, ("%s: GIC softc is not NULL", __func__));

	gic_dev = device_lookup_by_name("gic0");
	if (gic_dev == NULL)
		return (false);

	if (BUS_READ_IVAR(gic_dev, NULL, GIC_IVAR_HW_REV, &ver) != 0)
		return (false);
	if (ver < 3)
		return (false);

	gic_sc = device_get_softc(gic_dev);

	return (true);
}

void
vgic_v3_init(uint64_t ich_vtr_el2)
{
	uint32_t pribits, prebits;

	KASSERT(gic_sc != NULL, ("GIC softc is NULL"));

	vgic_v3_get_ro_regs();

	pribits = ICH_VTR_EL2_PRIBITS(ich_vtr_el2);
	switch (pribits) {
	case 5:
		virt_features.min_prio = 0xf8;
	case 6:
		virt_features.min_prio = 0xfc;
	case 7:
		virt_features.min_prio = 0xfe;
	case 8:
		virt_features.min_prio = 0xff;
	}

	prebits = ICH_VTR_EL2_PREBITS(ich_vtr_el2);
	switch (prebits) {
	case 5:
		virt_features.ich_ap0r_num = 1;
		virt_features.ich_ap1r_num = 1;
	case 6:
		virt_features.ich_ap0r_num = 2;
		virt_features.ich_ap1r_num = 2;
	case 7:
		virt_features.ich_ap0r_num = 4;
		virt_features.ich_ap1r_num = 4;
	}

	virt_features.ich_lr_num = ICH_VTR_EL2_LISTREGS(ich_vtr_el2);
}
