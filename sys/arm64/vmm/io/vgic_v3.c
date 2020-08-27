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

struct vgic_v3_irq {
	uint32_t irq;
	enum vgic_v3_irqtype irqtype;
	uint8_t enabled;
	uint8_t priority;
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

void
vgic_v3_cpuinit(void *arg, bool last_vcpu)
{
	struct hypctx *hypctx = arg;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;
	struct vgic_v3_redist *redist = &hypctx->vgic_redist;
	uint64_t aff, vmpidr_el2;
	int i;

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

	mtx_init(&cpu_if->lr_mtx, "VGICv3 ICH_LR_EL2 lock", NULL, MTX_SPIN);

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

	cpu_if->ich_ap0r_num = virt_features.ich_ap0r_num;
	cpu_if->ich_ap1r_num = virt_features.ich_ap1r_num;

	cpu_if->irqbuf = malloc(IRQBUF_SIZE_MIN * sizeof(*cpu_if->irqbuf),
	    M_VGIC_V3, M_WAITOK | M_ZERO);
	cpu_if->irqbuf_size = IRQBUF_SIZE_MIN;
	cpu_if->irqbuf_num = 0;
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
	dist->gicd_ctlr = 0; //GICD_CTLR_DS;

	dist->gicd_typer = ro_regs.gicd_typer;
	dist->nirqs = GICD_TYPER_I_NUM(dist->gicd_typer);
	dist->gicd_pidr2 = ro_regs.gicd_pidr2;

	mtx_init(&dist->dist_mtx, "VGICv3 Distributor lock", NULL, MTX_SPIN);
}

static uint64_t
dist_read_gicd_ixenabler(struct vgic_v3_dist *dist, int n)
{
	uint64_t ret;

	/*
	 * GIC Architecture specification, p 8-471: "When ARE is 1 for the
	 * Security state of an interrupt, the field for that interrupt is RES0
	 * and an implementation is permitted to* make the field RAZ/WI in this
	 * case".
	 */
	if (n == 0 && aff_routing_en(dist)) {
		return (0);
	}

	/* Ensure the register is valid */
	if ((n * sizeof(*dist->gicd_ixenabler)) > dist->gicd_ixenable_size)
		return (0);

	mtx_lock_spin(&dist->dist_mtx);
	ret = dist->gicd_ixenabler[n];
	mtx_unlock_spin(&dist->dist_mtx);

	return (ret);
}

static void
vgic_update_interrupts(struct hyp *hyp, int vcpuid, int irq,
    uint64_t old_ixenabler, uint64_t new_ixenabler, bool set)
{
	uint32_t updates;
	int i;

	i = 0;
	updates = old_ixenabler ^ new_ixenabler;
	for (; updates != 0; updates >>= 1) {
		if ((updates & 1) == 1) {
			vgic_v3_irq_toggle_enabled(irq + i, set, hyp, vcpuid);
		}
		i++;
	}
}

static void
dist_write_gicd_ixenabler(struct hyp *hyp, int vcpuid,
    struct vgic_v3_dist *dist, int idx, uint64_t val, bool set)
{
	uint32_t old_ixenabler;

	/* TODO: Handle LPIs */
	if ((idx * sizeof(*dist->gicd_ixenabler)) > dist->gicd_ixenable_size)
		panic("Invalid reg: %x %zx\n", idx, dist->gicd_ixenable_size);

	/*
	 * GIC Architecture specification, p 8-471: "When ARE is 1 for the
	 * Security state of an interrupt, the field for that interrupt is RES0
	 * and an implementation is permitted to* make the field RAZ/WI in this
	 * case".
	 */
	if (idx == 0 && aff_routing_en(dist)) {
		return;
	}

	mtx_lock_spin(&dist->dist_mtx);
	old_ixenabler = dist->gicd_ixenabler[idx];
	if (set)
		dist->gicd_ixenabler[idx] |= val;
	else
		dist->gicd_ixenabler[idx] &= ~val;
	vgic_update_interrupts(hyp, vcpuid, idx * 32, old_ixenabler,
	    dist->gicd_ixenabler[idx], set);
	mtx_unlock_spin(&dist->dist_mtx);
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
	    reg < (GICD_IGROUPR(0) + dist->gicd_igroup_size)) {

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

	if (reg >= GICD_ISENABLER(0) && /* 0x0100 */
	    reg < GICD_ISENABLER(0) + dist->gicd_ixenable_size) {
		n = (reg - GICD_ISENABLER(0)) / 4;
		*rval = dist_read_gicd_ixenabler(dist, n);
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICENABLER(0) && /* 0x0180 */
	    reg < GICD_ICENABLER(0) + dist->gicd_ixenable_size) {
		n = (reg - GICD_ICENABLER(0)) / 4;
		*rval = dist_read_gicd_ixenabler(dist, n);
		*retu = false;
		return (0);
	}

	/* TODO: GICD_ISPENDR 0x0200 */
	/* TODO: GICD_ICPENDR 0x0280 */
	/* TODO: GICD_ICACTIVER 0x0380 */

	if (reg >= GICD_IPRIORITYR(0) && /* 0x0400 */
	    reg < GICD_IPRIORITYR(0) + dist->gicd_ipriority_size) {
		n = (reg - GICD_IPRIORITYR(0)) / 4;
		/*
		 * GIC Architecture specification, p 8-483: when affinity
		 * routing is enabled, GICD_IPRIORITYR<n> is RAZ/WI for
		 * n = 0 to 7.
		 */
		if (aff_routing_en(dist) && n <= 7) {
			*rval = 0;
		} else {
			mtx_lock_spin(&dist->dist_mtx);
			*rval = dist->gicd_ipriorityr[n];
			mtx_unlock_spin(&dist->dist_mtx);
		}

		*retu = false;
		return (0);
	}

	/* TODO: GICD_ITARGETSR 0x0800 */

	if (reg >= GICD_ICFGR(0) && /* 0x0C00 */
	    reg < (GICD_ICFGR(0) + dist->gicd_icfg_size)) {
		if (reg == GICD_ICFGR(0)) {
			*rval = 0;
		} else {
			mtx_lock_spin(&dist->dist_mtx);
			*rval = dist->gicd_icfgr[(reg - GICD_ICFGR(0)) / 4];
			mtx_unlock_spin(&dist->dist_mtx);
		}
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IROUTER(0) && /* 0x6000 */
	    reg < GICD_IROUTER(0) + dist->gicd_iroute_size) {
		n = (reg - GICD_IROUTER(0)) / 8;
		if (n <= 31 || !aff_routing_en(dist)) {
			*rval = 0;
		} else {
			mtx_lock_spin(&dist->dist_mtx);
			*rval = dist->gicd_irouter[n];
			mtx_unlock_spin(&dist->dist_mtx);
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
	if (fault_ipa < dist->start || fault_ipa + size > dist->end ||
	    size != 4) {
		/* TODO: GICD_IROUTER is an 8 byte register */
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
		//wval |= GICD_CTLR_DS;
		mtx_lock_spin(&dist->dist_mtx);
#define	reg_changed(new, old, mask)	(((new) & (mask)) != ((old) & (mask)))
		if (reg_changed(wval, dist->gicd_ctlr, GICD_CTLR_G1A)) {
			if (!(wval & GICD_CTLR_G1A))
				vgic_v3_group_toggle_enabled(false, hyp);
			else
				vgic_v3_group_toggle_enabled(true, hyp);
		}
		dist->gicd_ctlr = wval;
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
	    reg < (GICD_IGROUPR(0) + dist->gicd_igroup_size)) {
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ISENABLER(0) && /* 0x0100 */
	    reg < GICD_ISENABLER(0) + dist->gicd_ixenable_size) {
		n = (reg - GICD_ISENABLER(0)) / 4;
		dist_write_gicd_ixenabler(hyp, vcpuid, dist, n, wval, true);
		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICENABLER(0) && /* 0x0180 */
	    reg < GICD_ICENABLER(0) + dist->gicd_ixenable_size) {
		n = (reg - GICD_ICENABLER(0)) / 4;
		dist_write_gicd_ixenabler(hyp, vcpuid, dist, n, wval, false);
		*retu = false;
		return (0);
	}

	/* TODO: GICD_ISPENDR 0x0200 */
	/* TODO: GICD_ICPENDR 0x0280 */
	/* TODO: GICD_ICACTIVER 0x0380 */

	if (reg >= GICD_IPRIORITYR(0) && /* 0x0400 */
	    reg < GICD_IPRIORITYR(0) + dist->gicd_ipriority_size) {
		n = (reg - GICD_IPRIORITYR(0)) / 4;
		/*
		 * GIC Architecture specification, p 8-483: when affinity
		 * routing is enabled, GICD_IPRIORITYR<n> is RAZ/WI for
		 * n = 0 to 7.
		 */
		if (!(aff_routing_en(dist) && n <= 7)) {
			mtx_lock_spin(&dist->dist_mtx);
			dist->gicd_ipriorityr[n] = wval;
			mtx_unlock_spin(&dist->dist_mtx);
		}

		*retu = false;
		return (0);
	}

	if (reg >= GICD_ICFGR(0) &&
	    reg < (GICD_ICFGR(0) + dist->gicd_icfg_size)) {
		if (reg != GICD_ICFGR(0)) {
			mtx_lock_spin(&dist->dist_mtx);
			dist->gicd_icfgr[(reg - GICD_ICFGR(0)) / 4] = wval;
			mtx_unlock_spin(&dist->dist_mtx);
		}
		*retu = false;
		return (0);
	}

	if (reg >= GICD_IROUTER(0) && /* 0x6000 */
	    reg < GICD_IROUTER(0) + dist->gicd_iroute_size) {
		n = (reg - GICD_IROUTER(0)) / 8;
		if (n > 31 && aff_routing_en(dist)) {
			mtx_lock_spin(&dist->dist_mtx);
			dist->gicd_irouter[n] = wval;
			mtx_unlock_spin(&dist->dist_mtx);
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
		*rval = 0; // & ~GICR_WAKER_PS & ~GICR_WAKER_CA;
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
		*rval = redist->gicr_ixenabler0;
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR0_BASE:
		*rval = redist->gicr_icfgr0;
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR1_BASE:
		*rval = redist->gicr_icfgr1;
		*retu = false;
		return (0);
	default:
		break;
	}

	if (reg >= (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) &&
	    reg < (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE +
	    sizeof(redist->gicr_ipriorityr))) {
		n = (reg - GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) / 4;
		*rval = redist->gicr_ipriorityr[n];
		*retu = false;
		return (0);
	}

	panic("%s: %lx", __func__, reg);
}

static void
redist_write_gicr_ixenabler0(struct hyp *hyp, int vcpuid,
    struct vgic_v3_redist *redist, uint64_t wval, bool set)
{
	uint32_t old_ixenabler0, new_ixenabler0;

	old_ixenabler0 = redist->gicr_ixenabler0;
	if (set)
		new_ixenabler0 = old_ixenabler0 | wval;
	else
		new_ixenabler0 = old_ixenabler0 & ~wval;
	vgic_update_interrupts(hyp, vcpuid, 0, old_ixenabler0, new_ixenabler0,
	    set);
	redist->gicr_ixenabler0 = new_ixenabler0;
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
		redist_write_gicr_ixenabler0(hyp, vcpuid, redist, wval,
		    reg == (GICR_SGI_BASE_SIZE + GICR_ISENABLER0));
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE:
		break;
	case GICR_SGI_BASE_SIZE + GICR_ICFGR0_BASE:
		redist->gicr_icfgr0 = wval;
		*retu = false;
		return (0);
	case GICR_SGI_BASE_SIZE + GICR_ICFGR1_BASE:
		redist->gicr_icfgr1 = wval;
		*retu = false;
		return (0);
	default:
		break;
	}

	if (reg >= (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) &&
	    reg < (GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE +
	    sizeof(redist->gicr_ipriorityr))) {
		n = (reg - GICR_SGI_BASE_SIZE + GICR_IPRIORITYR_BASE) / 4;
		redist->gicr_ipriorityr[n] = wval;
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
	int cpus, irq, vcpu;
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
		irq = (rval >> ICC_SGI1R_EL1_SGIID_SHIFT) &
		    ICC_SGI1R_EL1_SGIID_MASK;
		cpus = rval & 0xff;
		vcpu = 0;
		while (cpus > 0) {
			if (CPU_ISSET(vcpu, &active_cpus) && vcpu != vcpuid) {
				vgic_v3_inject_irq(&hyp->ctx[vcpu], irq,
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
	struct vgic_v3_dist *dist = &hyp->vgic_dist;

	/* GICD_IGROUPR */
	dist->gicd_igroup_size = howmany(dist->nirqs, 32) *
	    sizeof(uint32_t);

	/* GICD_ISENABLER & GICD_ICENABLER */
	dist->gicd_ixenable_size =
	    ((dist->gicd_typer & GICD_TYPER_ITLINESNUM_MASK) + 1) *
	    sizeof(*dist->gicd_ixenabler);
	dist->gicd_ixenabler = malloc(dist->gicd_ixenable_size, M_VGIC_V3,
	    M_WAITOK | M_ZERO);

	/* GICD_IPRIORITYR */
	dist->gicd_ipriority_size =
	    (8 * ((dist->gicd_typer & GICD_TYPER_ITLINESNUM_MASK) + 1)) *
	    sizeof(*dist->gicd_ipriorityr);
	dist->gicd_ipriorityr = malloc(dist->gicd_ipriority_size,
	    M_VGIC_V3, M_WAITOK | M_ZERO);

	/* GICD_ICFGR */
	dist->gicd_icfg_size = howmany(dist->nirqs, 16) *
	    sizeof(*dist->gicd_icfgr);
	dist->gicd_icfgr = malloc(dist->gicd_icfg_size,
	    M_VGIC_V3, M_WAITOK | M_ZERO);

	/* GICD_IROUTER */
	dist->gicd_iroute_size =
	    (32 * ((dist->gicd_typer & GICD_TYPER_ITLINESNUM_MASK) + 1)) *
	    sizeof(*dist->gicd_irouter);
	dist->gicd_irouter = malloc(dist->gicd_iroute_size, M_VGIC_V3,
	    M_WAITOK | M_ZERO);
}

static void
vgic_v3_mmio_destroy(struct hyp *hyp)
{
	struct vgic_v3_dist *dist = &hyp->vgic_dist;

	free(dist->gicd_ixenabler, M_VGIC_V3);
	free(dist->gicd_ipriorityr, M_VGIC_V3);
	free(dist->gicd_icfgr, M_VGIC_V3);
	free(dist->gicd_irouter, M_VGIC_V3);
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
		free(cpu_if->irqbuf, M_VGIC_V3);
	}

	vgic_v3_mmio_destroy(hyp);
}

int
vgic_v3_vcpu_pending_irq(void *arg)
{
	struct hypctx *hypctx = arg;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;

	return (cpu_if->irqbuf_num);
}

/* Removes ALL instances of interrupt 'irq' */
static int
vgic_v3_irqbuf_remove_nolock(uint32_t irq, struct vgic_v3_cpu_if *cpu_if)
{
	size_t dest = 0;
	size_t from = cpu_if->irqbuf_num;

	while (dest < cpu_if->irqbuf_num) {
		if (cpu_if->irqbuf[dest].irq == irq) {
			for (from = dest + 1; from < cpu_if->irqbuf_num; from++) {
				if (cpu_if->irqbuf[from].irq == irq)
					continue;
				cpu_if->irqbuf[dest++] = cpu_if->irqbuf[from];
			}
			cpu_if->irqbuf_num = dest;
		} else {
			dest++;
		}
	}

	return (from - dest);
}

int
vgic_v3_remove_irq(void *arg, uint32_t irq, bool ignore_state)
{
        struct hypctx *hypctx = arg;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;
	struct vgic_v3_dist *dist = &hypctx->hyp->vgic_dist;
	size_t i;

	if (irq >= dist->nirqs) {
		eprintf("Malformed IRQ %u.\n", irq);
		return (1);
	}

	mtx_lock_spin(&cpu_if->lr_mtx);

	for (i = 0; i < cpu_if->ich_lr_num; i++) {
		if (ICH_LR_EL2_VINTID(cpu_if->ich_lr_el2[i]) == irq &&
		    (lr_not_active(cpu_if->ich_lr_el2[i]) || ignore_state))
			lr_clear_irq(cpu_if->ich_lr_el2[i]);
	}
	vgic_v3_irqbuf_remove_nolock(irq, cpu_if);

	mtx_unlock_spin(&cpu_if->lr_mtx);

	return (0);
}

static struct vgic_v3_irq *
vgic_v3_irqbuf_add_nolock(struct vgic_v3_cpu_if *cpu_if)
{
	struct vgic_v3_irq *new_irqbuf, *old_irqbuf;
	size_t new_size;

	if (cpu_if->irqbuf_num == cpu_if->irqbuf_size) {
		/* Double the size of the buffered interrupts list */
		new_size = cpu_if->irqbuf_size << 1;
		if (new_size > IRQBUF_SIZE_MAX)
			return (NULL);

		new_irqbuf = NULL;
		/* TODO: malloc sleeps here and causes a panic */
		while (new_irqbuf == NULL)
			new_irqbuf = malloc(new_size * sizeof(*cpu_if->irqbuf),
			    M_VGIC_V3, M_NOWAIT | M_ZERO);
		memcpy(new_irqbuf, cpu_if->irqbuf,
		    cpu_if->irqbuf_size * sizeof(*cpu_if->irqbuf));

		old_irqbuf = cpu_if->irqbuf;
		cpu_if->irqbuf = new_irqbuf;
		cpu_if->irqbuf_size = new_size;
		free(old_irqbuf, M_VGIC_V3);
	}

	cpu_if->irqbuf_num++;

	return (&cpu_if->irqbuf[cpu_if->irqbuf_num - 1]);
}

static bool
vgic_v3_int_target(uint32_t irq, struct hypctx *hypctx)
{
	struct vgic_v3_dist *dist = &hypctx->hyp->vgic_dist;
	struct vgic_v3_redist *redist = &hypctx->vgic_redist;
	uint64_t irouter;
	uint64_t aff;
	uint32_t irq_off, irq_mask;
	int n;

	if (irq <= GIC_LAST_PPI)
		return (true);

	/* XXX Affinity routing disabled not implemented */
	if (!aff_routing_en(dist))
		return (true);

	irq_off = irq % 32;
	irq_mask = 1 << irq_off;
	n = irq / 32;

	irouter = dist->gicd_irouter[irq];
	/* Check if 1-of-N routing is active */
	if (irouter & GICD_IROUTER_IRM)
		/* Check if the VCPU is participating */
		return (redist->gicr_ctlr & GICR_CTLR_DPG1NS ? true : false);

	aff = redist->gicr_typer >> GICR_TYPER_AFF_SHIFT;
	/* Affinity in format for comparison with irouter */
	aff = GICR_TYPER_AFF(redist->gicr_typer);
	if ((irouter & aff) == aff)
		return (true);
	else
		return (false);
}

static uint8_t
vgic_v3_get_priority(uint32_t irq, struct hypctx *hypctx)
{
	struct vgic_v3_dist *dist = &hypctx->hyp->vgic_dist;
	struct vgic_v3_redist *redist = &hypctx->vgic_redist;
	size_t n;
	uint32_t off, mask;
	uint8_t priority;

	n = irq / 4;
	off = n % 4;
	mask = 0xff << off;
	/*
	 * When affinity routing is enabled, the Redistributor is used for
	 * SGIs and PPIs and the Distributor for SPIs. When affinity routing
	 * is not enabled, the Distributor registers are used for all
	 * interrupts.
	 */
	if (aff_routing_en(dist) && (n <= 7))
		priority = (redist->gicr_ipriorityr[n] & mask) >> off;
	else
		priority = (dist->gicd_ipriorityr[n] & mask) >> off;

	return (priority);
}

static bool
vgic_v3_intid_enabled(uint32_t irq, struct hypctx *hypctx)
{
	struct vgic_v3_dist *dist;
	struct vgic_v3_redist *redist;
	uint32_t irq_off, irq_mask;
	int n;

	irq_off = irq % 32;
	irq_mask = 1 << irq_off;
	n = irq / 32;

	if (irq <= GIC_LAST_PPI) {
		redist = &hypctx->vgic_redist;
		if (!(redist->gicr_ixenabler0 & irq_mask))
			return (false);
	} else {
		dist = &hypctx->hyp->vgic_dist;
		if (!(dist->gicd_ixenabler[n] & irq_mask))
			return (false);
	}

	return (true);
}

static inline bool
dist_group_enabled(struct vgic_v3_dist *dist)
{
	return ((dist->gicd_ctlr & GICD_CTLR_G1A) != 0);
}

int
vgic_v3_inject_irq(void *arg, uint32_t irq, enum vgic_v3_irqtype irqtype)
{
        struct hypctx *hypctx = arg;
	struct vgic_v3_dist *dist = &hypctx->hyp->vgic_dist;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;
	struct vgic_v3_irq *vip;
	int error;
	int i;
	uint8_t priority;
	bool enabled;


	if (irq >= dist->nirqs || irqtype >= VGIC_IRQ_INVALID) {
		eprintf("Malformed IRQ %u.\n", irq);
		return (1);
	}

	error = 0;
	mtx_lock_spin(&dist->dist_mtx);

	enabled = dist_group_enabled(&hypctx->hyp->vgic_dist) &&
	    vgic_v3_intid_enabled(irq, hypctx) &&
	    vgic_v3_int_target(irq, hypctx);
	priority = vgic_v3_get_priority(irq, hypctx);

	mtx_lock_spin(&cpu_if->lr_mtx);

	/*
	 * If the guest is running behind timer interrupts, don't swamp it with
	 * one interrupt after another. However, if the timer interrupt is being
	 * serviced by the guest (it is in a state other than pending, either
	 * active or pending and active), then add it to the buffer to be
	 * injected later. Otherwise, the timer would stop working because we
	 * disable the timer in the host interrupt handler.
	 */
	if (irqtype == VGIC_IRQ_CLK) {
		for (i = 0; i < cpu_if->ich_lr_num; i++)
			if (ICH_LR_EL2_VINTID(cpu_if->ich_lr_el2[i]) == irq &&
			    lr_pending(cpu_if->ich_lr_el2[i]))
				goto out;
		for (i = 0; i < cpu_if->irqbuf_num; i++)
			if (cpu_if->irqbuf[i].irq == irq)
				goto out;
	}

	vip = vgic_v3_irqbuf_add_nolock(cpu_if);
	if (!vip) {
		eprintf("Error adding IRQ %u to the IRQ buffer.\n", irq);
		error = 1;
		goto out;
	}
	vip->irq = irq;
	vip->irqtype = irqtype;
	vip->enabled = enabled;
	vip->priority = priority;

out:
	mtx_unlock_spin(&cpu_if->lr_mtx);
	mtx_unlock_spin(&dist->dist_mtx);

	return (error);
}

int
vgic_v3_inject_lpi(void *arg, uint32_t lpi)
{
        struct hypctx *hypctx = arg;
	struct vgic_v3_dist *dist = &hypctx->hyp->vgic_dist;
	struct vgic_v3_cpu_if *cpu_if = &hypctx->vgic_cpu_if;
	struct vgic_v3_irq *vip;
	int error;

	KASSERT(lpi >= GIC_FIRST_LPI, ("%s: Invalid LPI %u", __func__, lpi));

	error = 0;

	mtx_lock_spin(&dist->dist_mtx);
	mtx_lock_spin(&cpu_if->lr_mtx);
	vip = vgic_v3_irqbuf_add_nolock(cpu_if);
	if (!vip) {
		eprintf("Error adding LPI %u to the IRQ buffer.\n", lpi);
		error = ENXIO;
		goto out;
	}
	vip->irq = lpi;
	vip->irqtype = VGIC_IRQ_MISC;
	vip->enabled = 1;
	vip->priority = 1;

out:
	mtx_unlock_spin(&cpu_if->lr_mtx);
	mtx_unlock_spin(&dist->dist_mtx);

	return (error);
}

void
vgic_v3_group_toggle_enabled(bool enabled, struct hyp *hyp)
{
	struct hypctx *hypctx;
	struct vgic_v3_cpu_if *cpu_if;
	struct vgic_v3_irq *vip;
	int i, j;

	for (i = 0; i < VM_MAXCPU; i++) {
		hypctx = &hyp->ctx[i];
		cpu_if = &hypctx->vgic_cpu_if;

		mtx_lock_spin(&cpu_if->lr_mtx);

		for (j = 0; j < cpu_if->irqbuf_num; j++) {
			vip = &cpu_if->irqbuf[j];
			if (!enabled)
				vip->enabled = 0;
			else if (vgic_v3_intid_enabled(vip->irq, hypctx))
				vip->enabled = 1;
		}

		mtx_unlock_spin(&cpu_if->lr_mtx);
	}
}

static int
vgic_v3_irq_toggle_enabled_vcpu(uint32_t irq, bool enabled,
    struct vgic_v3_cpu_if *cpu_if)
{
	int i;

	mtx_lock_spin(&cpu_if->lr_mtx);

	if (enabled) {
		/*
		 * Enable IRQs that were injected when the interrupt ID was
		 * disabled
		 */
		for (i = 0; i < cpu_if->irqbuf_num; i++)
			if (cpu_if->irqbuf[i].irq == irq)
				cpu_if->irqbuf[i].enabled = true;
	} else {
		/* Remove the disabled IRQ from the LR regs if it is pending */
		for (i = 0; i < cpu_if->ich_lr_num; i++)
			if (lr_pending(cpu_if->ich_lr_el2[i]) &&
			    ICH_LR_EL2_VINTID(cpu_if->ich_lr_el2[i]) == irq)
				lr_clear_irq(cpu_if->ich_lr_el2[i]);

		/* Remove the IRQ from the interrupt buffer */
		vgic_v3_irqbuf_remove_nolock(irq, cpu_if);
	}

	mtx_unlock_spin(&cpu_if->lr_mtx);

	return (0);
}

int
vgic_v3_irq_toggle_enabled(uint32_t irq, bool enabled,
    struct hyp *hyp, int vcpuid)
{
	struct vgic_v3_cpu_if *cpu_if;
	int error;
	int i;

	if (irq <= GIC_LAST_PPI) {
		cpu_if = &hyp->ctx[vcpuid].vgic_cpu_if;
		return (vgic_v3_irq_toggle_enabled_vcpu(irq, enabled, cpu_if));
	} else {
		/* TODO: Update irqbuf for all VCPUs, not just VCPU 0 */
		for (i = 0; i < 1; i++) {
			cpu_if = &hyp->ctx[i].vgic_cpu_if;
			error = vgic_v3_irq_toggle_enabled_vcpu(irq, enabled, cpu_if);
			if (error)
				return (error);
		}
	}

	return (0);
}

static int
irqbuf_highest_priority(struct vgic_v3_cpu_if *cpu_if, int start, int end,
    struct hypctx *hypctx)
{
	uint32_t irq;
	int i, max_idx;
	uint8_t priority, max_priority;
	uint8_t vpmr;

	vpmr = (cpu_if->ich_vmcr_el2 & ICH_VMCR_EL2_VPMR_MASK) >> \
	    ICH_VMCR_EL2_VPMR_SHIFT;

	max_idx = -1;
	max_priority = 0xff;
	for (i = start; i < end; i++) {
		irq = cpu_if->irqbuf[i].irq;
		/* Check that the interrupt hasn't been already scheduled */
		if (irq == IRQ_SCHEDULED)
			continue;

		if (!dist_group_enabled(&hypctx->hyp->vgic_dist))
			continue;
		if (irq < GIC_FIRST_LPI && !vgic_v3_int_target(irq, hypctx))
			continue;

		priority = cpu_if->irqbuf[i].priority;
		if (priority >= vpmr)
			continue;

		if (max_idx == -1) {
			max_idx = i;
			max_priority = priority;
		} else if (priority > max_priority) {
			max_idx = i;
			max_priority = priority;
		} else if (priority == max_priority &&
		    cpu_if->irqbuf[i].irqtype < cpu_if->irqbuf[max_idx].irqtype) {
			max_idx = i;
			max_priority = priority;
		}
	}

	return (max_idx);
}

static inline bool
cpu_if_group_enabled(struct vgic_v3_cpu_if *cpu_if)
{
	return ((cpu_if->ich_vmcr_el2 & ICH_VMCR_EL2_VENG1) != 0);
}

static inline int
irqbuf_next_enabled(struct vgic_v3_irq *irqbuf, int start, int end,
    struct hypctx *hypctx, struct vgic_v3_cpu_if *cpu_if)
{
	int i;

	if (!cpu_if_group_enabled(cpu_if))
		return (-1);

	for (i = start; i < end; i++)
		if (irqbuf[i].enabled)
			break;

	if (i < end)
		return (i);
	else
		return (-1);
}

static inline int
vgic_v3_lr_next_empty(uint32_t ich_elrsr_el2, int start, int end)
{
	int i;

	for (i = start; i < end; i++)
		if (ich_elrsr_el2 & (1U << i))
			break;

	if (i < end)
		return (i);
	else
		return (-1);
}

/*
 * There are two cases in which the virtual timer interrupt is in the list
 * registers:
 *
 * 1. The virtual interrupt is active. The guest is executing the interrupt
 * handler, and the timer fired after it programmed the new alarm time but
 * before the guest had the chance to write to the EOIR1 register.
 *
 * 2. The virtual interrupt is pending and active. The timer interrupt is level
 * sensitive. The guest wrote to the EOR1 register, but the write hasn't yet
 * propagated to the timer.
 *
 * Injecting the interrupt in these cases would mean that another timer
 * interrupt is asserted as soon as the guest writes to the EOIR1 register (or
 * very shortly thereafter, in the pending and active scenario). This can lead
 * to the guest servicing timer interrupts one after the other and doing
 * nothing else. So do not inject a timer interrupt while one is active pending.
 * The buffered timer interrupts will be injected after the next world switch in
 * this case.
 */
static bool
clk_irq_in_lr(struct vgic_v3_cpu_if *cpu_if)
{
	uint64_t lr;
	int i;

	for (i = 0; i < cpu_if->ich_lr_num; i++) {
		lr = cpu_if->ich_lr_el2[i];
		if (ICH_LR_EL2_VINTID(lr) == GT_VIRT_IRQ &&
		    (lr_active(lr) || lr_pending_active(lr)))
			return (true);
	}

	return (false);
}

static void
vgic_v3_irqbuf_to_lr(struct hypctx *hypctx, struct vgic_v3_cpu_if *cpu_if,
    bool by_priority)
{
	struct vgic_v3_irq *vip;
	int irqbuf_idx;
	int lr_idx;
	bool clk_present;

	clk_present = clk_irq_in_lr(cpu_if);

	irqbuf_idx = 0;
	lr_idx = 0;
	for (;;) {
		if (by_priority)
			irqbuf_idx = irqbuf_highest_priority(cpu_if,
			    irqbuf_idx, cpu_if->irqbuf_num, hypctx);
		else
			irqbuf_idx = irqbuf_next_enabled(cpu_if->irqbuf,
			    irqbuf_idx, cpu_if->irqbuf_num, hypctx, cpu_if);
		if (irqbuf_idx == -1)
			break;

		lr_idx = vgic_v3_lr_next_empty(cpu_if->ich_elrsr_el2,
		    lr_idx, cpu_if->ich_lr_num);
		if (lr_idx == -1)
			break;

		vip = &cpu_if->irqbuf[irqbuf_idx];
		if (vip->irqtype == VGIC_IRQ_CLK && clk_present) {
			/* Skip injecting timer interrupt. */
			irqbuf_idx++;
			continue;
		}

		vip_to_lr(vip, cpu_if->ich_lr_el2[lr_idx]);
		vip->irq = IRQ_SCHEDULED;
		irqbuf_idx++;
		lr_idx++;
	}

	/* Remove all interrupts that were just scheduled. */
	vgic_v3_irqbuf_remove_nolock(IRQ_SCHEDULED, cpu_if);
}

void
vgic_v3_sync_hwstate(void *arg)
{
	struct hypctx *hypctx;
	struct vgic_v3_cpu_if *cpu_if;
	int lr_free;
	int i;
	bool by_priority;
	bool en_underflow_intr;

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

	/* Exit early if there are no buffered interrupts */
	if (cpu_if->irqbuf_num == 0) {
		cpu_if->ich_hcr_el2 &= ~ICH_HCR_EL2_UIE;
		goto out;
	}

	/* Test if all buffered interrupts can fit in the LR regs */
	lr_free = 0;
	for (i = 0; i < cpu_if->ich_lr_num; i++)
		if (cpu_if->ich_elrsr_el2 & (1U << i))
			lr_free++;

	by_priority = (lr_free <= cpu_if->ich_lr_num);
	vgic_v3_irqbuf_to_lr(hypctx, cpu_if, by_priority);

	lr_free = 0;
	for (i = 0; i < cpu_if->ich_lr_num; i++)
		if (cpu_if->ich_elrsr_el2 & (1U << i))
			lr_free++;

	en_underflow_intr = false;
	if (cpu_if->irqbuf_num > 0)
		for (i = 0; i < cpu_if->irqbuf_num; i++)
			if (cpu_if->irqbuf[i].irqtype != VGIC_IRQ_CLK) {
				en_underflow_intr = true;
				break;
			}
	if (en_underflow_intr) {
		cpu_if->ich_hcr_el2 |= ICH_HCR_EL2_UIE;
	} else {
		cpu_if->ich_hcr_el2 &= ~ICH_HCR_EL2_UIE;
	}

out:
	mtx_unlock_spin(&cpu_if->lr_mtx);
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
	ro_regs.gicd_typer |= GICD_TYPER_LPIS;

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
