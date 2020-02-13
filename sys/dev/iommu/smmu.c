/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019-2020 Ruslan Bukin <br@bsdpad.com>
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
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#ifdef FDT
#include <dev/fdt/fdt_intr.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#include <arm/arm/gic_common.h>
#include "smmu_reg.h"
#include "smmu_var.h"

static bus_get_domain_t smmu_get_domain;
static bus_read_ivar_t smmu_read_ivar;

static device_method_t smmu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_detach,	smmu_detach),

	/* Bus interface */
	DEVMETHOD(bus_get_domain,	smmu_get_domain),
	DEVMETHOD(bus_read_ivar,	smmu_read_ivar),

	/* End */
	DEVMETHOD_END
};

static struct resource_spec smmu_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 1, RF_ACTIVE },
	{ SYS_RES_IRQ, 2, RF_ACTIVE },
	RESOURCE_SPEC_END
};

DEFINE_CLASS_0(gic, smmu_driver, smmu_methods,
    sizeof(struct smmu_softc));

/*
 * Driver-specific definitions.
 */
MALLOC_DEFINE(M_SMMU, "SMMU", SMMU_DEVSTR);

#define	STRTAB_L1_SZ_SHIFT	20
#define	STRTAB_SPLIT		8
#define	STRTAB_L1_DESC_DWORDS	1

#define	STRTAB_STE_DWORDS	8

static int
smmu_event_intr(void *arg)
{

	printf("%s\n", __func__);

	return (FILTER_HANDLED);
}

static int
smmu_sync_intr(void *arg)
{

	printf("%s\n", __func__);

	return (FILTER_HANDLED);
}

static int
smmu_gerr_intr(void *arg)
{

	printf("%s\n", __func__);

	return (FILTER_HANDLED);
}

static inline int
ilog2(long x)
{

	KASSERT(x > 0 && powerof2(x),
	    ("%s: invalid arg %ld", __func__, x));

	return (flsl(x) - 1);
}

#define	SMMU_CMDQ_ALIGN	(64 * 1024)

static int
smmu_init_queue(struct smmu_softc *sc, struct smmu_queue *q,
    uint32_t prod_off, uint32_t cons_off)
{
	int sz;

	sz = (1 << q->size_log2) * 16;

	printf("%s: allocating %d bytes\n", __func__, sz);

	/* Set up the command circular buffer */
	q->addr = contigmalloc(sz, M_SMMU,
	    M_WAITOK | M_ZERO, 0, (1ul << 48) - 1, SMMU_CMDQ_ALIGN, 0);
	if (q->addr == NULL) {
		printf("failed to allocate %d bytes\n", sz);
		return (-1);
	}

	q->prod_off = prod_off;
	q->cons_off = cons_off;
	q->paddr = vtophys(q->addr);

	q->base = CMDQ_BASE_RA | EVENTQ_BASE_WA | PRIQ_BASE_WA;
	q->base |= q->paddr & Q_BASE_ADDR_M;
	q->base |= q->size_log2 << Q_LOG2SIZE_S;

	printf("%s: queue addr %p paddr %lx\n", __func__, q->addr, q->paddr);

	return (0);
}

static int
smmu_init_queues(struct smmu_softc *sc)
{
	int err;

	/* Command queue */
	err = smmu_init_queue(sc, &sc->cmd_q,
	    SMMU_CMDQ_PROD, SMMU_CMDQ_CONS);
	if (err)
		return (ENXIO);

	/* Event queue */
	err = smmu_init_queue(sc, &sc->evt_q,
	    SMMU_EVENTQ_PROD, SMMU_EVENTQ_CONS);
	if (err)
		return (ENXIO);

	if (!(sc->features & SMMU_FEATURE_PRI)) {
		printf("no PRI queue is available\n");
		return (0);
	}

	/* PRI queue */
	err = smmu_init_queue(sc, &sc->pri_q,
	    SMMU_PRIQ_PROD, SMMU_PRIQ_CONS);
	if (err)
		return (ENXIO);

	return (0);
}

static int
smmu_init_ste(struct smmu_softc *sc, uint64_t *ste)
{

	ste[0] = STE0_VALID | STE0_CONFIG_ABORT;
	ste[1] = STE1_SHCFG_INCOMING;
	ste[2] = 0;

	return (0);
}

static int
smmu_init_bypass(struct smmu_softc *sc,
    uint64_t *strtab, int num_l1_entries)
{
	int i;

	for (i = 0; i < num_l1_entries; i++) {
		smmu_init_ste(sc, strtab);
		strtab += STRTAB_STE_DWORDS;
	}

	return (0);
}

static int
smmu_init_strtab_linear(struct smmu_softc *sc)
{
	struct smmu_strtab *strtab;
	uint32_t num_l1_entries;
	uint32_t size;
	uint32_t reg;

	strtab = &sc->strtab;
	num_l1_entries = (1 << sc->sid_bits);

	size = num_l1_entries * (STRTAB_STE_DWORDS << 3);
	printf("%s: linear strtab size %d, num_l1_entries %d\n",
	    __func__, size, num_l1_entries);

	strtab->addr = contigmalloc(size, M_SMMU,
	    M_WAITOK | M_ZERO,	/* flags */
	    0,			/* low */
	    (1ul << 48) - 1,	/* high */
	    SMMU_CMDQ_ALIGN,	/* alignment */
	    0);			/* boundary */
	if (strtab->addr == NULL) {
		printf("failed to allocate strtab\n");
		return (ENXIO);
	}

	reg = STRTAB_BASE_CFG_FMT_LINEAR;
	reg |= sc->sid_bits << STRTAB_BASE_CFG_LOG2SIZE_S;
	strtab->base_cfg = reg;

	reg = vtophys(strtab->addr) & STRTAB_BASE_ADDR_M;
	reg |= STRTAB_BASE_RA;
	strtab->base = reg;

	smmu_init_bypass(sc, strtab->addr, num_l1_entries);

	return (0);
}

static int
smmu_init_strtab_2lvl(struct smmu_softc *sc)
{
	uint32_t size;
	uint32_t num_l1_entries;
	uint32_t l1size;

	size = STRTAB_L1_SZ_SHIFT - (ilog2(STRTAB_L1_DESC_DWORDS) + 3);
	size = min(size, sc->sid_bits - STRTAB_SPLIT);
	num_l1_entries = (1 << size);
	size += STRTAB_SPLIT;

	l1size = num_l1_entries * (STRTAB_L1_DESC_DWORDS << 3);

	printf("%s: size %d, l1 entries %d, l1size %d\n",
	    __func__, size, num_l1_entries, l1size);

	void *strtab;

	strtab = contigmalloc(l1size, M_SMMU,
	    M_WAITOK | M_ZERO,	/* flags */
	    0,			/* low */
	    (1ul << 48) - 1,	/* high */
	    SMMU_CMDQ_ALIGN,	/* alignment */
	    0);			/* boundary */
	if (strtab == NULL) {
		printf("failed to allocate strtab\n");
		return (ENXIO);
	}

	printf("%s: strtab %p\n", __func__, strtab);

	uint32_t reg;
	reg = STRTAB_BASE_CFG_FMT_2LVL;
	reg |= size << STRTAB_BASE_CFG_LOG2SIZE_S;
	reg |= STRTAB_SPLIT << STRTAB_BASE_CFG_SPLIT_S;
	bus_write_4(sc->res[0], SMMU_STRTAB_BASE_CFG, reg);

	reg = vtophys(strtab) & STRTAB_BASE_ADDR_M;
	reg |= STRTAB_BASE_RA;
	bus_write_4(sc->res[0], SMMU_STRTAB_BASE, reg);

	return (0);
}

static int
smmu_init_strtab(struct smmu_softc *sc)
{
	int error;

	/* XXX: Try linear for now. */
	sc->features &= ~SMMU_FEATURE_2_LVL_STREAM_TABLE;

	if (sc->features & SMMU_FEATURE_2_LVL_STREAM_TABLE)
		error = smmu_init_strtab_2lvl(sc);
	else
		error = smmu_init_strtab_linear(sc);

	return (error);
}

static int
smmu_write_ack(struct smmu_softc *sc, uint32_t reg,
    uint32_t reg_ack, uint32_t val)
{
	uint32_t v;
	int timeout;

	timeout = 100000;

	bus_write_4(sc->res[0], reg, val);

	do {
		v = bus_read_4(sc->res[0], reg_ack);
		if (v == val)
			break;
	} while (timeout--);

	if (timeout <= 0) {
		device_printf(sc->dev, "Failed to write reg.\n");
		return (-1);
	}

	return (0);
}

static int
smmu_disable(struct smmu_softc *sc)
{
	int ret;

	ret = smmu_write_ack(sc, SMMU_CR0, SMMU_CR0ACK, 0);

	return (ret);
}

static int
smmu_cmdq_issue_cmd(struct smmu_softc *sc, struct smmu_cmdq_entry *cmd)
{

	return (0);
}

static int
smmu_cmdq_issue_sync(struct smmu_softc *sc)
{

	return (0);
}

static int
smmu_reset(struct smmu_softc *sc)
{
	struct smmu_strtab *strtab;
	int error;
	int reg;

	reg = bus_read_4(sc->res[0], SMMU_CR0);

	if (reg & CR0_SMMUEN)
		device_printf(sc->dev,
		    "%s: Warning: SMMU is enabled\n", __func__);

	error = smmu_disable(sc);
	if (error)
		device_printf(sc->dev,
		    "%s: Could not disable SMMU.\n", __func__);

	reg = CR1_TABLE_SH_IS
	    | CR1_TABLE_OC_WBC
	    | CR1_TABLE_IC_WBC
	    | CR1_QUEUE_SH_IS
	    | CR1_QUEUE_OC_WBC
	    | CR1_QUEUE_IC_WBC;
	bus_write_4(sc->res[0], SMMU_CR1, reg);

	reg = CR2_PTM | CR2_RECINVSID | CR2_E2H;
	bus_write_4(sc->res[0], SMMU_CR2, reg);

	/* Stream table. */
	strtab = &sc->strtab;
	bus_write_4(sc->res[0], SMMU_STRTAB_BASE_CFG, strtab->base);
	bus_write_4(sc->res[0], SMMU_STRTAB_BASE, strtab->base_cfg);

	/* Command queue. */
	bus_write_4(sc->res[0], SMMU_CMDQ_BASE, sc->cmd_q.base);
	bus_write_4(sc->res[0], SMMU_CMDQ_PROD, sc->cmd_q.prod_off);
	bus_write_4(sc->res[0], SMMU_CMDQ_CONS, sc->cmd_q.cons_off);

	error = smmu_write_ack(sc, SMMU_CR0, SMMU_CR0ACK, CR0_CMDQEN);
	if (error) {
		device_printf(sc->dev, "Could not enable command queue\n");
		return (ENXIO);
	}

	struct smmu_cmdq_entry cmd;
	cmd.opcode = CMD_CFGI_STE_RANGE;
	smmu_cmdq_issue_cmd(sc, &cmd);
	smmu_cmdq_issue_sync(sc);

	return (0);
}

/*
 * Device interface.
 */
int
smmu_attach(device_t dev)
{
	struct smmu_softc *sc;
	uint32_t reg;
	int error;

	sc = device_get_softc(dev);

	sc->dev = dev;

	error = bus_alloc_resources(dev, smmu_spec, sc->res);
	if (error) {
		device_printf(dev, "Couldn't allocate resources\n");
		return (ENXIO);
	}

	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC,
	    smmu_event_intr, NULL, sc, &sc->intr_cookie[0]);
	if (error) {
		device_printf(dev, "Couldn't setup Event interrupt handler\n");
		goto fail;
	}

	error = bus_setup_intr(dev, sc->res[2], INTR_TYPE_MISC,
	    smmu_sync_intr, NULL, sc, &sc->intr_cookie[1]);
	if (error) {
		device_printf(dev, "Couldn't setup Sync interrupt handler\n");
		goto fail;
	}

	error = bus_setup_intr(dev, sc->res[3], INTR_TYPE_MISC,
	    smmu_gerr_intr, NULL, sc, &sc->intr_cookie[2]);
	if (error) {
		device_printf(dev, "Couldn't setup Gerr interrupt handler\n");
		goto fail;
	}

	reg = bus_read_4(sc->res[0], SMMU_IDR0);
	printf("IDR0 %x\n", reg);

	sc->features = 0;
	if (reg & IDR0_ST_LVL_2) {
		device_printf(sc->dev, "2-level stream table supported\n");
		sc->features |= SMMU_FEATURE_2_LVL_STREAM_TABLE;
	}

	if (reg & IDR0_CD2L) {
		device_printf(sc->dev, "2-level CD table supported.\n");
		sc->features |= SMMU_FEATURE_2_LVL_CD;
	}

	switch (reg & IDR0_TTENDIAN_M) {
	case IDR0_TTENDIAN_MIXED:
		device_printf(sc->dev, "Mixed endianess supported.\n");
		sc->features |= SMMU_FEATURE_TT_LE;
		sc->features |= SMMU_FEATURE_TT_BE;
		break;
	case IDR0_TTENDIAN_LITTLE:
		device_printf(sc->dev, "Little endian supported only.\n");
		sc->features |= SMMU_FEATURE_TT_LE;
		break;
	case IDR0_TTENDIAN_BIG:
		device_printf(sc->dev, "Big endian supported only.\n");
		sc->features |= SMMU_FEATURE_TT_BE;
		break;
	default:
		device_printf(sc->dev, "Unsupported endianness.\n");
		return (ENXIO);
	}

	if (reg & IDR0_SEV)
		sc->features |= SMMU_FEATURE_SEV;

	if (reg & IDR0_MSI)
		sc->features |= SMMU_FEATURE_MSI;

	if (reg & IDR0_HYP)
		sc->features |= SMMU_FEATURE_HYP;

	if (reg & IDR0_ATS)
		sc->features |= SMMU_FEATURE_ATS;

	if (reg & IDR0_PRI)
		sc->features |= SMMU_FEATURE_PRI;

	switch (reg & IDR0_STALL_MODEL_M) {
	case IDR0_STALL_MODEL_FORCE:
		/* Stall is forced. */
		sc->features |= SMMU_FEATURE_STALL_FORCE;
		/* FALLTHROUGH */
	case IDR0_STALL_MODEL_STALL:
		sc->features |= SMMU_FEATURE_STALL;
		break;
	}

	/* Grab translation stages supported. */
	if (reg & IDR0_S1P)
		sc->features |= SMMU_FEATURE_S1P;
	if (reg & IDR0_S2P)
		sc->features |= SMMU_FEATURE_S2P;

	switch (reg & IDR0_TTF_M) {
	case IDR0_TTF_ALL:
	case IDR0_TTF_AA64:
		sc->ias = 40;
		break;
	default:
		device_printf(dev, "No AArch64 table format support\n");
		return (ENXIO);
	}

	if (reg & IDR0_ASID16)
		sc->asid_bits = 16;
	else
		sc->asid_bits = 8;

	if (reg & IDR0_VMID16)
		sc->vmid_bits = 16;
	else
		sc->vmid_bits = 8;

	reg = bus_read_4(sc->res[0], SMMU_IDR1);
	printf("IDR1 %x\n", reg);

	if (reg & (IDR1_TABLES_PRESET | IDR1_QUEUES_PRESET | IDR1_REL)) {
		device_printf(dev, "Embedded implementations not supported\n");
		return (ENXIO);
	}

	uint32_t val;
	val = (reg & IDR1_CMDQS_M) >> IDR1_CMDQS_S;
	sc->cmd_q.size_log2 = val;
	printf("CMD queue size %d\n", val);

	val = (reg & IDR1_EVENTQS_M) >> IDR1_EVENTQS_S;
	sc->evt_q.size_log2 = val;
	printf("EVENT queue size %d\n", val);

	val = (reg & IDR1_PRIQS_M) >> IDR1_PRIQS_S;
	sc->pri_q.size_log2 = val;
	printf("PRI queue size %d\n", val);

	sc->ssid_bits = (reg & IDR1_SSIDSIZE_M) >> IDR1_SSIDSIZE_S;
	sc->sid_bits = (reg & IDR1_SIDSIZE_M) >> IDR1_SIDSIZE_S;

	if (sc->sid_bits <= STRTAB_SPLIT)
		sc->features &= ~SMMU_FEATURE_2_LVL_STREAM_TABLE;

	device_printf(dev, "ssid_bits %d\n", sc->ssid_bits);
	device_printf(dev, "sid_bits %d\n", sc->sid_bits);

	/* IDR5 */
	reg = bus_read_4(sc->res[0], SMMU_IDR5);
	printf("IDR1 %x\n", reg);

	switch (reg & IDR5_OAS_M) {
	case IDR5_OAS_32:
		sc->oas = 32;
		break;
	case IDR5_OAS_36:
		sc->oas = 36;
		break;
	case IDR5_OAS_40:
		sc->oas = 40;
		break;
	case IDR5_OAS_42:
		sc->oas = 42;
		break;
	case IDR5_OAS_44:
		sc->oas = 44;
		break;
	case IDR5_OAS_48:
		sc->oas = 48;
		break;
	case IDR5_OAS_52:
		sc->oas = 52;
		break;
	}

	printf("oas %d\n", sc->oas);

	sc->pgsizes = 0;
	if (reg & IDR5_GRAN64K)
		sc->pgsizes |= 64 * 1024;
	if (reg & IDR5_GRAN16K)
		sc->pgsizes |= 16 * 1024;
	if (reg & IDR5_GRAN4K)
		sc->pgsizes |= 4 * 1024;

	printf("pgsizes %x\n", sc->pgsizes);

	if ((reg & IDR5_VAX_M) == IDR5_VAX_52)
		sc->features |= SMMU_FEATURE_VAX;

	error = smmu_init_queues(sc);
	if (error) {
		device_printf(dev, "Couldn't allocate queues.\n");
		return (ENXIO);
	}

	error = smmu_init_strtab(sc);
	if (error) {
		device_printf(dev, "Couldn't allocate strtab.\n");
		return (ENXIO);
	}

	error = smmu_reset(sc);
	if (error) {
		device_printf(dev, "Couldn't reset SMMU.\n");
		return (ENXIO);
	}

	return (0);

fail:
	bus_release_resources(dev, smmu_spec, sc->res);

	return (0);
}

int
smmu_detach(device_t dev)
{
	struct smmu_softc *sc;

	sc = device_get_softc(dev);

	//if (device_is_attached(dev)) {
	//}

	bus_release_resources(dev, smmu_spec, sc->res);

	return (0);
}

static int
smmu_get_domain(device_t dev, device_t child, int *domain)
{
	//struct smmu_devinfo *di;

	//di = device_get_ivars(child);
	//if (di->smmu_domain < 0)
	//	return (ENOENT);

	//*domain = di->smmu_domain;

	printf("%s\n", __func__);

	return (0);
}

static int
smmu_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
	//struct smmu_softc *sc;

	//sc = device_get_softc(dev);

	printf("%s\n", __func__);

	return (ENOENT);
}
