/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Ruslan Bukin <br@bsdpad.com>
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

/*
 * Device interface.
 */
int
smmu_attach(device_t dev)
{
	struct smmu_softc *sc;
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
