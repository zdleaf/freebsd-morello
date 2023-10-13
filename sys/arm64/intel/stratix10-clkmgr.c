/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
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

/*
 * Intel Stratix 10 Clock Manager.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/vmem.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#if 0
#include <arm64/intel/stratix10-clkmgr.h>
#endif

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

struct s10_clkmgr_softc {
	device_t		dev;
	struct resource		*mem_res;
};

static int
s10_clkmgr_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "altr,clk-mgr"))
		return (ENXIO);

	device_set_desc(dev, "Stratix 10 Clock Manager");

	return (BUS_PROBE_DEFAULT);
}

static int
s10_clkmgr_attach(device_t dev)
{
	struct s10_clkmgr_softc *sc;
	int rid;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		return (ENXIO);
	}

	printf("%s: group 1\n", __func__);
	for (i = 0; i < 0x24; i += 4)
		printf("reg %04x: %04x\n", i, bus_read_4(sc->mem_res, i));

	printf("%s: group 2\n", __func__);
	for (i = 0x30; i < 0x90; i += 4)
		printf("reg %04x: %04x\n", i, bus_read_4(sc->mem_res, i));

	printf("%s: group 3\n", __func__);
	for (i = 0xa4; i < 0x100; i += 4)
		printf("reg %04x: %04x\n", i, bus_read_4(sc->mem_res, i));

	return (0);
}

static device_method_t s10_clkmgr_methods[] = {
	DEVMETHOD(device_probe,		s10_clkmgr_probe),
	DEVMETHOD(device_attach,	s10_clkmgr_attach),
	{ 0, 0 }
};

static driver_t s10_clkmgr_driver = {
	"s10_clkmgr",
	s10_clkmgr_methods,
	sizeof(struct s10_clkmgr_softc),
};

EARLY_DRIVER_MODULE(s10_clkmgr, simplebus, s10_clkmgr_driver,
    0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
