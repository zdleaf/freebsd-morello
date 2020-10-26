/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/gpio.h>
#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/phy/phy.h>

#include <arm64/rockchip/rk3399_vop.h>

#include "syscon_if.h"

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-vop-lit",	1 },
	{ NULL,				0 }
};

struct rk_vop_softc {
	struct syscon		*syscon;
	struct rk_vop_conf	*phy_conf;
	clk_t			aclk;
	clk_t			dclk;
	clk_t			hclk;
};

static int
rk_vop_enable(device_t dev, phandle_t node)
{
	struct rk_vop_softc *sc;
	uint64_t rate_aclk, rate_dclk, rate_hclk;
	int error;

	sc = device_get_softc(dev);

	//"aclk_vop", "dclk_vop", "hclk_vop";

	/* aclk */
	error = clk_get_by_ofw_name(dev, 0, "aclk_vop", &sc->aclk);
	if (error != 0) {
		device_printf(dev, "cannot get aclk_vop clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->aclk, &rate_aclk);
	if (error != 0) {
		device_printf(dev, "cannot get aclk frequency\n");
		return (ENXIO);
	}

	/* dclk */
	error = clk_get_by_ofw_name(dev, 0, "dclk_vop", &sc->dclk);
	if (error != 0) {
		device_printf(dev, "cannot get dclk_vop clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->dclk, &rate_dclk);
	if (error != 0) {
		device_printf(dev, "cannot get dclk frequency\n");
		return (ENXIO);
	}

	/* hclk */
	error = clk_get_by_ofw_name(dev, 0, "hclk_vop", &sc->hclk);
	if (error != 0) {
		device_printf(dev, "cannot get hclk_vop clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->hclk, &rate_hclk);
	if (error != 0) {
		device_printf(dev, "cannot get hclk frequency\n");
		return (ENXIO);
	}

	device_printf(dev, "aclk rate is %ld\n", rate_aclk);
	device_printf(dev, "dclk rate is %ld\n", rate_dclk);
	device_printf(dev, "hclk rate is %ld\n", rate_hclk);

	return (0);
}

static int
rk_vop_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 Visual Output Processor");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_vop_attach(device_t dev)
{
#if 0
	struct phynode_init_def phy_init;
	struct phynode *phynode;
	struct rk_vop_softc *sc;
	phandle_t node;
	phandle_t xnode;
	pcell_t handle;
	intptr_t phy;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, "clocks", (void *)&handle,
	    sizeof(handle)) <= 0) {
		device_printf(dev, "cannot get clocks handle\n");
		return (ENXIO);
	}
	xnode = OF_node_from_xref(handle);
	if (OF_hasprop(xnode, "arasan,soc-ctl-syscon") &&
	    syscon_get_by_ofw_property(dev, xnode,
	    "arasan,soc-ctl-syscon", &sc->syscon) != 0) {
		device_printf(dev, "cannot get grf driver handle\n");
		return (ENXIO);
	}

	if (sc->syscon == NULL) {
		device_printf(dev, "failed to get syscon\n");
		return (ENXIO);
	}
#endif

	phandle_t node;

	node = ofw_bus_get_node(dev);

	rk_vop_enable(dev, node);

	return (0);
}

static device_method_t rk_vop_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_vop_probe),
	DEVMETHOD(device_attach,	rk_vop_attach),
	DEVMETHOD_END
};

static driver_t rk_vop_driver = {
	"rk_vop",
	rk_vop_methods,
	sizeof(struct rk_vop_softc)
};

static devclass_t rk_vop_devclass;
EARLY_DRIVER_MODULE(rk_vop, simplebus, rk_vop_driver,
    rk_vop_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
MODULE_VERSION(rk_vop, 1);
