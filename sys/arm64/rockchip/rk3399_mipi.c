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
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/phy/phy.h>

#include <arm64/rockchip/rk3399_mipi.h>

#include "syscon_if.h"

#define	MIPI_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	MIPI_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-mipi-dsi",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_mipi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct rk_mipi_softc {
	struct simplebus_softc	sc;
	struct syscon		*syscon;
	struct rk_mipi_conf	*phy_conf;
	clk_t			ref;
	clk_t			pclk;
	clk_t			phy_cfg;
	clk_t			grf;
	struct resource		*res[2];
};

static int
rk_mipi_enable(device_t dev)
{
	struct rk_mipi_softc *sc;
	uint64_t rate_ref, rate_pclk, rate_phy_cfg, rate_grf;
	int error;

	sc = device_get_softc(dev);

	/* ref */
	error = clk_get_by_ofw_name(dev, 0, "ref", &sc->ref);
	if (error != 0) {
		device_printf(dev, "cannot get ref clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->ref, &rate_ref);
	if (error != 0) {
		device_printf(dev, "cannot get aclk frequency\n");
		return (ENXIO);
	}

	/* pclk */
	error = clk_get_by_ofw_name(dev, 0, "pclk", &sc->pclk);
	if (error != 0) {
		device_printf(dev, "cannot get pclk clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->pclk, &rate_pclk);
	if (error != 0) {
		device_printf(dev, "cannot get pclk frequency\n");
		return (ENXIO);
	}

	/* phy_cfg */
	error = clk_get_by_ofw_name(dev, 0, "phy_cfg", &sc->phy_cfg);
	if (error != 0) {
		device_printf(dev, "cannot get phy_cfg clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->phy_cfg, &rate_phy_cfg);
	if (error != 0) {
		device_printf(dev, "cannot get phy_cfg frequency\n");
		return (ENXIO);
	}

	/* grf */
	error = clk_get_by_ofw_name(dev, 0, "grf", &sc->grf);
	if (error != 0) {
		device_printf(dev, "cannot get grf clock\n");
		return (ENXIO);
	}

	error = clk_get_freq(sc->grf, &rate_grf);
	if (error != 0) {
		device_printf(dev, "cannot get grf frequency\n");
		return (ENXIO);
	}

	device_printf(dev, "ref rate is %ld\n", rate_ref);
	device_printf(dev, "pclk rate is %ld\n", rate_pclk);
	device_printf(dev, "phy_cfg rate is %ld\n", rate_phy_cfg);
	device_printf(dev, "grf rate is %ld\n", rate_grf);

	return (0);
}

static int
rk_mipi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 MIPI DSI");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_mipi_attach(device_t dev)
{
	struct rk_mipi_softc *sc;
	device_t cdev;
	phandle_t node;
	phandle_t child;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, rk_mipi_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	rk_mipi_enable(dev);

	simplebus_init(dev, node);
#if 0
	if (simplebus_fill_ranges(node, &sc->sc) < 0) {
		device_printf(dev, "could not get ranges\n");
		return (ENXIO);
	}
#endif

	bus_generic_probe(dev);

	/* Attach child devices */
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		device_printf(dev, "a child found\n");
		cdev = simplebus_add_device(dev, child, 0, NULL, -1, NULL);
		if (cdev != NULL) {
			//device_probe_and_attach(cdev);
			device_printf(dev, "child dev created\n");
		}
	}

	return (bus_generic_attach(dev));
}

static device_method_t rk_mipi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_mipi_probe),
	DEVMETHOD(device_attach,	rk_mipi_attach),
	DEVMETHOD_END
};

#if 0
static driver_t rk_mipi_driver = {
	"rk_mipi",
	rk_mipi_methods,
	sizeof(struct rk_mipi_softc)
};
#endif

DEFINE_CLASS_1(rk_mipi, rk_mipi_driver, rk_mipi_methods,
    sizeof(struct rk_mipi_softc), simplebus_driver);

static devclass_t rk_mipi_devclass;
EARLY_DRIVER_MODULE(rk_mipi, simplebus, rk_mipi_driver,
    rk_mipi_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
MODULE_VERSION(rk_mipi, 1);
