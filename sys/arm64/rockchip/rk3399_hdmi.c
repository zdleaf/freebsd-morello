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

#include <arm64/rockchip/rk3399_vop.h>
#include <arm64/rockchip/rk3399_hdmi.h>

#include "syscon_if.h"

#define	RD4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))
#define	ARRAY_SIZE(x)		(sizeof(x) / sizeof(x[0]))

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-dw-hdmi",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_hdmi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define	CLK_NENTRIES	5

struct rk_hdmi_softc {
	struct syscon		*syscon;
	struct rk_hdmi_conf	*phy_conf;
	clk_t			clk[CLK_NENTRIES];
	struct resource		*res[2];
	struct syscon		*grf;
};

static char * clk_table[CLK_NENTRIES] = {
	"iahb",
	"isfr",
	"vpll",
	"grf",
	"cec",
};

static int
rk_hdmi_enable(device_t dev)
{
	struct rk_hdmi_softc *sc;
	uint64_t rate;
	int error;
	int i;

	sc = device_get_softc(dev);

	for (i = 0; i < CLK_NENTRIES; i++) {
		error = clk_get_by_ofw_name(dev, 0, clk_table[i], &sc->clk[i]);
		if (error != 0) {
			device_printf(dev, "cannot get '%s' clock\n",
			    clk_table[i]);
			return (ENXIO);
		}

		error = clk_get_freq(sc->clk[i], &rate);
		if (error != 0) {
			device_printf(dev, "cannot get '%s' clock frequency\n",
			    clk_table[i]);
			return (ENXIO);
		}

		device_printf(dev, "%s rate is %ld\n", clk_table[i], rate);
	}

	return (0);
}

static void
rk_hdmi_dsi_enable(device_t dev, struct display_timing *timing)
{
	struct rk_hdmi_softc *sc;

	sc = device_get_softc(dev);
}

static void
rk_hdmi_phy_write(struct rk_hdmi_softc *sc, uint8_t test_code,
    uint8_t *test_data, uint8_t size)
{

}

static void
rk_hdmi_phy_enable(device_t dev, struct display_timing *timing)
{
}

static void
rk_hdmi_configure(struct rk_hdmi_softc *sc)
{
#if 0
	uint32_t reg;

	/* Select VOP Little for MIPI DSI. */
	reg = SYSCON_READ_4(sc->grf, GRF_SOC_CON20);
	SYSCON_WRITE_4(sc->grf, GRF_SOC_CON20, reg);

	reg = SYSCON_READ_4(sc->grf, GRF_SOC_CON22);
	SYSCON_WRITE_4(sc->grf, GRF_SOC_CON22, reg);
#endif
}

static int
rk_hdmi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 HDMI");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_hdmi_attach(device_t dev)
{
	struct rk_hdmi_softc *sc;
	phandle_t node;
	int err;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	err = syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf);
	if (err != 0) {
		device_printf(dev, "cannot get grf syscon: %d\n", err);
		return (ENXIO);
	}

	if (bus_alloc_resources(dev, rk_hdmi_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	struct display_timing *edid;

	/* TODO: read edid from a connected HDMI monitor. */
	edid = NULL;

	rk_hdmi_enable(dev);
	rk_hdmi_configure(sc);
	rk_hdmi_dsi_enable(dev, edid);
	rk_hdmi_phy_enable(dev, edid);

	return (0);
}

static device_method_t rk_hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_hdmi_probe),
	DEVMETHOD(device_attach,	rk_hdmi_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(rk_hdmi, rk_hdmi_driver, rk_hdmi_methods,
    sizeof(struct rk_hdmi_softc), simplebus_driver);

static devclass_t rk_hdmi_devclass;
EARLY_DRIVER_MODULE(rk_hdmi, simplebus, rk_hdmi_driver,
    rk_hdmi_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
MODULE_VERSION(rk_hdmi, 1);
