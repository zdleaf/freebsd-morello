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

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/phy/phy.h>

#include <arm64/rockchip/rk3399_vop.h>

#include "syscon_if.h"

#define	VOP_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	VOP_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-vop-lit",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_vop_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

/*
timing0 {
	clock-frequency = <0x07270e00>;
	hactive = <0x00000440>;
	hfront-porch = <0x00000018>;
	hback-porch = <0x00000017>;
	hsync-len = <0x00000004>;
	vactive = <0x00000780>;
	vfront-porch = <0x00000004>;
	vback-porch = <0x00000003>;
	vsync-len = <0x00000002>;
	hsync-active = <0x00000000>;
	vsync-active = <0x00000000>;
	de-active = <0x00000000>;
	pixelclk-active = <0x00000000>;
	phandle = <0x000000ba>;
};
*/

enum display_flags {
	DISPLAY_FLAGS_HSYNC_LOW		= 1 << 0,
	DISPLAY_FLAGS_HSYNC_HIGH	= 1 << 1,
	DISPLAY_FLAGS_VSYNC_LOW		= 1 << 2,
	DISPLAY_FLAGS_VSYNC_HIGH	= 1 << 3,

	/* data enable flag */
	DISPLAY_FLAGS_DE_LOW		= 1 << 4,
	DISPLAY_FLAGS_DE_HIGH		= 1 << 5,
	/* drive data on pos. edge */
	DISPLAY_FLAGS_PIXDATA_POSEDGE	= 1 << 6,
	/* drive data on neg. edge */
	DISPLAY_FLAGS_PIXDATA_NEGEDGE	= 1 << 7,
	DISPLAY_FLAGS_INTERLACED	= 1 << 8,
	DISPLAY_FLAGS_DOUBLESCAN	= 1 << 9,
	DISPLAY_FLAGS_DOUBLECLK		= 1 << 10,
};

struct timing_entry {
	uint32_t min;
	uint32_t typ;
	uint32_t max;
};

struct display_timing {
	struct timing_entry pixelclock;

	struct timing_entry hactive;		/* hor. active video */
	struct timing_entry hfront_porch;	/* hor. front porch */
	struct timing_entry hback_porch;	/* hor. back porch */
	struct timing_entry hsync_len;		/* hor. sync len */

	struct timing_entry vactive;		/* ver. active video */
	struct timing_entry vfront_porch;	/* ver. front porch */
	struct timing_entry vback_porch;	/* ver. back porch */
	struct timing_entry vsync_len;		/* ver. sync len */

	enum display_flags flags;		/* display flags */
	bool hdmi_monitor;			/* is hdmi monitor? */
};

struct display_timing ts050_timings;

struct rk_vop_softc {
	struct syscon		*syscon;
	struct rk_vop_conf	*phy_conf;
	clk_t			aclk;
	clk_t			dclk;
	clk_t			hclk;
	struct resource		*res[2];
};

static void
rk_vop_set_polarity(struct rk_vop_softc *sc, uint32_t pin_polarity)
{
	uint32_t reg;

	/* MIPI */
	reg = VOP_READ(sc, RK3399_DSP_CTRL1);
	reg &= ~DSP_CTRL1_MIPI_POL_M;
	reg |= pin_polarity << DSP_CTRL1_MIPI_POL_S;
	VOP_WRITE(sc, RK3399_DSP_CTRL1, reg);
}

static int
rk_vop_mode_set(device_t dev, struct display_timing *edid)
{
	struct rk_vop_softc *sc;
	uint32_t pin_polarity;
	uint32_t mode;
	uint32_t reg;

	sc = device_get_softc(dev);

	pin_polarity = (1 << DCLK_INVERT);
	rk_vop_set_polarity(sc, pin_polarity);

	/* Remove standby bit */
	reg = VOP_READ(sc, RK3399_SYS_CTRL);
	reg &= ~SYS_CTRL_STANDBY_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	/* Enable MIPI output only. */
	reg &= ~SYS_CTRL_ALL_OUT_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);
	reg |= SYS_CTRL_MIPI_OUT_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	/* Set mode */
	mode = 0; /* RGB888 */
	reg = VOP_READ(sc, RK3399_DSP_CTRL0);
	reg &= ~DSP_CTRL0_OUT_MODE_M;
	reg |= (mode << DSP_CTRL0_OUT_MODE_S);
	VOP_WRITE(sc, RK3399_DSP_CTRL0, reg);

	uint32_t hactive = edid->hactive.typ;
	uint32_t vactive = edid->vactive.typ;
	uint32_t hsync_len = edid->hsync_len.typ;
	uint32_t hback_porch = edid->hback_porch.typ;
	uint32_t vsync_len = edid->vsync_len.typ;
	uint32_t vback_porch = edid->vback_porch.typ;
	uint32_t hfront_porch = edid->hfront_porch.typ;
	uint32_t vfront_porch = edid->vfront_porch.typ;

	reg = hsync_len;
	reg |= (hsync_len + hback_porch + hactive + hfront_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_HTOTAL_HS_END, reg);

	reg = (hsync_len + hback_porch + hactive);
	reg |= (hsync_len + hback_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_HACT_ST_END, reg);

	reg = vsync_len;
	reg |= (vsync_len + vback_porch + vactive + vfront_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_VTOTAL_VS_END, reg);

	reg = (vsync_len + vback_porch + vactive);
	reg |= (vsync_len + vback_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_VACT_ST_END, reg);

	reg = hsync_len + hback_porch + hactive;
	reg |= (hsync_len + hback_porch) << 16;
	VOP_WRITE(sc, RK3399_POST_DSP_HACT_INFO, reg);

	reg = vsync_len + vback_porch + vactive;
	reg |= (vsync_len + vback_porch) << 16;
	VOP_WRITE(sc, RK3399_POST_DSP_VACT_INFO, reg);

	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

	return (0);
}

static int
rk_vop_enable(device_t dev, phandle_t node, struct display_timing *edid)
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

	uint32_t reg;
	reg = (edid->hactive.typ - 1);
	reg |= (edid->vactive.typ - 1) << 16;
	VOP_WRITE(sc, RK3399_WIN0_ACT_INFO, reg);

	reg = (edid->hsync_len.typ + edid->hback_porch.typ);
	reg |= (edid->vsync_len.typ + edid->vback_porch.typ) << 16;
	VOP_WRITE(sc, RK3399_WIN0_DSP_ST, reg);

	reg = (edid->hactive.typ - 1);
	reg |= (edid->vactive.typ - 1) << 16;
	VOP_WRITE(sc, RK3399_WIN0_DSP_INFO, reg);

	reg = VOP_READ(sc, RK3399_WIN0_COLOR_KEY);
	device_printf(dev, "color key %x\n", reg);
	reg = 0;
	VOP_WRITE(sc, RK3399_WIN0_COLOR_KEY, reg);

	uint32_t lb_mode, rgb_mode;
	int bpp;

	bpp = 24; //lets say

	switch (bpp) {
	case 24:
		rgb_mode = RGB888;
		VOP_WRITE(sc, RK3399_WIN0_VIR,
		    WIN0_VIR_WIDTH_RGB888(edid->hactive.typ));
		break;
	default:
		panic("unknown bpp");
	};

	if (edid->hactive.typ <= 1280)
		lb_mode = LB_RGB_1280X8;
	else
		panic("unknown lb_mode");

	reg = VOP_READ(sc, RK3399_WIN0_CTRL0);
	device_printf(dev, "win0 ctrl0 %x\n", reg);
	reg &= ~WIN0_CTRL0_LB_MODE_M;
	reg &= ~WIN0_CTRL0_DATA_FMT_M;
	reg &= ~WIN0_CTRL0_EN;
	VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);

	reg |= lb_mode << WIN0_CTRL0_LB_MODE_S;
	reg |= rgb_mode << WIN0_CTRL0_DATA_FMT_S;
	reg |= WIN0_CTRL0_EN;
	VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);

	uint64_t vbase;
	uint64_t fb_base;
	int sz;
	sz = edid->hactive.typ * edid->vactive.typ * 3;
	vbase = (intptr_t)kmem_alloc_contig(sz, M_ZERO, 0, ~0, PAGE_SIZE, 0,
	    VM_MEMATTR_UNCACHEABLE);
	fb_base = (intptr_t)vtophys(vbase);

	device_printf(dev, "fb_base %lx\n", fb_base);

	VOP_WRITE(sc, RK3399_WB_YRGB_MST, fb_base);
	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

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
	struct rk_vop_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, rk_vop_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	struct display_timing *edid;

	/* TODO: read edid from DTS */

	edid = &ts050_timings;
	edid->pixelclock.typ = 0x07270e00;
	edid->hactive.typ = 0x00000440;
	edid->hfront_porch.typ = 0x00000018;
	edid->hback_porch.typ = 0x00000017;
	edid->hsync_len.typ = 0x00000004;
	edid->vactive.typ = 0x00000780;
	edid->vfront_porch.typ = 0x00000004;
	edid->vback_porch.typ = 0x00000003;
	edid->vsync_len.typ = 0x00000002;

	rk_vop_mode_set(dev, edid);
	rk_vop_enable(dev, node, edid);

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
