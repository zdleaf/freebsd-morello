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

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/phy/phy.h>

#include <arm64/rockchip/rk3399_vop.h>
#include <arm64/rockchip/rk3399_hdmi.h>

#include <dev/hdmi/dwc_hdmi.h>

#include "hdmi_if.h"
#include "syscon_if.h"

#define	RD4(sc, reg)		bus_read_4((sc)->res[0], ((reg) << 2))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res[0], ((reg) << 2), (val))
#define	ARRAY_SIZE(x)		(sizeof(x) / sizeof(x[0]))

#define	DDC_SLAVE_ADDR		0x50
#define	DDC_EDID_SEG_ADDR	0x30
#define	HDMI_EDID_BLOCK_SIZE	128

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-dw-hdmi",	1 },
	{ NULL,				0 }
};

#if 0
static struct resource_spec rk_hdmi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};
#endif

#define	CLK_NENTRIES	5

struct rk_hdmi_softc {
	struct dwc_hdmi_softc	base;
	struct syscon		*syscon;
	struct rk_hdmi_conf	*phy_conf;
	clk_t			clk[CLK_NENTRIES];
	struct resource		*res[2];
	struct syscon		*grf;
	phandle_t		i2c_xref;
	eventhandler_tag	eh_tag;
};

static char * clk_table[CLK_NENTRIES] = {
	"iahb",
	"isfr",
	"vpll",
	"grf",
	"cec",
};

#if 0
static const struct hdmi_phy_config rockchip_phy_config[] = {
	{
		.mpixelclock = 74250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0272,
	}, {
		.mpixelclock = 148500000,
		.sym_ctr = 0x802b, .term = 0x0004, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = 297000000,
		.sym_ctr = 0x8039, .term = 0x0005, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = 584000000,
		.sym_ctr = 0x8039, .term = 0x0000, .vlev_ctr = 0x019d,
	}, {
		.mpixelclock = ~0ul,
		.sym_ctr = 0x0000, .term = 0x0000, .vlev_ctr = 0x0000,
	}
};
#endif

static const struct hdmi_mpll_config rockchip_mpll_cfg[] = {
	{
		.mpixelclock = 40000000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0018,
	}, {
		.mpixelclock = 65000000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 66000000,
		.cpce = 0x013e, .gmp = 0x0003, .curr = 0x0038,
	}, {
		.mpixelclock = 83500000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 146250000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x0038,
	}, {
		.mpixelclock = 148500000,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = 272000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = 340000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = ~0ul,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}
};

static device_t
imx_hdmi_get_i2c_dev(device_t dev)
{
	struct rk_hdmi_softc *sc;

	sc = device_get_softc(dev);
	if (sc->i2c_xref == 0)
		return (NULL);

	device_printf(dev, "%s: i2c dev returned\n", __func__);

	return (OF_device_from_xref(sc->i2c_xref));
}

static int
rk_hdmi_wait_i2c(struct rk_hdmi_softc *sc)
{
	uint32_t reg;
	int timeout;

	timeout = 10000;

	do {
		reg = RD4(sc, HDMI_IH_I2CM_STAT0);
		if (reg & IH_I2CM_STAT0_I2C_MASTER_DONE) {
			WR4(sc, HDMI_IH_I2CM_STAT0, reg);
			return (0);
		}
		DELAY(100);
	} while (timeout--);

	return (1);
}

static int
rk_hdmi_read_edid(device_t dev, int block, struct display_timing *edid)
{
	struct rk_hdmi_softc *sc;
	uint32_t i2c_clk_high, i2c_clk_low;
	uint32_t reg;
	uint8_t data;
	int error;
	int shift;
	int i;

	sc = device_get_softc(dev);

	i2c_clk_high = 0x7a;
	i2c_clk_low = 0x8d;
	shift = (block % 2) * 0x80;

	WR4(sc, HDMI_I2CM_SS_SCL_HCNT_0_ADDR, i2c_clk_high);
	WR4(sc, HDMI_I2CM_SS_SCL_LCNT_0_ADDR, i2c_clk_low);

	reg = RD4(sc, HDMI_I2CM_DIV);
	reg &= ~I2CM_DIV_FAST_STD_MODE;
	WR4(sc, HDMI_I2CM_DIV, reg);

	WR4(sc, HDMI_I2CM_SLAVE, DDC_SLAVE_ADDR);
	WR4(sc, HDMI_I2CM_SEGADDR, DDC_EDID_SEG_ADDR);
	WR4(sc, HDMI_I2CM_SEGPTR, block >> 1);

	for (i = 0; i < HDMI_EDID_BLOCK_SIZE; i++) {
		WR4(sc, HDMI_I2CM_ADDRESS, shift + i);

		if (block == 0)
			WR4(sc, HDMI_I2CM_OPERATION, I2CM_OPERATION_RD);
		else
			WR4(sc, HDMI_I2CM_OPERATION, I2CM_OPERATION_RD_EXT);

		error = rk_hdmi_wait_i2c(sc);
		if (error != 0) {
			printf("Could not read EDID data\n");
			return (-1);
		}

		data = RD4(sc, HDMI_I2CM_DATAI);
		printf("data %x\n", data);
		if (data != 0)
			panic("ok");
	}

	return (0);
}

static void
rk_hdmi_init(device_t dev)
{
	struct rk_hdmi_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	reg = IH_MUTE_WAKEUP_INTERRUPT | IH_MUTE_WAKEUP_INTERRUPT;
	WR4(sc, HDMI_IH_MUTE, reg);

	WR4(sc, HDMI_PHY_I2CM_INT, PHY_I2CM_INT_DONE_M);

	reg = PHY_I2CM_CTLINT_NACK_M | PHY_I2CM_CTLINT_ARBITRATION_M;
	WR4(sc, HDMI_PHY_I2CM_CTLINT, reg);
};

static void
rk_hdmi_phy_init(device_t dev)
{
	struct rk_hdmi_softc *sc;

	sc = device_get_softc(dev);

	/* Unmask hot plug interrupt */
	WR4(sc, HDMI_PHY_MASK0, (uint8_t)~PHY_MASK0_HPD);

	/* Clear hot plug interrupts */
	WR4(sc, HDMI_IH_PHY_STAT0, IH_PHY_STAT0_HPD);
};

static int
rk_hdmi_clk_enable(device_t dev)
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

		error = clk_enable(sc->clk[i]);
		if (error != 0) {
			device_printf(dev, "cannot enable '%s' clock\n",
			    clk_table[i]);
			return (ENXIO);
		}

		error = clk_get_freq(sc->clk[i], &rate);
		if (error != 0) {
			device_printf(dev, "cannot get '%s' clock frequency\n",
			    clk_table[i]);
			return (ENXIO);
		}

		device_printf(dev, "%s rate is %ld Hz\n", clk_table[i], rate);
	}

	uint32_t reg;

	reg = RD4(sc, HDMI_PHY_STAT0);
	device_printf(dev, "phy stat0 %x\n", reg);

	return (0);
}

static int
hdmi_phy_write(struct rk_hdmi_softc *sc, uint32_t addr, uint32_t data)
{
	int timeout;
	uint32_t reg;

	timeout = 10000;

	WR4(sc, HDMI_IH_I2CMPHY_STAT0, 0xff);
	WR4(sc, HDMI_PHY_I2CM_ADDRESS, addr);
	WR4(sc, HDMI_PHY_I2CM_DATAO_1, (data >> 8) & 0xff);
	WR4(sc, HDMI_PHY_I2CM_DATAO_0, data & 0xff);
	WR4(sc, HDMI_PHY_I2CM_OPERATION, PHY_I2CM_OPERATION_WR);

	do {
		reg = RD4(sc, HDMI_IH_I2CMPHY_STAT0);
		if (reg & IH_I2CMPHY_STAT0_I2CMPHYDONE) {
			/* TODO: check for IH_I2CMPHY_STAT0_I2CMPHYERROR bit */
			WR4(sc, HDMI_IH_I2CMPHY_STAT0, reg);
			return (0);
		}

		DELAY(1000);
	} while (timeout--);

	return (1);
}

static void
hdmi_phy_configure(struct rk_hdmi_softc *sc, struct display_timing *edid)
{
	uint32_t reg;
	int i;

	reg = RD4(sc, HDMI_PHY_CONF0);
	reg &= ~PHY_CONF0_TXPWRON;
	reg |= PHY_CONF0_PDDQ;
	WR4(sc, HDMI_PHY_CONF0, reg);

	reg = RD4(sc, HDMI_MC_PHYRSTZ);
	reg &= ~MC_PHYRSTZ_RST;
	WR4(sc, HDMI_MC_PHYRSTZ, reg);
	reg |= MC_PHYRSTZ_RST;
	WR4(sc, HDMI_MC_PHYRSTZ, reg);
	WR4(sc, HDMI_MC_HEACPHY_RST, MC_HEACPHY_RST);

	reg = RD4(sc, HDMI_PHY_TST0);
	reg |= PHY_TST0_TESTCLR;
	WR4(sc, HDMI_PHY_TST0, reg);
	WR4(sc, HDMI_PHY_I2CM_SLAVE, PHY_I2CM_SLAVE_PHY_GEN2);
	reg &= ~PHY_TST0_TESTCLR;
	WR4(sc, HDMI_PHY_TST0, reg);

	const struct hdmi_mpll_config *mpll_cfg;
	mpll_cfg = rockchip_mpll_cfg;

	for (i = 0; mpll_cfg[i].mpixelclock != (~0ul); i++)
		if (edid->pixelclock.typ <= mpll_cfg[i].mpixelclock)
			break;

	hdmi_phy_write(sc, PHY_OPMODE_PLLCFG, mpll_cfg[i].cpce);
	hdmi_phy_write(sc, PHY_PLLGMPCTRL, mpll_cfg[i].gmp);
	hdmi_phy_write(sc, PHY_PLLCURRCTRL, mpll_cfg[i].curr);
	hdmi_phy_write(sc, PHY_PLLPHBYCTRL, 0);
	hdmi_phy_write(sc, PHY_PLLCLKBISTPHASE, 0x6);
}

static void
hdmi_phy_set(struct rk_hdmi_softc *sc, struct display_timing *edid)
{
	uint32_t reg;
	int i;

	//edid->pixelclock.typ;

	/* Twice per HDMI spec */

	for (i = 0; i < 2; i++) {
		reg = RD4(sc, HDMI_PHY_CONF0);
		reg |= PHY_CONF0_SELDATAENPOL;
		reg &= ~PHY_CONF0_SELDIPIF;
		reg &= ~PHY_CONF0_ENTMDS;
		reg &= ~PHY_CONF0_PDZ;
		WR4(sc, HDMI_PHY_CONF0, reg);

		hdmi_phy_configure(sc, edid);
	}
}

static void
hdmi_av_composer(struct rk_hdmi_softc *sc, struct display_timing *edid)
{
	uint32_t hbl;
	uint32_t vbl;
	uint32_t reg;

	hbl = edid->hback_porch.typ + edid->hfront_porch.typ +
	    edid->hsync_len.typ;
	vbl = edid->vback_porch.typ + edid->vfront_porch.typ +
	    edid->vsync_len.typ;

	reg = FC_INVIDCONF_DE_IN_POLARITY |
	      FC_INVIDCONF_R_V_BLANK_IN_OSC |
	      FC_INVIDCONF_IN_I_P;
	if (edid->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		reg |= FC_INVIDCONF_VSYNC_IN_POLARITY;
	if (edid->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		reg |= FC_INVIDCONF_HSYNC_IN_POLARITY;
	if (edid->hdmi_monitor)
		reg |= FC_INVIDCONF_DVI_MODEZ;
	WR4(sc, HDMI_FC_INVIDCONF, reg);

	WR4(sc, HDMI_FC_INHACTIV1, edid->hactive.typ >> 8);
	WR4(sc, HDMI_FC_INHACTIV0, edid->hactive.typ);
	WR4(sc, HDMI_FC_INVACTIV1, edid->vactive.typ >> 8);
	WR4(sc, HDMI_FC_INVACTIV0, edid->vactive.typ);
	WR4(sc, HDMI_FC_INHBLANK1, hbl >> 8);
	WR4(sc, HDMI_FC_INHBLANK0, hbl);
	WR4(sc, HDMI_FC_INVBLANK, vbl);
	WR4(sc, HDMI_FC_HSYNCINDELAY1, edid->hfront_porch.typ >> 8);
	WR4(sc, HDMI_FC_HSYNCINDELAY0, edid->hfront_porch.typ);
	WR4(sc, HDMI_FC_VSYNCINDELAY, edid->vfront_porch.typ);
	WR4(sc, HDMI_FC_HSYNCINWIDTH1, edid->hsync_len.typ >> 8);
	WR4(sc, HDMI_FC_HSYNCINWIDTH0, edid->hsync_len.typ);
	WR4(sc, HDMI_FC_VSYNCINWIDTH, edid->vsync_len.typ);
}

static void
rk_hdmi_enable(device_t dev, struct display_timing *edid)
{
	struct rk_hdmi_softc *sc;

	sc = device_get_softc(dev);

	hdmi_av_composer(sc, edid);
	hdmi_phy_set(sc, edid);
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
	uint32_t reg;

	/* Select VOP Little for HDMI. */
	reg = SYSCON_READ_4(sc->grf, GRF_SOC_CON20);
	reg &= ~CON20_HDMI_VOP_SEL_M;
	reg |= CON20_HDMI_VOP_SEL_L;
	SYSCON_WRITE_4(sc->grf, GRF_SOC_CON20, reg);
}

static void
imx_hdmi_init(void *arg)
{
	struct rk_hdmi_softc *sc;
	device_t dev;

	dev = arg;

	sc = device_get_softc(dev);

	if (OF_device_from_xref(sc->i2c_xref) != NULL) {
		if (sc->eh_tag != NULL)
			EVENTHANDLER_DEREGISTER_NOWAIT(device_attach,
			    sc->eh_tag);
		printf("%s: dwc_hdmi_init\n", __func__);
		dwc_hdmi_init(dev);
		return;
	}

	//if (bootverbose)
		device_printf((device_t)dev, "Waiting for DDC i2c device\n");

	if (sc->eh_tag == NULL)
		sc->eh_tag = EVENTHANDLER_REGISTER(device_attach,
			imx_hdmi_init, dev, EVENTHANDLER_PRI_ANY);
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
	phandle_t node, i2c_xref;
	int err;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	err = syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf);
	if (err != 0) {
		device_printf(dev, "cannot get grf syscon: %d\n", err);
		return (ENXIO);
	}

#if 0
	if (bus_alloc_resources(dev, rk_hdmi_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}
#endif

	sc->base.sc_dev = dev;
	sc->base.sc_get_i2c_dev = imx_hdmi_get_i2c_dev;

	err = 0;
	/* Allocate memory resources. */
	sc->base.sc_reg_shift = 2;
	sc->base.sc_mem_rid = 0;
	sc->base.sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->base.sc_mem_rid, RF_ACTIVE);
	if (sc->base.sc_mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		err = ENXIO;
		return (ENXIO);
		//goto out;
	}   

	if (OF_getencprop(node, "ddc-i2c-bus",
	    &i2c_xref, sizeof(i2c_xref)) == -1) {
		panic("could not get ddc i2c\n");
		sc->i2c_xref = 0;
	} else
		sc->i2c_xref = i2c_xref;

	config_intrhook_oneshot(imx_hdmi_init, dev);

	return (0);

	struct display_timing *edid;

	/* TODO: read edid from a connected HDMI monitor. */
	edid = NULL;

	rk_hdmi_init(dev);
	rk_hdmi_clk_enable(dev);
	rk_hdmi_phy_init(dev);
	rk_hdmi_configure(sc);

	return (0);

	while (1) {
		rk_hdmi_read_edid(dev, 0, NULL);
		rk_hdmi_read_edid(dev, 1, NULL);
	}

	rk_hdmi_enable(dev, edid);
	rk_hdmi_phy_enable(dev, edid);

	return (0);
}

static device_method_t rk_hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_hdmi_probe),
	DEVMETHOD(device_attach,	rk_hdmi_attach),

	/* HDMI methods */
	DEVMETHOD(hdmi_get_edid,	dwc_hdmi_get_edid),
	DEVMETHOD(hdmi_set_videomode,	dwc_hdmi_set_videomode),
	DEVMETHOD_END
};

DEFINE_CLASS_1(rk_hdmi, rk_hdmi_driver, rk_hdmi_methods,
    sizeof(struct rk_hdmi_softc), simplebus_driver);

static devclass_t rk_hdmi_devclass;
EARLY_DRIVER_MODULE(rk_hdmi, simplebus, rk_hdmi_driver,
    rk_hdmi_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);
MODULE_VERSION(rk_hdmi, 1);
