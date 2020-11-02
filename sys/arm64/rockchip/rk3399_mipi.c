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
#include <arm64/rockchip/rk3399_mipi.h>

#include "syscon_if.h"

#define	RD4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))
#define	ARRAY_SIZE(x)		(sizeof(x) / sizeof(x[0]))

static struct display_timing ts050_timings;

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-mipi-dsi",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_mipi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define	CLK_NENTRIES	4

struct rk_mipi_softc {
	struct simplebus_softc	sc;
	struct syscon		*syscon;
	struct rk_mipi_conf	*phy_conf;
	clk_t			clk[CLK_NENTRIES];
	struct resource		*res[2];
	struct syscon		*grf;
};

static char * clk_table[CLK_NENTRIES] = { "ref", "pclk", "phy_cfg", "grf" };

static int
rk_mipi_enable(device_t dev)
{
	struct rk_mipi_softc *sc;
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

		device_printf(dev, "%s rate is %ld Hz\n", clk_table[i], rate);
	}

	return (0);
}

static void
rk_mipi_dsi_enable(device_t dev, struct display_timing *timing)
{
	struct rk_mipi_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	WR4(sc, DSI_VHSACR, timing->hsync_len.typ);
	WR4(sc, DSI_VHBPCR, timing->hback_porch.typ);
	WR4(sc, DSI_VLCR, (timing->hsync_len.typ +
	    timing->hback_porch.typ + timing->hactive.typ +
	    timing->hfront_porch.typ));
	WR4(sc, DSI_VVSACR, timing->vsync_len.typ);
	WR4(sc, DSI_VVBPCR, timing->vback_porch.typ);
	WR4(sc, DSI_VVFPCR, timing->vfront_porch.typ);
	WR4(sc, DSI_VVACR, timing->vactive.typ);

	/* Signal polarity: all high. */
	WR4(sc, DSI_LPCR, 0);

	/* Set video mode */
	reg = RD4(sc, DSI_MCR);
	reg &= ~MCR_CMDM;
	WR4(sc, DSI_MCR, reg);

	/* Burst transmittion */
	reg = RD4(sc, DSI_VMCR);
	reg &= ~VMCR_VMT_M;
	reg |= (VMCR_VMT_BRST << VMCR_VMT_S);
	WR4(sc, DSI_VMCR, reg);

	/* pix num */
	reg = (0x4b0 << VPCR_VPSIZE_S);
	WR4(sc, DSI_VPCR, reg);

	/* Set color coding to 24 bit. */
	reg = (LCOLCR_COLC_24 << LCOLCR_COLC_S);
	WR4(sc, DSI_LCOLCR, reg);

        /* Enable low power mode */
	reg = RD4(sc, DSI_VMCR);
	reg |= VMCR_LPCE;
	reg |= VMCR_LPHFPE;
	reg |= VMCR_LPVAE;
	reg |= VMCR_LPVFPE;
	reg |= VMCR_LPVBPE;
	reg |= VMCR_LPVSAE;
	WR4(sc, DSI_VMCR, reg);

	uint32_t txbyte_clk, txesc_clk;
	txbyte_clk = (timing->pixelclock.typ * 6) / 8;
	txesc_clk = 20000000;

	reg = RD4(sc, DSI_CCR);
	reg &= ~CCR_TOCKDIV_M;
	reg &= ~CCR_TXECKDIV_M;
	reg |= 0x0a << CCR_TOCKDIV_S;
	reg |= (txbyte_clk / txesc_clk) << CCR_TXECKDIV_S;
	WR4(sc, DSI_CCR, reg);

        /* Timeout count for hs<->lp transation between Line period */
	reg = RD4(sc, DSI_TCCR0);
	reg &= ~(0xffff << 16);
	reg |= 0x3e8 << 16;
	WR4(sc, DSI_TCCR0, reg);

        /* Phy State transfer timing */
	reg = RD4(sc, DSI_PCONFR);
	reg &= ~SW_TIME_M;
	reg |= (32 << SW_TIME_S); /* Stop Wait Time */
	WR4(sc, DSI_PCONFR, reg);

	reg = RD4(sc, DSI_CLCR);
	reg |= CLCR_DPCC; /* D-PHY Clock Control */
	WR4(sc, DSI_CLCR, reg);

	reg = RD4(sc, DSI_CLTCR);
	reg &= ~(0xff << 24);
	reg |= 0x14 << 24; /* PHY_HS2LP_TIME */
	reg &= ~(0xff << 16);
	reg |= 0x10 << 16; /* PHY_LP2HS_TIME */
	reg &= ~(0x7fff << 0);
	reg |= 0x2710 << 0; /* MAX_RD_TIME */
	WR4(sc, DSI_CLTCR, reg);

	/* Enable the DSI */
	reg = RD4(sc, DSI_CR);
	reg |= CR_EN;
	WR4(sc, DSI_CR, reg);
}

static void
rk_mipi_phy_write(struct rk_mipi_softc *sc, uint8_t test_code,
    uint8_t *test_data, uint8_t size)
{
	uint32_t reg;
	int i;

	/* Write Test code */
	reg = RD4(sc, DSI_PHY_TST_CTRL0);
	reg |= TST_CTRL0_TESTCLK;
	WR4(sc, DSI_PHY_TST_CTRL0, reg);

	reg = RD4(sc, DSI_PHY_TST_CTRL1);
	reg &= ~TST_CTRL1_TESTDIN_M;
	reg |= test_code << TST_CTRL1_TESTDIN_S;
	WR4(sc, DSI_PHY_TST_CTRL1, reg);
	reg |= TST_CTRL1_TESTEN;
	WR4(sc, DSI_PHY_TST_CTRL1, reg);

	reg = RD4(sc, DSI_PHY_TST_CTRL0);
	reg &= ~TST_CTRL0_TESTCLK;
	WR4(sc, DSI_PHY_TST_CTRL0, reg);

	reg = RD4(sc, DSI_PHY_TST_CTRL1);
	reg &= ~TST_CTRL1_TESTEN;
	WR4(sc, DSI_PHY_TST_CTRL1, reg);

	/* Write Test data */
	for (i = 0; i < size; i++) {
		reg = RD4(sc, DSI_PHY_TST_CTRL0);
		reg &= ~TST_CTRL0_TESTCLK;
		WR4(sc, DSI_PHY_TST_CTRL0, reg);

		reg = RD4(sc, DSI_PHY_TST_CTRL1);
		reg &= ~TST_CTRL1_TESTDIN_M;
		reg |= test_data[i] << TST_CTRL1_TESTDIN_S;
		WR4(sc, DSI_PHY_TST_CTRL1, reg);

		reg = RD4(sc, DSI_PHY_TST_CTRL0);
		reg |= TST_CTRL0_TESTCLK;
		WR4(sc, DSI_PHY_TST_CTRL0, reg);
	}
}

static void
rk_mipi_phy_enable(device_t dev, struct display_timing *timing)
{
	struct rk_mipi_softc *sc;
	uint8_t test_data[2];
	uint32_t max_fbdiv;
	uint32_t ddr_clk;
	uint32_t ref_clk;
	uint32_t remain;
	uint32_t prediv;
	uint32_t reg;
	uint64_t fbdiv;
	int i;

	sc = device_get_softc(dev);

	ddr_clk = (timing->pixelclock.typ * 6);
	ref_clk = 24000000;
	remain = ref_clk;
	max_fbdiv = 512;
	prediv = 1;

	int freq_rang[][2] = {
		{90, 0x01},   {100, 0x10},  {110, 0x20},  {130, 0x01},
		{140, 0x11},  {150, 0x21},  {170, 0x02},  {180, 0x12},
		{200, 0x22},  {220, 0x03},  {240, 0x13},  {250, 0x23},
		{270, 0x04},  {300, 0x14},  {330, 0x05},  {360, 0x15},
		{400, 0x25},  {450, 0x06},  {500, 0x16},  {550, 0x07},
		{600, 0x17},  {650, 0x08},  {700, 0x18},  {750, 0x09},
		{800, 0x19},  {850, 0x29},  {900, 0x39},  {950, 0x0a},
		{1000, 0x1a}, {1050, 0x2a}, {1100, 0x3a}, {1150, 0x0b},
		{1200, 0x1b}, {1250, 0x2b}, {1300, 0x3b}, {1350, 0x0c},
		{1400, 0x1c}, {1450, 0x2c}, {1500, 0x3c}
        };

	/* Shutdown mode */
	reg = RD4(sc, DSI_PCTLR);
	reg &= ~PCTLR_UNSHUTDOWN;
	reg &= ~PCTLR_DEN;
	WR4(sc, DSI_PCTLR, reg);

	/* PLL locking */
	reg = RD4(sc, DSI_PHY_TST_CTRL0);
	reg |= TST_CTRL0_TESTCLR;
	WR4(sc, DSI_PHY_TST_CTRL0, reg);
	reg &= ~TST_CTRL0_TESTCLR;
	WR4(sc, DSI_PHY_TST_CTRL0, reg);

	test_data[0] = 0x80 | (ddr_clk / (200000000)) << 3 | 0x3;
	rk_mipi_phy_write(sc, CODE_PLL_VCORANGE_VCOCAP, test_data, 1);

	test_data[0] = 0x8;
	rk_mipi_phy_write(sc, CODE_PLL_CPCTRL, test_data, 1);

	test_data[0] = 0x80 | 0x40;
	rk_mipi_phy_write(sc, CODE_PLL_LPF_CP, test_data, 1);

	/* Select a suitable value for fsfreqrang reg */

	for (i = 0; i < ARRAY_SIZE(freq_rang); i++) {
		if (ddr_clk / 1000000 <= freq_rang[i][0])
			break;
	}

	if (i == ARRAY_SIZE(freq_rang))
		panic("out of range\n");

	test_data[0] = freq_rang[i][1] << 1;
	rk_mipi_phy_write(sc, CODE_HS_RX_LANE0, test_data, 1);

	uint32_t max_prediv, min_prediv;

	max_prediv = (ref_clk / (5 * 1000000));
	min_prediv = ((ref_clk / (40 * 1000000)) ?
	    (ref_clk / (40 * 1000000) + 1) : 1);

	if (max_prediv < min_prediv)
		panic("Invalid ref_clk\n");

	for (i = min_prediv; i < max_prediv; i++) {
		if ((ddr_clk * i % ref_clk < remain) &&
		    (ddr_clk * i / ref_clk) < max_fbdiv) {
			prediv = i;
			remain = ddr_clk * i % ref_clk;
		}
	}
	fbdiv = ddr_clk * prediv / ref_clk;
	ddr_clk = ref_clk * fbdiv / prediv;

	device_printf(dev, "ref_clk=%u, prediv=%u, fbdiv=%lu, phyclk=%u\n",
	    ref_clk, prediv, fbdiv, ddr_clk);

	/* config prediv and feedback reg */
	test_data[0] = prediv - 1;
	rk_mipi_phy_write(sc, CODE_PLL_INPUT_DIV_RAT, test_data, 1);
	test_data[0] = (fbdiv - 1) & 0x1f;
	rk_mipi_phy_write(sc, CODE_PLL_LOOP_DIV_RAT, test_data, 1);
	test_data[0] = (fbdiv - 1) >> 5 | 0x80;
	rk_mipi_phy_write(sc, CODE_PLL_LOOP_DIV_RAT, test_data, 1);
	test_data[0] = 0x30;
	rk_mipi_phy_write(sc, CODE_PLL_INPUT_LOOP_DIV_RAT, test_data, 1);

	/* rest config */
	test_data[0] = 0x4d;
	rk_mipi_phy_write(sc, CODE_BANDGAP_BIAS_CTRL, test_data, 1);

	test_data[0] = 0x3d;
	rk_mipi_phy_write(sc, CODE_TERMINATION_CTRL, test_data, 1);

	test_data[0] = 0xdf;
	rk_mipi_phy_write(sc, CODE_TERMINATION_CTRL, test_data, 1);

	test_data[0] =  0x7;
	rk_mipi_phy_write(sc, CODE_AFE_BIAS_BANDGAP_ANOLOG, test_data, 1);

	test_data[0] = 0x80 | 0x7;
	rk_mipi_phy_write(sc, CODE_AFE_BIAS_BANDGAP_ANOLOG, test_data, 1);

	test_data[0] = 0x80 | 15;
	rk_mipi_phy_write(sc, CODE_HSTXDATALANEREQUSETSTATETIME,
			  test_data, 1);
	test_data[0] = 0x80 | 85;
	rk_mipi_phy_write(sc, CODE_HSTXDATALANEPREPARESTATETIME,
			  test_data, 1);
	test_data[0] = 0x40 | 10;
	rk_mipi_phy_write(sc, CODE_HSTXDATALANEHSZEROSTATETIME,
			  test_data, 1);

	/* enter into stop mode */
	reg = RD4(sc, DSI_PCONFR);
	reg &= ~PCONFR_NL_M;
	reg |= PCONFR_NL_4;
	WR4(sc, DSI_PCONFR, reg);

	reg = RD4(sc, DSI_PCTLR);
	reg |= PCTLR_FORCEPLL | PCTLR_CKE | PCTLR_DEN | PCTLR_UNSHUTDOWN;
	WR4(sc, DSI_PCTLR, reg);
}

static void
rk_mipi_configure(struct rk_mipi_softc *sc)
{
	uint32_t reg;

	/* Select VOP Little for MIPI DSI. */
	reg = SYSCON_READ_4(sc->grf, GRF_SOC_CON20);
	reg &= ~CON20_DSI0_VOP_SEL_M;
	reg |= CON20_DSI0_VOP_SEL_L;
	SYSCON_WRITE_4(sc->grf, GRF_SOC_CON20, reg);

	reg = SYSCON_READ_4(sc->grf, GRF_SOC_CON22);
	/* Configure in TX mode */
	reg &= ~CON22_DPHY_TX0_RXMODE_M;
	reg |= CON22_DPHY_TX0_RXMODE_DIS;
	/* Disable stop mode */
	reg &= ~CON22_DPHY_TX0_TXSTOPMODE_M;
	reg |= CON22_DPHY_TX0_TXSTOPMODE_DIS;
	/* Disable turnequest */
	reg &= ~CON22_DPHY_TX0_TURNREQUEST_M;
	reg |= CON22_DPHY_TX0_TURNREQUEST_DIS;
	SYSCON_WRITE_4(sc->grf, GRF_SOC_CON22, reg);
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
	int err;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	err = syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf);
	if (err != 0) {
		device_printf(dev, "cannot get grf syscon: %d\n", err);
		return (ENXIO);
	}

	if (bus_alloc_resources(dev, rk_mipi_spec, sc->res) != 0) {
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

	rk_mipi_enable(dev);
	rk_mipi_configure(sc);
	rk_mipi_dsi_enable(dev, edid);
	rk_mipi_phy_enable(dev, edid);

	simplebus_init(dev, node);
#if 0
	if (simplebus_fill_ranges(node, &sc->sc) < 0) {
		device_printf(dev, "could not get ranges\n");
		return (ENXIO);
	}
#endif

	device_printf(dev, "DSI version: %x\n", RD4(sc, DSI_VR));

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
