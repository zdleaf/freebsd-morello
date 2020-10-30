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
#include <dev/gpio/gpiobusvar.h>

#include <arm64/rockchip/rk3399_mipi.h>

#include "syscon_if.h"

#define	MIPI_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	MIPI_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static struct ofw_compat_data compat_data[] = {
	{ "simple-panel-dsi",	1 },
	{ NULL,			0 }
};

#if 0
static struct resource_spec dsi_panel_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};
#endif

struct dsi_panel_softc {
	struct syscon		*syscon;
	struct dsi_panel_conf	*phy_conf;
	clk_t			ref;
	clk_t			pclk;
	clk_t			phy_cfg;
	clk_t			grf;
	struct resource		*res[2];
	struct gpiobus_pin	*gpio_rset;
	struct gpiobus_pin	*gpio_en;
};

struct cmd_ctrl_hdr {
	uint8_t dtype;	/* data type */
	uint8_t wait;	/* delay ms */
	uint8_t len;	/* payload len */
};

struct dsi_command {
	struct cmd_ctrl_hdr hdr;
	uint8_t *data;
};

#if 0
static int
dsi_panel_enable(device_t dev)
{
	struct dsi_panel_softc *sc;
	uint64_t rate_ref;
	int error;

	sc = device_get_softc(dev);

	/* aclk */
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

	device_printf(dev, "ref rate is %ld\n", rate_ref);

	return (0);
}
#endif

static int
dsi_panel_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Simple DSI panel");
	return (BUS_PROBE_DEFAULT);
}

static int
dsi_panel_attach(device_t dev)
{
	struct dsi_panel_softc *sc;

#if 0
	device_t cdev;
	phandle_t node;

	node = ofw_bus_get_node(dev);
#endif

	sc = device_get_softc(dev);

	int error;
	error = ofw_gpiobus_parse_gpios(dev, "reset-gpios", &sc->gpio_rset);
	error = ofw_gpiobus_parse_gpios(dev, "enable-gpios", &sc->gpio_en);

	/* Put panel out of reset. */

	error = GPIO_PIN_SET(sc->gpio_rset->dev, sc->gpio_rset->pin, 1);
	if (error != 0) {
		device_printf(dev, "Could not set gpio pin\n");
		return (ENXIO);
	}

	DELAY(2000);

	error = GPIO_PIN_SETFLAGS(sc->gpio_rset->dev, sc->gpio_rset->pin,
	    GPIO_PIN_OUTPUT);
	if (error != 0) {
		device_printf(dev, "Could not configure gpio pin\n");
		return (ENXIO);
	}

	DELAY(2000);

	error = GPIO_PIN_SET(sc->gpio_rset->dev, sc->gpio_rset->pin, 0);
	if (error != 0) {
		device_printf(dev, "Could not set gpio pin\n");
		return (ENXIO);
	}

	/* Enable the panel. */

	error = GPIO_PIN_SET(sc->gpio_en->dev, sc->gpio_en->pin, 0);
	if (error != 0) {
		device_printf(dev, "Could not set gpio pin\n");
		return (ENXIO);
	}

	DELAY(2000);

	error = GPIO_PIN_SETFLAGS(sc->gpio_en->dev, sc->gpio_en->pin,
	    GPIO_PIN_OUTPUT);
	if (error != 0) {
		device_printf(dev, "Could not configure gpio pin\n");
		return (ENXIO);
	}

	DELAY(2000);

	error = GPIO_PIN_SET(sc->gpio_en->dev, sc->gpio_en->pin, 1);
	if (error != 0) {
		device_printf(dev, "Could not set gpio pin\n");
		return (ENXIO);
	}

	uint8_t *seq;
	phandle_t node;
	int nitems;
	int len;

	node = ofw_bus_get_node(dev);

	if ((len = OF_getproplen(node, "panel-init-sequence")) <= 0) {
		device_printf(dev, "could not find panel init sequence");
		return (ENXIO);
	}

	printf("len %d\n", len);

	nitems = OF_getprop_alloc_multi(node, "panel-init-sequence",
	    len, (void **)&seq);
	if (nitems != 1)
		return (ENXIO);

	struct cmd_ctrl_hdr *dchdr;
	uint8_t *bufp;
	int cnt;

	bufp = seq;
	cnt = 0;

	/* Count amount of commands first */
	while (len > sizeof(*dchdr)) {
		dchdr = (struct cmd_ctrl_hdr *)bufp;
		bufp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bufp += dchdr->len;
		len -= dchdr->len;
		cnt++;
	}

	printf("cnt %d\n", cnt);

	struct dsi_command *cmds;
	struct dsi_command *cmd;
	int i;
	int j;

	cmds = malloc(sizeof(struct dsi_command) * cnt, M_DEVBUF,
	    M_WAITOK | M_ZERO);

	bufp = seq;

	for (i = 0; i < cnt; i++) {
		dchdr = (struct cmd_ctrl_hdr *)bufp;
		len -= sizeof(*dchdr);
		bufp += sizeof(*dchdr);
		cmds[i].hdr = *dchdr;
		cmds[i].data = bufp;
		bufp += dchdr->len;
		len -= dchdr->len;
	}

	for (i = 0; i < cnt; i++) {
		cmd = &cmds[i];
		printf("cmd %x, wait %x, len %d, data ",
		    cmd->hdr.dtype, cmd->hdr.wait, cmd->hdr.len);
		for (j = 0; j < cmd->hdr.len; j++)
			printf("%x ", cmd->data[j]);
		printf("\n");
	}

	return (0);
}

static device_method_t dsi_panel_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dsi_panel_probe),
	DEVMETHOD(device_attach,	dsi_panel_attach),
	DEVMETHOD_END
};

static driver_t dsi_panel_driver = {
	"dsi_panel",
	dsi_panel_methods,
	sizeof(struct dsi_panel_softc)
};

static devclass_t dsi_panel_devclass;
EARLY_DRIVER_MODULE(dsi_panel, rk_mipi, dsi_panel_driver,
    dsi_panel_devclass, 0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(dsi_panel, 1);
