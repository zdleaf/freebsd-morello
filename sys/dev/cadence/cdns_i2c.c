/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Ruslan Bukin <br@bsdpad.com>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/extres/clk/clk.h>

#include "iicbus_if.h"

struct cdns_i2c_softc {
	device_t	dev;
	struct resource	*res[2];
	struct mtx	mtx;
	int		busy;
	void *		intrhand;
	device_t	iicbus;
};

static struct ofw_compat_data compat_data[] = {
	{"cdns,i2c",	1},
	{NULL,		0}
};

static struct resource_spec cdns_i2c_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define	CDNS_I2C_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	CDNS_I2C_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	CDNS_I2C_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)
#define	CDNS_I2C_READ(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	CDNS_I2C_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static int
cdns_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{

	return (0);
}

static void
cdns_i2c_intr_locked(struct cdns_i2c_softc *sc)
{
}

static void
cdns_i2c_intr(void *arg)
{
	struct cdns_i2c_softc *sc;

	sc = (struct cdns_i2c_softc *)arg;

	CDNS_I2C_LOCK(sc);
	cdns_i2c_intr_locked(sc);
	CDNS_I2C_UNLOCK(sc);
}

static int
cdns_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct cdns_i2c_softc *sc;

	sc = device_get_softc(dev);

	CDNS_I2C_LOCK(sc);

	while (sc->busy)
		mtx_sleep(sc, &sc->mtx, 0, "i2cbuswait", 0);
	sc->busy = 1;

	sc->busy = 0;

	CDNS_I2C_UNLOCK(sc);
	return (0);
}

static int
cdns_i2c_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Cadence I2C");

	return (BUS_PROBE_DEFAULT);
}

static int
cdns_i2c_attach(device_t dev)
{
	struct cdns_i2c_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, cdns_i2c_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, cdns_i2c_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, cdns_i2c_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	clk_set_assigned(dev, ofw_bus_get_node(dev));

#if 0
	/* Activate the module clocks. */
	error = clk_get_by_ofw_name(dev, 0, "i2c", &sc->sclk);
	if (error != 0) {
		device_printf(dev, "cannot get i2c clock\n");
		goto fail;
	}
	error = clk_enable(sc->sclk);
	if (error != 0) {
		device_printf(dev, "cannot enable i2c clock\n");
		goto fail;
	}
	/* pclk clock is optional. */
	error = clk_get_by_ofw_name(dev, 0, "pclk", &sc->pclk);
	if (error != 0 && error != ENOENT) {
		device_printf(dev, "cannot get pclk clock\n");
		goto fail;
	}
	if (sc->pclk != NULL) {
		error = clk_enable(sc->pclk);
		if (error != 0) {
			device_printf(dev, "cannot enable pclk clock\n");
			goto fail;
		}
	}
#endif

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "cdns_i2c", MTX_DEF);

	bus_generic_attach(dev);

	return (0);
}

static int
cdns_i2c_detach(device_t dev)
{
	struct cdns_i2c_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);

	if (sc->iicbus != NULL)
		if ((error = device_delete_child(dev, sc->iicbus)) != 0)
			return (error);

#if 0
	if (sc->sclk != NULL)
		clk_release(sc->sclk);
	if (sc->pclk != NULL)
		clk_release(sc->pclk);
#endif

	if (sc->intrhand != NULL)
		bus_teardown_intr(sc->dev, sc->res[1], sc->intrhand);

	bus_release_resources(dev, cdns_i2c_spec, sc->res);

	mtx_destroy(&sc->mtx);

	return (0);
}

static phandle_t
cdns_i2c_get_node(device_t bus, device_t dev)
{

	return ofw_bus_get_node(bus);
}

static device_method_t cdns_i2c_methods[] = {
	DEVMETHOD(device_probe,		cdns_i2c_probe),
	DEVMETHOD(device_attach,	cdns_i2c_attach),
	DEVMETHOD(device_detach,	cdns_i2c_detach),

	/* OFW methods */
	DEVMETHOD(ofw_bus_get_node,	cdns_i2c_get_node),

	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		cdns_i2c_reset),
	DEVMETHOD(iicbus_transfer,	cdns_i2c_transfer),

	DEVMETHOD_END
};

static driver_t cdns_i2c_driver = {
	"cdns_i2c",
	cdns_i2c_methods,
	sizeof(struct cdns_i2c_softc),
};

static devclass_t cdns_i2c_devclass;

EARLY_DRIVER_MODULE(cdns_i2c, simplebus, cdns_i2c_driver, cdns_i2c_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
EARLY_DRIVER_MODULE(ofw_iicbus, cdns_i2c, ofw_iicbus_driver,
    ofw_iicbus_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_DEPEND(cdns_i2c, iicbus, 1, 1, 1);
MODULE_VERSION(cdns_i2c, 1);
