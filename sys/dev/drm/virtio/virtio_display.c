/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
 *
 * Portions of this work were supported by Innovate UK project 105694,
 * "Digital Security by Design (DSbD) Technology Platform Prototype".
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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/hwreset/hwreset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

struct virtio_softc {
	struct drm_encoder	encoder;
	struct drm_connector	connector;
	struct drm_bridge	bridge;
	struct drm_display_mode	mode;
	struct i2c_adapter	*ddc;
};

static enum drm_connector_status
virtio_connector_detect(struct drm_connector *connector, bool force)
{

	//return (connector_status_disconnected);

	return (connector_status_connected);
}

static const struct drm_connector_funcs virtio_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = virtio_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int
virtio_connector_get_modes(struct drm_connector *connector)
{
	struct virtio_softc *sc;
	struct edid *edid = NULL;
	int ret = 0;

	sc = container_of(connector, struct virtio_softc, connector);

	edid = drm_get_edid(connector, sc->ddc);
	drm_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);

	return (ret);
}

static const struct drm_connector_helper_funcs
    virtio_connector_helper_funcs = {
	.get_modes = virtio_connector_get_modes,
};

static int
virtio_bridge_attach(struct drm_bridge *bridge)
{
	struct virtio_softc *sc;

	sc = container_of(bridge, struct virtio_softc, bridge);

	sc->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(&sc->connector,
	    &virtio_connector_helper_funcs);

	drm_connector_init(bridge->dev, &sc->connector,
	    &virtio_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(&sc->connector, &sc->encoder);

	return (0);
}

static enum drm_mode_status
virtio_bridge_mode_valid(struct drm_bridge *bridge,
    const struct drm_display_mode *mode)
{
	struct virtio_softc *sc;

	sc = container_of(bridge, struct virtio_softc, bridge);

	return (MODE_OK);
}

static void
virtio_bridge_mode_set(struct drm_bridge *bridge,
    const struct drm_display_mode *orig_mode,
    const struct drm_display_mode *mode)
{
	struct virtio_softc *sc;

	sc = container_of(bridge, struct virtio_softc, bridge);

	/* Copy the mode, this will be set in bridge_enable function */
	memcpy(&sc->mode, mode, sizeof(struct drm_display_mode));
}

static void
virtio_bridge_enable(struct drm_bridge *bridge)
{

	printf("%s\n", __func__);
}

static void
virtio_bridge_disable(struct drm_bridge *bridge)
{
	struct virtio_softc *sc;

	sc = container_of(bridge, struct virtio_softc, bridge);
}

static const struct drm_bridge_funcs virtio_bridge_funcs = {
	.attach = virtio_bridge_attach,
	.enable = virtio_bridge_enable,
	.disable = virtio_bridge_disable,
	.mode_set = virtio_bridge_mode_set,
	.mode_valid = virtio_bridge_mode_valid,
};

static void
virtio_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{
#if 0
	struct virtio_softc *sc;
	struct virtio_softc *base_sc;

	base_sc = container_of(encoder, struct virtio_softc, encoder);
	sc = container_of(base_sc, struct virtio_softc, base_sc);

	/*
	 * Note: we are setting vpll, which should be the same as vop dclk.
	 */
	clk_set_freq(sc->clk[2], mode->crtc_clock * 1000, 0);
#endif
}

static const struct drm_encoder_helper_funcs virtio_encoder_helper_funcs = {
	.mode_set = virtio_encoder_mode_set,
};

static const struct drm_encoder_funcs virtio_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
virtio_add_encoder(device_t dev, struct drm_crtc *crtc,
    struct drm_device *drm)
{
	struct virtio_softc *sc;

	sc = device_get_softc(dev);

	drm_encoder_helper_add(&sc->encoder, &virtio_encoder_helper_funcs);
	sc->encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->encoder, &virtio_encoder_funcs,
	    DRM_MODE_ENCODER_TMDS, NULL);

	sc->bridge.funcs = &virtio_bridge_funcs;
	drm_bridge_attach(&sc->encoder, &sc->bridge, NULL);

	return (0);
}
