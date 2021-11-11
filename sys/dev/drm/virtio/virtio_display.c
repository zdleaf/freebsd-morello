/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020-2021 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2019 Emmanuel Vadot <manu@FreeBSD.org>
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

struct dw_hdmi_softc {
	struct drm_encoder	encoder;
	struct drm_connector	connector;
	struct drm_bridge	bridge;
	struct drm_display_mode	mode;
};

static enum drm_connector_status
dw_hdmi_connector_detect(struct drm_connector *connector, bool force)
{

	//return (connector_status_disconnected);

	return (connector_status_connected);
}

static const struct drm_connector_funcs dw_hdmi_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = dw_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int
dw_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct dw_hdmi_softc *sc;
	struct edid *edid = NULL;
	int ret = 0;

	sc = container_of(connector, struct dw_hdmi_softc, connector);

	edid = drm_get_edid(connector, sc->ddc);
	drm_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);

	return (ret);
}

static const struct drm_connector_helper_funcs
    dw_hdmi_connector_helper_funcs = {
	.get_modes = dw_hdmi_connector_get_modes,
};

static int
dw_hdmi_bridge_attach(struct drm_bridge *bridge)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	sc->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(&sc->connector,
	    &dw_hdmi_connector_helper_funcs);

	drm_connector_init(bridge->dev, &sc->connector,
	    &dw_hdmi_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(&sc->connector, &sc->encoder);

	return (0);
}

static enum drm_mode_status
dw_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
    const struct drm_display_mode *mode)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	return (MODE_OK);
}

static void
dw_hdmi_bridge_mode_set(struct drm_bridge *bridge,
    const struct drm_display_mode *orig_mode,
    const struct drm_display_mode *mode)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	/* Copy the mode, this will be set in bridge_enable function */
	memcpy(&sc->mode, mode, sizeof(struct drm_display_mode));
}

static void
dw_hdmi_bridge_disable(struct drm_bridge *bridge)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);
}

static const struct drm_bridge_funcs dw_hdmi_bridge_funcs = {
	.attach = dw_hdmi_bridge_attach,
	.enable = dw_hdmi_bridge_enable,
	.disable = dw_hdmi_bridge_disable,
	.mode_set = dw_hdmi_bridge_mode_set,
	.mode_valid = dw_hdmi_bridge_mode_valid,
};

/* Encoder funcs, belongs here */
static void aw_de2_dw_hdmi_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{
	struct aw_dw_hdmi_softc *sc;
	struct dw_hdmi_softc *base_sc;
	uint64_t freq;

	base_sc = container_of(encoder, struct dw_hdmi_softc, encoder);
	sc = container_of(base_sc, struct aw_dw_hdmi_softc, base_sc);

	clk_get_freq(sc->clk_tmds, &freq);
	DRM_DEBUG_DRIVER("%s: Setting clock %s from %ju to %ju\n",
	    __func__,
	    clk_get_name(sc->clk_tmds),
	    freq,
	    (uintmax_t)mode->crtc_clock * 1000);
	clk_set_freq(sc->clk_tmds, mode->crtc_clock * 1000, CLK_SET_ROUND_ANY);
	clk_get_freq(sc->clk_tmds, &freq);
	DRM_DEBUG_DRIVER("%s: New clock %s is %ju\n",
	    __func__,
	    clk_get_name(sc->clk_tmds),
	    freq);
}

static const struct drm_encoder_helper_funcs
    aw_de2_dw_hdmi_encoder_helper_funcs = {
	.mode_set = aw_de2_dw_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs aw_dw_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
aw_dw_hdmi_add_encoder(device_t dev, struct drm_crtc *crtc,
    struct drm_device *drm)
{
	struct aw_dw_hdmi_softc *sc;

	sc = device_get_softc(dev);

	drm_encoder_helper_add(&sc->base_sc.encoder,
	    &aw_de2_dw_hdmi_encoder_helper_funcs);
	sc->base_sc.encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->base_sc.encoder, &aw_dw_hdmi_encoder_funcs,
	  DRM_MODE_ENCODER_TMDS, NULL);

	/* This part should be in dw_hdmi */
	sc->base_sc.bridge.funcs = &dw_hdmi_bridge_funcs;
	drm_bridge_attach(&sc->base_sc.encoder, &sc->base_sc.bridge, NULL);

	return (0);
}

static void rk_dw_hdmi_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{
	struct rk_dw_hdmi_softc *sc;
	struct dw_hdmi_softc *base_sc;

	base_sc = container_of(encoder, struct dw_hdmi_softc, encoder);
	sc = container_of(base_sc, struct rk_dw_hdmi_softc, base_sc);

	/*
	 * Note: we are setting vpll, which should be the same as vop dclk.
	 */
	clk_set_freq(sc->clk[2], mode->crtc_clock * 1000, 0);
}

static const struct drm_encoder_helper_funcs rk_dw_hdmi_encoder_helper_funcs = {
	.mode_set = rk_dw_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs rk_dw_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
rk_dw_hdmi_add_encoder(device_t dev, struct drm_crtc *crtc,
    struct drm_device *drm)
{
	struct rk_dw_hdmi_softc *sc;

	sc = device_get_softc(dev);

	drm_encoder_helper_add(&sc->base_sc.encoder,
	    &rk_dw_hdmi_encoder_helper_funcs);
	sc->base_sc.encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->base_sc.encoder, &rk_dw_hdmi_encoder_funcs,
	  DRM_MODE_ENCODER_TMDS, NULL);

	/* This part should be in dw_hdmi */
	sc->base_sc.bridge.funcs = &dw_hdmi_bridge_funcs;
	drm_bridge_attach(&sc->base_sc.encoder, &sc->base_sc.bridge, NULL);

	return (0);
}
