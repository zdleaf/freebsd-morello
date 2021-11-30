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

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

#include <dev/drm/virtio/virtio_gpu.h>
#include <dev/drm/virtio/virtio_plane.h>
#include <dev/drm/virtio/virtio_drm.h>

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
#if 0
	struct edid *edid = NULL;

	sc = container_of(connector, struct virtio_drm_softc, connector);

	edid = drm_get_edid(connector, sc->ddc);
	drm_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);

	return (ret);
#endif

	struct virtio_drm_softc *sc;
	int error;

	sc = container_of(connector, struct virtio_drm_softc, connector);

	error = drm_add_edid_modes(connector, sc->edids[0]);

printf("nmodes %d\n", error);

	return (error);
}

static const struct drm_connector_helper_funcs
    virtio_connector_helper_funcs = {
	.get_modes = virtio_connector_get_modes,
};

static void
virtio_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{

}

static const struct drm_encoder_helper_funcs virtio_encoder_helper_funcs = {
	.mode_set = virtio_encoder_mode_set,
};

static const struct drm_encoder_funcs virtio_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
rk_crtc_atomic_check(struct drm_crtc *crtc, struct drm_crtc_state *state)
{

	printf("%s\n", __func__);

	return (0);
}

static void
rk_crtc_atomic_begin(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{

	printf("%s\n", __func__);
}

static void
rk_crtc_atomic_flush(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{

	printf("%s\n", __func__);
}

static void
rk_crtc_atomic_enable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{

	printf("%s\n", __func__);
}

static void
rk_crtc_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{

	printf("%s\n", __func__);
}

static void
rk_crtc_mode_set_nofb(struct drm_crtc *crtc)
{

	printf("%s\n", __func__);
}

static const struct drm_crtc_helper_funcs rk_vop_crtc_helper_funcs = {
	.atomic_check   = rk_crtc_atomic_check,
	.atomic_begin   = rk_crtc_atomic_begin,
	.atomic_flush   = rk_crtc_atomic_flush,
	.atomic_enable  = rk_crtc_atomic_enable,
	.atomic_disable = rk_crtc_atomic_disable,
	.mode_set_nofb  = rk_crtc_mode_set_nofb,
};

static const struct drm_crtc_funcs rk_vop_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config		= drm_atomic_helper_set_config,
	.gamma_set		= drm_atomic_helper_legacy_gamma_set,
};

int
virtio_add_encoder(device_t dev, struct drm_crtc *crtc, struct drm_device *drm)
{
	struct virtio_drm_softc *sc;
	int error;

	sc = device_get_softc(dev);

	virtio_plane_create(sc, drm);

	error = drm_crtc_init_with_planes(drm, &sc->crtc, &sc->planes[0].plane,
	    &sc->planes[1].plane, &rk_vop_funcs, NULL);
	if (error != 0) {
		device_printf(dev, "%s: drm_crtc_init_with_planes failed\n",
		    __func__);
		return (error);
	}

	drm_crtc_helper_add(&sc->crtc, &rk_vop_crtc_helper_funcs);

	drm_encoder_helper_add(&sc->encoder, &virtio_encoder_helper_funcs);
	sc->encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->encoder, &virtio_encoder_funcs,
	    DRM_MODE_ENCODER_TMDS, NULL);

	sc->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(&sc->connector,
	    &virtio_connector_helper_funcs);
	drm_connector_init(&sc->drm_dev, &sc->connector,
	    &virtio_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_attach_encoder(&sc->connector, &sc->encoder);
	drm_connector_register(&sc->connector);

	return (0);
}
