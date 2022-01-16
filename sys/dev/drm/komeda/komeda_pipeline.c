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
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/eventhandler.h>
#include <sys/gpio.h>
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_graph.h>

#include <drm/drm_drv.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include <dev/extres/hwreset/hwreset.h>
#include <dev/extres/clk/clk.h>
#include <dev/extres/phy/phy.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <dev/drm/komeda/komeda_plane.h>
#include <dev/drm/komeda/komeda_drv.h>
//#include <dev/drm/komeda/komeda_pipeline.h>

//#include "komeda_pipeline_if.h"
//#include "dw_hdmi_if.h"

#define	VOP_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	VOP_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

#define	dprintf(fmt, ...)

/*
 * VBLANK functions
 */
static int
komeda_pipeline_enable_vblank(struct drm_crtc *crtc)
{

	dprintf("%s\n", __func__);

	return (0);
}

static void
komeda_pipeline_disable_vblank(struct drm_crtc *crtc)
{

	dprintf("%s\n", __func__);
}

static uint32_t
komeda_pipeline_get_vblank_counter(struct drm_crtc *crtc)
{
	struct komeda_drm_softc *sc;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct komeda_drm_softc, crtc);

	return (sc->vbl_counter);
}

static const struct drm_crtc_funcs komeda_pipeline_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config		= drm_atomic_helper_set_config,

	.get_vblank_counter	= komeda_pipeline_get_vblank_counter,
	.enable_vblank		= komeda_pipeline_enable_vblank,
	.disable_vblank		= komeda_pipeline_disable_vblank,

	.gamma_set		= drm_atomic_helper_legacy_gamma_set,
};

static int
komeda_crtc_atomic_check(struct drm_crtc *crtc, struct drm_crtc_state *state)
{

	dprintf("%s\n", __func__);

	return (0);
}

static void
komeda_crtc_atomic_begin(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	struct komeda_drm_softc *sc;
	unsigned long flags;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct komeda_drm_softc, crtc);

	if (crtc->state->event == NULL)
		return;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	if (drm_crtc_vblank_get(crtc) != 0)
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
	else
		drm_crtc_arm_vblank_event(crtc, crtc->state->event);

	crtc->state->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void
komeda_crtc_atomic_flush(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct komeda_drm_softc *sc;
	struct drm_pending_vblank_event *event;

	dprintf("%s\n", __func__);

	event = crtc->state->event;

	sc = container_of(crtc, struct komeda_drm_softc, crtc);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&sc->drm_dev.event_lock);
		/*
		 * If not in page flip, arm it for later
		 * Else send it
		 */
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&sc->drm_dev.event_lock);
	}
}

static void
komeda_crtc_atomic_enable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
#if 0
	uint32_t hsync_len, vsync_len;
	uint32_t hact_st, hact_end;
	uint32_t vact_st, vact_end;
	struct komeda_drm_softc *sc;
	struct drm_display_mode *adj;
	uint32_t mode1;
	uint32_t reg;
	int pol;

	adj = &crtc->state->adjusted_mode;

	sc = container_of(crtc, struct komeda_drm_softc, crtc);

	dprintf("%s\n", __func__);

	/* Enable VBLANK events */
	drm_crtc_vblank_on(crtc);
#endif
}

static void
komeda_crtc_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	struct komeda_drm_softc *sc;
	uint32_t irqflags;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct komeda_drm_softc, crtc);

	/* Disable VBLANK events */
	drm_crtc_vblank_off(crtc);

	spin_lock_irqsave(&crtc->dev->event_lock, irqflags);

	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}

	spin_unlock_irqrestore(&crtc->dev->event_lock, irqflags);
}

static void
komeda_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_display_mode *mode;
	struct komeda_drm_softc *sc;

	sc = container_of(crtc, struct komeda_drm_softc, crtc);
	mode = &crtc->state->adjusted_mode;

	//komeda_pipeline_clk_enable(sc->dev, mode);
}

static const struct drm_crtc_helper_funcs komeda_pipeline_crtc_helper_funcs = {
	.atomic_check	= komeda_crtc_atomic_check,
	.atomic_begin	= komeda_crtc_atomic_begin,
	.atomic_flush	= komeda_crtc_atomic_flush,
	.atomic_enable	= komeda_crtc_atomic_enable,
	.atomic_disable	= komeda_crtc_atomic_disable,
	.mode_set_nofb	= komeda_crtc_mode_set_nofb,
};

static int
komeda_pipeline_add_encoder(struct komeda_drm_softc *sc,
    struct drm_device *drm)
{

	return (0);
}

static int
komeda_pipeline_create_pipeline(device_t dev, struct drm_device *drm)
{
	struct komeda_drm_softc *sc;
	int error;

	sc = device_get_softc(dev);

	dprintf("%s\n", __func__);

	komeda_plane_create(sc, drm);

	error = drm_crtc_init_with_planes(drm, &sc->crtc, &sc->planes[0].plane,
	    &sc->planes[1].plane, &komeda_pipeline_funcs, NULL);
	if (error != 0) {
		device_printf(sc->dev,
		    "%s: drm_crtc_init_with_planes failed\n", __func__);
		return (error);
	}

	drm_crtc_helper_add(&sc->crtc, &komeda_pipeline_crtc_helper_funcs);

	error = komeda_pipeline_add_encoder(sc, drm);

	return (error);
}
