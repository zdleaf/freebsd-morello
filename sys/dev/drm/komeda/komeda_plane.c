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
#include <dev/drm/komeda/komeda_pipeline.h>
#include <dev/drm/komeda/komeda_drv.h>
#include <dev/drm/komeda/komeda_regs.h>

//#include "komeda_if.h"
//#include "dw_hdmi_if.h"

#define	dprintf(fmt, ...)

static const u32 komeda_plane_formats[] = {
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	/* Except last (WB) layer. */
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
};

static uint32_t
komeda_convert_format(uint32_t format)
{

	switch (format) {
	case DRM_FORMAT_ARGB2101010:
		return (0);
	case DRM_FORMAT_ABGR2101010:
		return (1);
	case DRM_FORMAT_RGBA1010102:
		return (2);
	case DRM_FORMAT_BGRA1010102:
		return (3);
	case DRM_FORMAT_ARGB8888:
		return (8);
	case DRM_FORMAT_ABGR8888:
		return (9);
	case DRM_FORMAT_RGBA8888:
		return (10);
	case DRM_FORMAT_BGRA8888:
		return (11);
	case DRM_FORMAT_XRGB8888:
		return (16);
	case DRM_FORMAT_XBGR8888:
		return (17);
	case DRM_FORMAT_RGBX8888:
		return (18);
	case DRM_FORMAT_BGRX8888:
		return (19);
	case DRM_FORMAT_RGB888:
		return (24);
	case DRM_FORMAT_BGR888:
		return (25);
	case DRM_FORMAT_RGBA5551:
		return (32);
	case DRM_FORMAT_ABGR1555:
		return (33);
	case DRM_FORMAT_RGB565:
		return (34);
	case DRM_FORMAT_BGR565:
		return (35);
	default:
		return (-1);
	}

	return (-1);
}

static int
komeda_plane_atomic_check(struct drm_plane *plane,
    struct drm_plane_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;

	printf("%s\n", __func__);

	crtc = state->crtc;
	if (crtc == NULL)
		return (0);

	crtc_state = drm_atomic_get_existing_crtc_state(state->state, crtc);
	if (crtc_state == NULL)
		return (-EINVAL);

	return (drm_atomic_helper_check_plane_state(state, crtc_state,
	    DRM_PLANE_HELPER_NO_SCALING,
	    DRM_PLANE_HELPER_NO_SCALING,
	    true, true));
}

static void
komeda_plane_atomic_disable(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{

	printf("%s\n", __func__);
}

static void
komeda_timing_setup(struct komeda_drm_softc *sc, struct drm_plane *plane)
{
	uint32_t hactive, hfront_porch, hback_porch, hsync_len;
	uint32_t vactive, vfront_porch, vback_porch, vsync_len;
	struct drm_plane_state *state;
	struct drm_display_mode *m;
	struct drm_crtc *crtc;
	uint32_t reg;

	state = plane->state;
	crtc = state->crtc;

	m = &crtc->state->adjusted_mode;

	hactive = m->crtc_hdisplay;
	hfront_porch = m->crtc_hsync_start - m->crtc_hdisplay;
	hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
	hback_porch = m->crtc_htotal - m->crtc_hsync_end;

	vactive = m->crtc_vdisplay;
	vfront_porch = m->crtc_vsync_start - m->crtc_vdisplay;
	vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	vback_porch = m->crtc_vtotal - m->crtc_vsync_end;

	reg = hactive << ACTIVESIZE_HACTIVE_S;
	reg |= vactive << ACTIVESIZE_VACTIVE_S;
	DPU_WR4(sc, BS_ACTIVESIZE, reg);

	reg = hfront_porch << HINTERVALS_HFRONTPORCH_S;
	reg |= hback_porch << HINTERVALS_HBACKPORCH_S;
	DPU_WR4(sc, BS_HINTERVALS, reg);

	reg = vfront_porch << VINTERVALS_VFRONTPORCH_S;
	reg |= vback_porch << VINTERVALS_VBACKPORCH_S;
	DPU_WR4(sc, BS_VINTERVALS, reg);

	reg = vsync_len << SYNC_VSYNCWIDTH_S;
	reg |= hsync_len << SYNC_HSYNCWIDTH_S;
	reg |= m->flags & DRM_MODE_FLAG_PVSYNC ? SYNC_VSP : 0;
	reg |= m->flags & DRM_MODE_FLAG_PHSYNC ? SYNC_HSP : 0;
	DPU_WR4(sc, BS_SYNC, reg);

	DPU_WR4(sc, BS_PROG_LINE, D71_DEFAULT_PREPRETCH_LINE - 1);
	DPU_WR4(sc, BS_PREFETCH_LINE, D71_DEFAULT_PREPRETCH_LINE);

	reg = BS_CONTROL_EN | BS_CONTROL_VM;
#if 0
	if (c->pipeline->dual_link) {
		DPU_WR4(sc, BS_DRIFT_TO, hfront_porch + 16);
		reg |= BS_CONTROL_DL;
	}
#endif
	DPU_WR4(sc, BS_CONTROL, reg);
}

static void
komeda_plane_atomic_update(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{
	struct komeda_drm_softc *sc;
	uint32_t src_w, src_h, dst_w, dst_h;
	struct komeda_plane *komeda_plane;
	struct drm_gem_cma_object *bo;
	struct drm_plane_state *state;
	struct drm_rect *src, *dst;
	struct drm_crtc *crtc;
	struct drm_fb_cma *fb;
	dma_addr_t paddr;
	int fmt;
	int i, id;

	printf("%s\n", __func__);

	state = plane->state;
	dst = &state->dst;
	src = &state->src;
	crtc = state->crtc;

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	dst_w = drm_rect_width(&state->dst);
	dst_h = drm_rect_height(&state->dst);

	komeda_plane = container_of(plane, struct komeda_plane, plane);
	fb = container_of(plane->state->fb, struct drm_fb_cma, drm_fb);

	sc = komeda_plane->sc;
	id = komeda_plane->id;

	for (i = 0; i < nitems(komeda_plane_formats); i++)
		if (komeda_plane_formats[i] == state->fb->format->format)
			break;

	fmt = komeda_convert_format(komeda_plane_formats[i]);
	printf("%s: fmt %d\n", __func__, fmt);

	if (state->fb->format->has_alpha && id > 0)
		printf("%s: cursor plane\n", __func__);

	bo = drm_fb_cma_get_gem_obj(fb, 0);
	paddr = bo->pbase + fb->drm_fb.offsets[0];
	paddr += (state->src.x1 >> 16) * fb->drm_fb.format->cpp[0];
	paddr += (state->src.y1 >> 16) * fb->drm_fb.pitches[0];

	printf("%s: pbase %lx, paddr %lx\n", __func__, bo->pbase, paddr);

	const struct drm_format_info *info;
	int block_h;

	info = fb->drm_fb.format;
	block_h = drm_format_info_block_height(info, 0);

	DPU_WR4(sc, LR_P0_STRIDE, fb->drm_fb.pitches[0] * block_h);
	DPU_WR8(sc, LR_P0_PTR_LOW, paddr);
	DPU_WR4(sc, LR_FORMAT, fmt);

	struct drm_display_mode *m;
	uint32_t reg;

	m = &crtc->state->adjusted_mode;
	printf("%s: adj mode hdisplay %d vdisplay %d\n", __func__,
	    m->hdisplay, m->vdisplay);

	reg = m->hdisplay << IN_SIZE_HSIZE_S | m->vdisplay << IN_SIZE_VSIZE_S;
	DPU_WR4(sc, LR_IN_SIZE, reg);
	DPU_WR4(sc, LR_CONTROL, CONTROL_EN);

	printf("%s: LPU layer 0 block info %x\n", __func__,
	    DPU_RD4(sc, LPU0_LAYER0_BLOCK_INFO));
	printf("%s: LPU layer 0 output id 0 %x\n", __func__,
	    DPU_RD4(sc, LPU0_LAYER0_OUTPUT_ID0));

	komeda_timing_setup(sc, plane);

	DPU_WR4(sc, GCU_CONTROL, CONTROL_MODE_DO0_ACTIVE);

	/* Layer 0 outputs to CU 0 */
	reg = DPU_RD4(sc, LPU0_LAYER0_OUTPUT_ID0);
	DPU_WR4(sc, CU0_CU_INPUT_ID0, reg);

	reg = m->hdisplay << 0 | m->vdisplay << 16;
	DPU_WR4(sc, CU0_INPUT0_SIZE, reg);
	DPU_WR4(sc, CU0_CU_SIZE, reg);

	/* Enable CU */
	DPU_WR4(sc, CU0_CU_CONTROL, CU_CONTROL_COPR);

	int timeout;

	timeout = 10000;

	do {
		reg = DPU_RD4(sc, GCU_CONTROL);
		if ((reg & CONTROL_MODE_M) == CONTROL_MODE_DO0_ACTIVE)
			break;
	} while (timeout--);

	if (timeout <= 0)
		printf("%s: Failed to set DO0 active\n", __func__);
	else
		printf("%s: plane initialized\n", __func__);

	/* Flush */
	DPU_WR4(sc, GCU_CONFIG_VALID0, CONFIG_VALID0_CVAL);

#if 0
	uint32_t reg;
	uint32_t dsp_stx, dsp_sty;
	int lb_mode;
	int id;
	int i;

	sc = vop_plane->sc;
	id = vop_plane->id;

	dprintf("%s: id %d\n", __func__, vop_plane->id);

	dprintf("%s: src w %d h %d, dst w %d h %d\n",
	    __func__, src_w, src_h, dst_w, dst_h);

	/* TODO */
	if (!plane->state->visible)
		panic("plane is not visible");

	/* Actual size. */
	reg = (src_w - 1);
	reg |= (src_h - 1) << 16;
	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_ACT_INFO, reg);

	dsp_stx = dst->x1 + crtc->mode.htotal - crtc->mode.hsync_start;
	dsp_sty = dst->y1 + crtc->mode.vtotal - crtc->mode.vsync_start;
	reg = dsp_sty << 16 | (dsp_stx & 0xffff);
	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_DSP_ST, reg);
	else
		VOP_WRITE(sc, RK3399_WIN2_DSP_ST0, reg);

	reg = (dst_w - 1);
	reg |= (dst_h - 1) << 16;
	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_DSP_INFO, reg);
	else
		VOP_WRITE(sc, RK3399_WIN2_DSP_INFO0, reg);

	if (dst_w <= 1280)
		lb_mode = LB_RGB_1280X8;
	else if (dst_w <= 1920)
		lb_mode = LB_RGB_1920X5;
	else if (dst_w <= 2560)
		lb_mode = LB_RGB_2560X4;
	else if (dst_w <= 3840)
		lb_mode = LB_RGB_3840X2;
	else
		panic("unknown lb_mode, dst_w %d", dst_w);

	if (id == 0) {
		VOP_WRITE(sc, RK3399_WIN0_VIR, state->fb->pitches[0] >> 2);

		reg = VOP_READ(sc, RK3399_WIN0_CTRL0);
		reg &= ~WIN0_CTRL0_LB_MODE_M;
		reg &= ~WIN0_CTRL0_DATA_FMT_M;
		reg &= ~WIN0_CTRL0_EN;
		VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);
		reg |= lb_mode << WIN0_CTRL0_LB_MODE_S;
		reg |= rgb_mode << WIN0_CTRL0_DATA_FMT_S;
		reg |= WIN0_CTRL0_EN;
		VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);
	} else {
		VOP_WRITE(sc, RK3399_WIN2_VIR0_1, state->fb->pitches[0] >> 2);

		reg = VOP_READ(sc, RK3399_WIN2_CTRL0);
		reg &= ~WIN2_CTRL0_DATA_FMT_M;
		reg &= ~WIN2_CTRL0_EN;
		VOP_WRITE(sc, RK3399_WIN2_CTRL0, reg);
		reg |= rgb_mode << WIN2_CTRL0_DATA_FMT_S;
		reg |= WIN2_CTRL0_EN;
		reg |= WIN2_CTRL0_GATE;
		VOP_WRITE(sc, RK3399_WIN2_CTRL0, reg);
	}

	/* Cursor plane alpha. */
	if (state->fb->format->has_alpha && id > 0) {
		VOP_WRITE(sc, RK3399_WIN2_DST_ALPHA_CTRL, DST_FACTOR_M0(3));

		reg = SRC_ALPHA_EN;
		reg |= 1 << SRC_BLEND_M0_S;
		reg |= SRC_ALPHA_CAL_M0;
		reg |= SRC_FACTOR_M0;
		VOP_WRITE(sc, RK3399_WIN2_SRC_ALPHA_CTRL, reg);
	}

	bo = drm_fb_cma_get_gem_obj(fb, 0);
	paddr = bo->pbase + fb->drm_fb.offsets[0];
	paddr += (state->src.x1 >> 16) * fb->drm_fb.format->cpp[0];
	paddr += (state->src.y1 >> 16) * fb->drm_fb.pitches[0];

	dprintf("%s: pbase %lx\n", __func__, bo->pbase);

	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_YRGB_MST, paddr);
	else
		VOP_WRITE(sc, RK3399_WIN2_MST0, paddr);

	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);
#endif
}

static struct drm_plane_helper_funcs komeda_plane_helper_funcs = {
	.atomic_check	= komeda_plane_atomic_check,
	.atomic_disable	= komeda_plane_atomic_disable,
	.atomic_update	= komeda_plane_atomic_update,
};

static const struct drm_plane_funcs komeda_plane_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.destroy		= drm_plane_cleanup,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= drm_atomic_helper_plane_reset,
	.update_plane		= drm_atomic_helper_update_plane,
};

int
komeda_plane_create(struct komeda_pipeline *pipeline, struct drm_device *drm)
{
	struct komeda_drm_softc *sc;
	enum drm_plane_type type;
	int error;
	int i;

	sc = pipeline->sc;

	printf("%s\n", __func__);

	for (i = 0; i < 2; i++) {
		if (i == 0)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_CURSOR;

		pipeline->planes[i].sc = sc;
		pipeline->planes[i].id = i;

		error = drm_universal_plane_init(drm,
		    &pipeline->planes[i].plane,
		    0,
		    &komeda_plane_funcs,
		    komeda_plane_formats,
		    nitems(komeda_plane_formats),
		    NULL, type, NULL);
		if (error != 0) {
			device_printf(sc->dev, "Could not init plane.");
			return (error);
		}
		drm_plane_helper_add(&pipeline->planes[i].plane,
		    &komeda_plane_helper_funcs);
	}

	return (0);
}
