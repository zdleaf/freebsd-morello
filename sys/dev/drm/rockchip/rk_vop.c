/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020-2021 Ruslan Bukin <br@bsdpad.com>
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

#include <dev/drm/rockchip/rk_vop.h>

#include "rk_vop_if.h"
#include "dw_hdmi_if.h"

#define	VOP_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	VOP_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

#define	dprintf(fmt, ...)

static const u32 rk_vop_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
};

#define	CLK_NENTRIES	3
static char * clk_table[CLK_NENTRIES] = { "aclk_vop", "dclk_vop", "hclk_vop" };

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-vop-lit",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_vop_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct rk_vop_plane {
	struct drm_plane	plane;
	struct rk_vop_softc	*sc;
	int id;
};

struct rk_vop_softc {
	device_t		dev;
	struct rk_vop_conf	*phy_conf;
	clk_t			clk[CLK_NENTRIES];
	struct resource		*res[2];
	eventhandler_tag        sc_hdmi_evh;
	const struct videomode	*sc_mode;
	hwreset_t		hwreset_axi;
	hwreset_t		hwreset_ahb;
	hwreset_t		hwreset_dclk;
	struct rk_vop_plane	planes[2];

	struct drm_pending_vblank_event	*event;
	struct drm_device		*drm;
	struct drm_crtc			crtc;
	struct drm_encoder		encoder;
	uint32_t			vbl_counter;
	device_t			outport;
	void				*intrhand;
};

static enum rockchip_data_format
vop_convert_format(uint32_t format)
{

	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return VOP_FMT_ARGB8888;
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
		return VOP_FMT_RGB888;
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
		return VOP_FMT_RGB565;
	case DRM_FORMAT_NV12:
		return VOP_FMT_YUV420SP;
	case DRM_FORMAT_NV16:
		return VOP_FMT_YUV422SP;
	case DRM_FORMAT_NV24:
		return VOP_FMT_YUV444SP;
	default:
		return (-1);
	}
}

static void
rk_vop_set_polarity(struct rk_vop_softc *sc, uint32_t pin_polarity)
{
	uint32_t reg;

	/* HDMI */
	reg = VOP_READ(sc, RK3399_DSP_CTRL1);
	reg &= ~DSP_CTRL1_HDMI_POL_M;
	reg |= pin_polarity << DSP_CTRL1_HDMI_POL_S;
	VOP_WRITE(sc, RK3399_DSP_CTRL1, reg);
}

static int
rk_vop_clk_enable(device_t dev)
{
	struct rk_vop_softc *sc;
	uint64_t rate;
	int error;
	int i;

	sc = device_get_softc(dev);

	/* Resets. */
	error = hwreset_get_by_ofw_name(sc->dev, 0, "axi", &sc->hwreset_axi);
	if (error != 0) {
		device_printf(sc->dev, "Cannot get 'axi' reset\n");
		return (ENXIO);
	}

	error = hwreset_get_by_ofw_name(sc->dev, 0, "ahb", &sc->hwreset_ahb);
	if (error != 0) {
		device_printf(sc->dev, "Cannot get 'ahb' reset\n");
		return (ENXIO);
	}

	error = hwreset_get_by_ofw_name(sc->dev, 0, "dclk", &sc->hwreset_dclk);
	if (error != 0) {
		device_printf(sc->dev, "Cannot get 'dclk' reset\n");
		return (ENXIO);
	}

	error = hwreset_assert(sc->hwreset_axi);
	if (error != 0) {
		device_printf(sc->dev, "Cannot assert 'axi' reset\n");
		return (error);
	}

	error = hwreset_assert(sc->hwreset_ahb);
	if (error != 0) {
		device_printf(sc->dev, "Cannot assert 'ahb' reset\n");
		return (error);
	}

	error = hwreset_assert(sc->hwreset_dclk);
	if (error != 0) {
		device_printf(sc->dev, "Cannot assert 'dclk' reset\n");
		return (error);
	}

	for (i = 0; i < CLK_NENTRIES; i++) {
		error = clk_get_by_ofw_name(dev, 0, clk_table[i], &sc->clk[i]);
		if (error != 0) {
			device_printf(dev, "cannot get '%s' clock\n",
			    clk_table[i]);
			return (ENXIO);
		}
	}

	/* DCLK */
	error = clk_set_freq(sc->clk[1], 148500000, 0);
	if (error != 0) {
		device_printf(sc->dev, "Failed to set dclk\n");
		return (error);
	}

	/* ACLK */
	error = clk_set_freq(sc->clk[0], 800000000, 0);
	if (error != 0) {
		device_printf(sc->dev, "Failed to set aclk\n");
		return (error);
	}

	/* HCLK */
	error = clk_set_freq(sc->clk[2], 400000000, 0);
	if (error != 0) {
		device_printf(sc->dev, "Failed to set hclk\n");
		return (error);
	}

	for (i = 0; i < CLK_NENTRIES; i++) {
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

	error = hwreset_deassert(sc->hwreset_axi);
	if (error != 0) {
		device_printf(sc->dev, "Cannot deassert 'axi' reset\n");
		return (error);
	}

	error = hwreset_deassert(sc->hwreset_ahb);
	if (error != 0) {
		device_printf(sc->dev, "Cannot deassert 'ahb' reset\n");
		return (error);
	}

	error = hwreset_deassert(sc->hwreset_dclk);
	if (error != 0) {
		device_printf(sc->dev, "Cannot deassert 'dclk' reset\n");
		return (error);
	}

	return (0);
}

static void
rk_vop_intr(void *arg)
{
	struct rk_vop_softc *sc;
	int status;

	sc = arg;

	status = VOP_READ(sc, RK3399_INTR_STATUS0);
	dprintf("%s: status0 %x\n", __func__, status);

	/* ack all the interrupts */
	VOP_WRITE(sc, RK3399_INTR_CLEAR0, ~0);

	if (status & INTR_STATUS0_FS_INTR) {
		atomic_add_32(&sc->vbl_counter, 1);
		drm_crtc_handle_vblank(&sc->crtc);
	}
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
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, rk_vop_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, rk_vop_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, rk_vop_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	/* There is a single port node. */
	node = ofw_bus_find_child(node, "port");
	if (node != 0) {
		device_printf(sc->dev, "port node found, %d\n", node);
		OF_device_register_xref(OF_xref_from_node(node), dev);
	}

	error = rk_vop_clk_enable(dev);
	if (error != 0)
		return (ENXIO);

	device_printf(sc->dev, "VOP version: %x\n", VOP_READ(sc, RK3399_VERSION_INFO));

	return (0);
}

static int
rk_vop_commit(device_t dev)
{
	struct rk_vop_softc *sc;

	sc = device_get_softc(dev);

	dprintf("%s\n", __func__);

#if 0
	AW_DE2_MIXER_WRITE_4(sc, 0x08, 1);

	if (__drm_debug & DRM_UT_DRIVER)
		rk_vop_mixer_dump_regs(sc);
#endif

	return (0);
}

static int
rk_vop_plane_atomic_check(struct drm_plane *plane,
    struct drm_plane_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;

	dprintf("%s\n", __func__);

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
rk_vop_plane_atomic_disable(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{
	//struct rk_vop_plane *plane;
	//struct rk_vop_softc *sc;

	//plane = container_of(plane, struct rk_vop_plane, plane);
	//sc = plane->sc;

	//reg = AW_DE2_MIXER_READ_4(sc, OVL_UI_ATTR_CTL(mixer_plane->id));
	//reg &= ~OVL_UI_ATTR_EN;
	//AW_DE2_MIXER_WRITE_4(sc, OVL_UI_ATTR_CTL(mixer_plane->id), reg);
}

static void
rk_vop_plane_atomic_update(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{
	struct drm_plane_state *state;
	struct rk_vop_plane *vop_plane;
	struct rk_vop_softc *sc;
	struct drm_gem_cma_object *bo;
	struct drm_fb_cma *fb;
	uint32_t src_w, src_h, dst_w, dst_h;
	dma_addr_t paddr;
	uint32_t reg;
	struct drm_crtc *crtc;
	struct drm_rect *src;
	struct drm_rect *dst;
	uint32_t dsp_stx, dsp_sty;
	int rgb_mode;
	int lb_mode;
	int id;
	int i;

	state = plane->state;
	dst = &state->dst;
	src = &state->src;
	crtc = state->crtc;
	vop_plane = container_of(plane, struct rk_vop_plane, plane);
	fb = container_of(plane->state->fb, struct drm_fb_cma, drm_fb);

	sc = vop_plane->sc;
	id = vop_plane->id;

	dprintf("%s: id %d\n", __func__, vop_plane->id);

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	dst_w = drm_rect_width(&state->dst);
	dst_h = drm_rect_height(&state->dst);

	dprintf("%s: src w %d h %d, dst w %d h %d\n",
	    __func__, src_w, src_h, dst_w, dst_h);

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

	if (id == 0)
		reg = VOP_READ(sc, RK3399_WIN0_COLOR_KEY);
	else
		reg = VOP_READ(sc, RK3399_WIN2_COLOR_KEY);
	reg &= ~(1 << 31);
	reg &= ~(0x3fffffff);
	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_COLOR_KEY, reg);
	else
		VOP_WRITE(sc, RK3399_WIN2_COLOR_KEY, reg);

	for (i = 0; i < nitems(rk_vop_plane_formats); i++)
		if (rk_vop_plane_formats[i] == state->fb->format->format)
			break;

	rgb_mode = vop_convert_format(rk_vop_plane_formats[i]);
	dprintf("fmt %d\n", rgb_mode);

	if (dst_w <= 1280)
		lb_mode = LB_RGB_1280X8;
	else if (dst_w <= 1920)
		lb_mode = LB_RGB_1920X5;
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
		reg |= (1 << 0); //ungate
		VOP_WRITE(sc, RK3399_WIN2_CTRL0, reg);
	}

	if (state->fb->format->has_alpha && id > 0) {
		VOP_WRITE(sc, RK3399_WIN2_DST_ALPHA_CTRL, (3 << 6));
		reg = (1 << 0); //SRC_ALPHA_EN
		reg |= (1 << 3); //SRC_BLEND_M0
		reg |= (1 << 5); //SRC_ALPHA_CAL_M0
		reg |= (1 << 6); //SRC_FACTOR_M0
		VOP_WRITE(sc, RK3399_WIN2_SRC_ALPHA_CTRL, reg);
	}

	bo = drm_fb_cma_get_gem_obj(fb, 0);
	paddr = bo->pbase + fb->drm_fb.offsets[0];
	paddr += (state->src.x1 >> 16) * fb->drm_fb.format->cpp[0];
	paddr += (state->src.y1 >> 16) * fb->drm_fb.pitches[0];

	if (id == 0)
		VOP_WRITE(sc, RK3399_WIN0_YRGB_MST, paddr);
	else
		VOP_WRITE(sc, RK3399_WIN2_MST0, paddr);

	//reg = VOP_READ(sc, RK3399_WIN_CTRL0(id));
	//reg |= WIN0_CTRL0_EN;
	//VOP_WRITE(sc, RK3399_WIN_CTRL0(id), reg);

	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

	dprintf("%s: NEW buf paddr %x for plane %d\n", __func__, paddr, id);
}

static struct drm_plane_helper_funcs rk_vop_plane_helper_funcs = {
	.atomic_check	= rk_vop_plane_atomic_check,
	.atomic_disable	= rk_vop_plane_atomic_disable,
	.atomic_update	= rk_vop_plane_atomic_update,
};

static const struct drm_plane_funcs rk_vop_plane_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.destroy		= drm_plane_cleanup,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= drm_atomic_helper_plane_reset,
	.update_plane		= drm_atomic_helper_update_plane,
};

/*
 * VBLANK functions
 */
static int
rk_vop_enable_vblank(struct drm_crtc *crtc)
{
	struct rk_vop_softc *sc;
	uint32_t reg;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct rk_vop_softc, crtc);

	DRM_DEBUG_DRIVER("%s: Enabling VBLANK\n", __func__);

	reg = VOP_READ(sc, RK3399_INTR_EN0);
	reg |= INTR_EN0_FS_INTR;
	reg |= (1 << 0);
	reg |= 0xffff0000;
	VOP_WRITE(sc, RK3399_INTR_EN0, reg);

	dprintf("%s: en0 %x\n", __func__,
	    VOP_READ(sc, RK3399_INTR_EN0));
	dprintf("%s: status0 %x\n", __func__,
	    VOP_READ(sc, RK3399_INTR_STATUS0));
	dprintf("%s: rstatus0 %x\n", __func__,
	    VOP_READ(sc, RK3399_INTR_RAW_STATUS0));

	return (0);
}

static void
rk_vop_disable_vblank(struct drm_crtc *crtc)
{
	struct rk_vop_softc *sc;
	uint32_t reg;

	sc = container_of(crtc, struct rk_vop_softc, crtc);

	DRM_DEBUG_DRIVER("%s: Disabling VBLANK\n", __func__);

	dprintf("%s\n", __func__);

	reg = 0xffff0000;
	VOP_WRITE(sc, RK3399_INTR_EN0, reg);
}

static uint32_t
rk_vop_get_vblank_counter(struct drm_crtc *crtc)
{
	struct rk_vop_softc *sc;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct rk_vop_softc, crtc);

	return (sc->vbl_counter);
}

static const struct drm_crtc_funcs rk_vop_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config		= drm_atomic_helper_set_config,

	.get_vblank_counter	= rk_vop_get_vblank_counter,
	.enable_vblank		= rk_vop_enable_vblank,
	.disable_vblank		= rk_vop_disable_vblank,

	.gamma_set		= drm_atomic_helper_legacy_gamma_set,
};

static int
rk_crtc_atomic_check(struct drm_crtc *crtc, struct drm_crtc_state *state)
{

	dprintf("%s\n", __func__);

	/* Not sure we need to something here, should replace with an helper */
	return (0);
}

static void
rk_crtc_atomic_begin(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	struct rk_vop_softc *sc;
	unsigned long flags;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct rk_vop_softc, crtc);

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
rk_crtc_atomic_flush(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct rk_vop_softc *sc;
	struct drm_pending_vblank_event *event;

	dprintf("%s\n", __func__);

	event = crtc->state->event;

	sc = container_of(crtc, struct rk_vop_softc, crtc);

	//AW_DE2_MIXER_COMMIT(sc->mixer);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&sc->drm->event_lock);
		/*
		 * If not in page flip, arm it for later
		 * Else send it
		 */
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&sc->drm->event_lock);
	}
}

static void
rk_crtc_atomic_enable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	uint32_t hsync_len, vsync_len;
	uint32_t hact_st, hact_end;
	uint32_t vact_st, vact_end;
	struct rk_vop_softc *sc;
	struct drm_display_mode *adj;
	uint32_t mode1;
	uint32_t reg;
	int pol;

	adj = &crtc->state->adjusted_mode;

	sc = container_of(crtc, struct rk_vop_softc, crtc);

	dprintf("%s\n", __func__);

	pol = (1 << DCLK_INVERT);
	if (adj->flags & DRM_MODE_FLAG_PHSYNC)
		pol |= (1 << HSYNC_POSITIVE);
	if (adj->flags & DRM_MODE_FLAG_PVSYNC)
		pol |= (1 << VSYNC_POSITIVE);
	rk_vop_set_polarity(sc, pol);

	/* Remove standby bit */
	reg = VOP_READ(sc, RK3399_SYS_CTRL);
	reg &= ~SYS_CTRL_STANDBY_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	/* Enable HDMI output only. */
	reg = VOP_READ(sc, RK3399_SYS_CTRL);
	reg &= ~SYS_CTRL_ALL_OUT_EN;
	reg |= SYS_CTRL_HDMI_OUT_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	dprintf("SYS_CTRL %x\n", VOP_READ(sc, RK3399_SYS_CTRL));

	/* Set mode */
	mode1 = 0; /* RGB888 */
	reg = VOP_READ(sc, RK3399_DSP_CTRL0);
	reg &= ~DSP_CTRL0_OUT_MODE_M;
	reg |= (mode1 << DSP_CTRL0_OUT_MODE_S);
	VOP_WRITE(sc, RK3399_DSP_CTRL0, reg);

	hsync_len = adj->hsync_end - adj->hsync_start;
	vsync_len = adj->vsync_end - adj->vsync_start;
	hact_st = adj->htotal - adj->hsync_start;
	hact_end = hact_st + adj->hdisplay;
	vact_st = adj->vtotal - adj->vsync_start;
	vact_end = vact_st + adj->vdisplay;

	reg = hsync_len;
	reg |= adj->htotal << 16;
	VOP_WRITE(sc, RK3399_DSP_HTOTAL_HS_END, reg);

	reg = hact_end;
	reg |= hact_st << 16;
	VOP_WRITE(sc, RK3399_DSP_HACT_ST_END, reg);
	VOP_WRITE(sc, RK3399_POST_DSP_HACT_INFO, reg);

	reg = vsync_len;
	reg |= adj->vtotal << 16;
	VOP_WRITE(sc, RK3399_DSP_VTOTAL_VS_END, reg);

	reg = vact_end;
	reg |= vact_st << 16;
	VOP_WRITE(sc, RK3399_DSP_VACT_ST_END, reg);
	VOP_WRITE(sc, RK3399_POST_DSP_VACT_INFO, reg);

	VOP_WRITE(sc, RK3399_LINE_FLAG, vact_end);
	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

	/* Enable VBLANK events */
	drm_crtc_vblank_on(crtc);
}

static void
rk_crtc_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_state)
{
	struct rk_vop_softc *sc;
	uint32_t irqflags;

	dprintf("%s\n", __func__);

	sc = container_of(crtc, struct rk_vop_softc, crtc);

#if 0
	uint32_t reg;

	/* Disable TCON */
	AW_DE2_TCON_LOCK(sc);
	reg = AW_DE2_TCON_READ_4(sc, TCON_CTL);
	reg &= ~TCON_CTL_EN;
	AW_DE2_TCON_WRITE_4(sc, TCON_CTL, reg);
	AW_DE2_TCON_UNLOCK(sc);
#endif

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
rk_crtc_mode_set_nofb(struct drm_crtc *crtc)
{

	dprintf("%s\n", __func__);
}

static const struct drm_crtc_helper_funcs rk_vop_crtc_helper_funcs = {
	.atomic_check	= rk_crtc_atomic_check,
	.atomic_begin	= rk_crtc_atomic_begin,
	.atomic_flush	= rk_crtc_atomic_flush,
	.atomic_enable	= rk_crtc_atomic_enable,
	.atomic_disable	= rk_crtc_atomic_disable,
	.mode_set_nofb	= rk_crtc_mode_set_nofb,
};

static int
rk_vop_create_pipeline(device_t dev, struct drm_device *drm)
{
	enum drm_plane_type type;
	struct rk_vop_softc *sc;
	phandle_t node;
	intptr_t xref;
	int error;
	int i;

	sc = device_get_softc(dev);

	dprintf("%s\n", __func__);

#if 0
	/* Create the different planes available */
	rk_vop_ui_plane_create(sc, drm);
	rk_vop_vi_plane_create(sc, drm);

	/* 
	 * Init the crtc
	 * UI 0 and VI are the only plane available in both mixers
	 */
	AW_DE2_TCON_CREATE_CRTC(sc->tcon, drm,
	    &sc->ui_planes[0].plane, &sc->vi_planes[0].plane);
#endif

	for (i = 0; i < 2; i++) {
		if (i == 0)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_CURSOR;

		error = drm_universal_plane_init(drm,
		    &sc->planes[i].plane,
		    0,
		    &rk_vop_plane_funcs,
		    rk_vop_plane_formats,
		    nitems(rk_vop_plane_formats),
		    NULL, type, NULL);
		if (error != 0)
			panic("could not init plane");
		drm_plane_helper_add(&sc->planes[i].plane,
		    &rk_vop_plane_helper_funcs);

		sc->planes[i].sc = sc;
		sc->planes[i].id = i;
	}

	error = drm_crtc_init_with_planes(drm, &sc->crtc, &sc->planes[0].plane,
	    &sc->planes[1].plane, &rk_vop_funcs, NULL);
	if (error != 0) {
		device_printf(sc->dev,
		    "%s: drm_crtc_init_with_planes failed\n", __func__);
		return (error);
	}

	drm_crtc_helper_add(&sc->crtc, &rk_vop_crtc_helper_funcs);

	dprintf("%s: add encoder\n", __func__);

	if ((node = OF_finddevice("/hdmi")) == -1)
		panic("could not find hdmi node");

	xref = OF_xref_from_node(node);
	sc->outport = OF_device_from_xref(xref);

	DW_HDMI_ADD_ENCODER(sc->outport, &sc->crtc, drm);

	return (0);
}

static device_method_t rk_vop_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_vop_probe),
	DEVMETHOD(device_attach,	rk_vop_attach),

	/* VOP interface */
	DEVMETHOD(rk_vop_create_pipeline,	rk_vop_create_pipeline),
	DEVMETHOD(rk_vop_commit,		rk_vop_commit),

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
