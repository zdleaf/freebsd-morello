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

#include <dev/extres/hwreset/hwreset.h>
#include <dev/extres/clk/clk.h>
#include <dev/extres/syscon/syscon.h>
#include <dev/extres/phy/phy.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <dev/drm/rockchip/rk_vop.h>

#include "rk_vop_if.h"
#if 0
#include "hdmi_if.h"
#include "syscon_if.h"
#endif

#define	VOP_READ(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define	VOP_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

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
	struct rk_vop_softc	*sc;
};

#define	CLK_NENTRIES	3
struct rk_vop_softc {
	device_t		dev;
	struct syscon		*syscon;
	struct rk_vop_conf	*phy_conf;
	clk_t			clk[CLK_NENTRIES];
	struct resource		*res[2];
	eventhandler_tag        sc_hdmi_evh;
	const struct videomode	*sc_mode;
	hwreset_t		hwreset_axi;
	hwreset_t		hwreset_ahb;
	hwreset_t		hwreset_dclk;
	struct drm_plane	planes[2];
};

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
rk_vop_mode_set(device_t dev, const struct videomode *mode)
{
	struct rk_vop_softc *sc;
	uint32_t pin_polarity;
	uint32_t mode1;
	uint32_t reg;

	sc = device_get_softc(dev);

	reg = VOP_READ(sc, RK3399_REG_CFG_DONE);
	printf("cfg done %x\n", reg);
	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 0);

	pin_polarity = (1 << DCLK_INVERT);
	if (mode->flags & VID_PHSYNC)
		pin_polarity |= 1 << HSYNC_POSITIVE;
	if (mode->flags & VID_PVSYNC)
		pin_polarity |= 1 << VSYNC_POSITIVE;
	rk_vop_set_polarity(sc, pin_polarity);

	/* Remove standby bit */
	reg = VOP_READ(sc, RK3399_SYS_CTRL);
	reg &= ~SYS_CTRL_STANDBY_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	/* Enable HDMI output only. */
	reg = VOP_READ(sc, RK3399_SYS_CTRL);
	reg &= ~SYS_CTRL_ALL_OUT_EN;
	reg |= SYS_CTRL_HDMI_OUT_EN;
	VOP_WRITE(sc, RK3399_SYS_CTRL, reg);

	printf("SYS_CTRL %x\n", VOP_READ(sc, RK3399_SYS_CTRL));

	/* Set mode */
	mode1 = 0; /* RGB888 */
	reg = VOP_READ(sc, RK3399_DSP_CTRL0);
	reg &= ~DSP_CTRL0_OUT_MODE_M;
	reg |= (mode1 << DSP_CTRL0_OUT_MODE_S);
	VOP_WRITE(sc, RK3399_DSP_CTRL0, reg);

	uint32_t hactive = mode->hdisplay;
	uint32_t vactive = mode->vdisplay;
	uint32_t hsync_len = mode->hsync_end - mode->hsync_start;
	uint32_t vsync_len = mode->vsync_end - mode->vsync_start;
	uint32_t hback_porch = mode->htotal - mode->hsync_end;
	uint32_t vback_porch = mode->vtotal - mode->vsync_end;
	uint32_t hfront_porch = mode->hsync_start - mode->hdisplay;
	uint32_t vfront_porch = mode->vsync_start - mode->vdisplay;

	reg = VOP_READ(sc, RK3399_DSP_HTOTAL_HS_END);
	reg = hsync_len;
	reg |= (hsync_len + hback_porch + hactive + hfront_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_HTOTAL_HS_END, reg);

	reg = (hsync_len + hback_porch + hactive);
	reg |= (hsync_len + hback_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_HACT_ST_END, reg);

	reg = vsync_len;
	reg |= (vsync_len + vback_porch + vactive + vfront_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_VTOTAL_VS_END, reg);

	reg = (vsync_len + vback_porch + vactive);
	reg |= (vsync_len + vback_porch) << 16;
	VOP_WRITE(sc, RK3399_DSP_VACT_ST_END, reg);

	reg = hsync_len + hback_porch + hactive;
	reg |= (hsync_len + hback_porch) << 16;
	VOP_WRITE(sc, RK3399_POST_DSP_HACT_INFO, reg);

	reg = vsync_len + vback_porch + vactive;
	reg |= (vsync_len + vback_porch) << 16;
	VOP_WRITE(sc, RK3399_POST_DSP_VACT_INFO, reg);

	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

	return (0);
}

static char * clk_table[CLK_NENTRIES] = { "aclk_vop", "dclk_vop", "hclk_vop" };

static int
rk_vop_clk_enable(device_t dev, const struct videomode *mode)
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

#if 0
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
#endif

	for (i = 0; i < CLK_NENTRIES; i++) {
		error = clk_get_by_ofw_name(dev, 0, clk_table[i], &sc->clk[i]);
		if (error != 0) {
			device_printf(dev, "cannot get '%s' clock\n",
			    clk_table[i]);
			return (ENXIO);
		}
	}

#if 1
	/* DCLK */
	error = clk_set_freq(sc->clk[1], 148500000, 0);
	if (error != 0) {
		panic("dclk fail to set");
	}

	/* ACLK */
	error = clk_set_freq(sc->clk[0], 800000000, 0);
	if (error != 0) {
		panic("aclk fail to set");
	}

	/* HCLK */
	error = clk_set_freq(sc->clk[2], 400000000, 0);
	if (error != 0) {
		panic("hclk fail to set");
	}
#endif

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

#if 0
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
#endif

	return (0);
}

static int
rk_vop_enable(device_t dev, phandle_t node, const struct videomode *mode)
{
	struct rk_vop_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	reg = (mode->hdisplay - 1);
	reg |= (mode->vdisplay - 1) << 16;
	VOP_WRITE(sc, RK3399_WIN0_ACT_INFO, reg);

	reg = (mode->hsync_end - mode->hsync_start +
		mode->htotal - mode->hsync_end);
	reg |= (mode->vsync_end - mode->vsync_start +
		mode->vtotal - mode->vsync_end) << 16;
	VOP_WRITE(sc, RK3399_WIN0_DSP_ST, reg);

	reg = (mode->hdisplay - 1);
	reg |= (mode->vdisplay - 1) << 16;
	VOP_WRITE(sc, RK3399_WIN0_DSP_INFO, reg);

	reg = VOP_READ(sc, RK3399_WIN0_COLOR_KEY);
	reg &= ~(1 << 31);
	reg &= ~(0x3fffffff);
	device_printf(dev, "color key %x\n", reg);
	VOP_WRITE(sc, RK3399_WIN0_COLOR_KEY, reg);

	uint32_t lb_mode, rgb_mode;
	int bpp;

	bpp = 32; // HDMI

	switch (bpp) {
	case 32:
		rgb_mode = ARGB8888;
		VOP_WRITE(sc, RK3399_WIN0_VIR,
		    WIN0_VIR_WIDTH_ARGB888(mode->hdisplay));
		break;
	default:
		panic("unknown bpp");
	};

	if (mode->hdisplay <= 1280)
		lb_mode = LB_RGB_1280X8;
	else if (mode->hdisplay <= 1920)
		lb_mode = LB_RGB_1920X5;
	else
		panic("unknown lb_mode: %d", mode->hdisplay);

	reg = VOP_READ(sc, RK3399_WIN0_CTRL0);
	device_printf(dev, "win0 ctrl0 %x\n", reg);
	reg &= ~WIN0_CTRL0_LB_MODE_M;
	reg &= ~WIN0_CTRL0_DATA_FMT_M;
	reg &= ~WIN0_CTRL0_EN;
	VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);

	reg |= lb_mode << WIN0_CTRL0_LB_MODE_S;
	reg |= rgb_mode << WIN0_CTRL0_DATA_FMT_S;
	reg |= WIN0_CTRL0_EN;
	VOP_WRITE(sc, RK3399_WIN0_CTRL0, reg);

	uint64_t vbase;
	uint64_t fb_base;
	uint8_t *base;
	int sz;
	sz = mode->hdisplay * mode->vdisplay * 4;
	vbase = (intptr_t)kmem_alloc_contig(sz, M_ZERO, 0, ~0, PAGE_SIZE, 0,
	    VM_MEMATTR_UNCACHEABLE);
	base = (uint8_t *)vbase;
	fb_base = (intptr_t)vtophys(vbase);

	int i;
	for (i = 0; i < 1920*1080; i += 4) {
		base[i] = 0;
		base[i + 1] = 0;
		base[i + 2] = 0xff;
		base[i + 3] = 0x00;
	}

	device_printf(dev, "fb_base %lx\n", fb_base);

	VOP_WRITE(sc, RK3399_WIN0_YRGB_MST, fb_base);
	VOP_WRITE(sc, RK3399_REG_CFG_DONE, 1);

	return (0);
}

static int
vop_mode_is_valid(const struct videomode *mode)
{

	if (mode->dot_clock != 148500)
		return (0);

	return (1);
}

static const struct videomode *
vop_pick_mode(struct edid_info *ei)
{
	const struct videomode *videomode;
	const struct videomode *m;
	int n;

	videomode = NULL;

	/*
	 * Pick a mode.
	 */
	if (ei->edid_preferred_mode != NULL) {
		if (vop_mode_is_valid(ei->edid_preferred_mode))
			videomode = ei->edid_preferred_mode;
	}

	if (videomode == NULL) {
		m = ei->edid_modes;

		sort_modes(ei->edid_modes,
		    &ei->edid_preferred_mode,
		    ei->edid_nmodes);
		for (n = 0; n < ei->edid_nmodes; n++)
			if (vop_mode_is_valid(&m[n])) {
				videomode = &m[n];
				break;
			}
	}

	return videomode;
}

#if 0
static uint8_t my_edid[128] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x4, 0x69, 0xa3, 0x28, 0xec, 0xa2, 0x2, 0x0, 0x1b, 0x18, 0x1, 0x3, 0x80, 0x3e, 0x22, 0x78, 0x3a, 0x1c, 0xb5, 0xa3, 0x57, 0x4f, 0xa0, 0x27, 0xd, 0x50, 0x54, 0xbf, 0xef, 0x0, 0xd1, 0xc0, 0x81, 0x40, 0x81, 0x80, 0x95, 0x0, 0xb3, 0x0, 0x71, 0x4f, 0x81, 0xc0, 0x1, 0x1, 0x4, 0x74, 0x0, 0x30, 0xf2, 0x70, 0x5a, 0x80, 0xb0, 0x58, 0x8a, 0x0, 0x6d, 0x55, 0x21, 0x0, 0x0, 0x1a, 0x2, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x0, 0x6d, 0x55, 0x21, 0x0, 0x0, 0x1e, 0x0, 0x0, 0x0, 0xfd, 0x0, 0x1e, 0x50, 0x18, 0xa0, 0x1e, 0x0, 0xa, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x0, 0x0, 0x0, 0xfc, 0x0, 0x41, 0x53, 0x55, 0x53, 0x20, 0x50, 0x42, 0x32, 0x38, 0x37, 0x51, 0xa, 0x20, 0x1, 0x5e};

static void
rk_vop_hdmi_event(void *arg, device_t hdmi_dev)
{
	struct rk_vop_softc *sc;
	uint8_t *edid;
	struct edid_info ei;
	const struct videomode *videomode;
	device_t dev;
	int error;

	sc = arg;

	dev = sc->dev;

	printf("%s\n", __func__);

#if 0
	uint32_t edid_len;
	edid = NULL;
	edid_len = 0;
	if (HDMI_GET_EDID(hdmi_dev, &edid, &edid_len) != 0)
		device_printf(sc->dev,
		    "failed to get EDID info from HDMI framer\n");
	else
		device_printf(sc->dev, "got edid, len %d\n", edid_len);
#endif

	edid = my_edid;

	int i;
	printf("%s: edid: \n", __func__);
	for (i = 0; i < 128; i++)
		printf("%x ", edid[i]);
	printf("\n");

	if (edid) {
		if (edid_parse(edid, &ei) == 0)
			edid_print(&ei);
		else
			printf("failed to parse EDID\n");
	}

	videomode = vop_pick_mode(&ei);
	sc->sc_mode = videomode;

	phandle_t node;
	node = ofw_bus_get_node(dev);

	error = rk_vop_clk_enable(dev, sc->sc_mode);
	if (error != 0)
		panic("error 1");

	rk_vop_mode_set(dev, sc->sc_mode);
	rk_vop_enable(dev, node, sc->sc_mode);

#if 0
	printf("config done\n");
	for (i = 0; i < 0x2000; i += 4)
		printf("reg %x %x\n", i, VOP_READ(sc, i));
#endif

	HDMI_SET_VIDEOMODE(hdmi_dev, sc->sc_mode);
}
#endif

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

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, rk_vop_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	/* There is a single port node. */
	node = ofw_bus_find_child(node, "port");
	if (node != 0) {
		printf("port node found, %d\n", node);
		OF_device_register_xref(OF_xref_from_node(node), dev);
	}

#if 0
	sc->sc_hdmi_evh = EVENTHANDLER_REGISTER(hdmi_event,
	    rk_vop_hdmi_event, sc, 0);
#endif

	return (0);
}

static int
rk_vop_commit(device_t dev)
{
	struct rk_vop_softc *sc;

	sc = device_get_softc(dev);

	printf("%s\n", __func__);

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
#if 0
	struct rk_vop_mixer_plane *mixer_plane;
	struct rk_vop_oftc *sc;
	uint32_t reg;

	mixer_plane = container_of(plane, struct rk_vop_mixer_plane, plane);
	sc = mixer_plane->sc;

	reg = AW_DE2_MIXER_READ_4(sc, OVL_UI_ATTR_CTL(mixer_plane->id));
	reg &= ~OVL_UI_ATTR_EN;
	AW_DE2_MIXER_WRITE_4(sc, OVL_UI_ATTR_CTL(mixer_plane->id), reg);
#endif
}

static void rk_vop_plane_atomic_update(struct drm_plane *plane,
					 struct drm_plane_state *old_state)
{

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

static const u32 rk_vop_plane_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static int
rk_vop_create_pipeline(device_t dev, struct drm_device *drm)
{
	struct rk_vop_softc *sc;

	sc = device_get_softc(dev);

	printf("%s\n", __func__);

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

	enum drm_plane_type type;
	int i;

	type = DRM_PLANE_TYPE_PRIMARY;

	for (i = 0; i < 2; i++) {
		if (i > 0)
			type = DRM_PLANE_TYPE_OVERLAY;

		drm_universal_plane_init(drm,
		    &sc->planes[i],
		    0,
		    &rk_vop_plane_funcs,
		    rk_vop_plane_formats,
		    nitems(rk_vop_plane_formats),
		    NULL, type, NULL);
		drm_plane_helper_add(&sc->planes[i],
		    &rk_vop_plane_helper_funcs);
	}

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
