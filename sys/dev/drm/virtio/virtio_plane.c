/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
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

#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>

#include <dev/fdt/simplebus.h>

#include <dev/drm/virtio/virtio_gpu.h>
#include <dev/drm/virtio/virtio_plane.h>
#include <dev/drm/virtio/virtio_drm.h>
#include <dev/drm/virtio/virtio_cmd.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#define	dprintf(fmt, ...)	printf(fmt, ##__VA_ARGS__)

static const u32 virtio_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_BGRA8888,
};

enum virtio_gpu_formats
virtio_convert_format(uint32_t format)
{

	switch (format) {
	case DRM_FORMAT_XRGB8888:
		return (VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM);
	case DRM_FORMAT_ARGB8888:
		return (VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM);
	case DRM_FORMAT_BGRX8888:
		return (VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM);
	case DRM_FORMAT_BGRA8888:
		return (VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM);
	default:
		return (0);
	}
}

static int
virtio_plane_atomic_check(struct drm_plane *plane,
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
virtio_plane_atomic_disable(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{

	dprintf("%s\n", __func__);
}

static void
virtio_tick(void *arg)
{
	struct virtio_drm_softc *sc;

	dprintf("%s\n", __func__);

	sc = arg;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	virtio_gpu_cmd_transfer_to_host_2d(sc, 22, sc->src_w, sc->src_h, 0, 0);
	virtio_gpu_cmd_resource_flush(sc, 22, sc->src_w, sc->src_h, 0, 0);

	callout_reset(&sc->flush_ticker, hz / 25, virtio_tick, sc);
}

static void
virtio_plane_atomic_update(struct drm_plane *plane,
    struct drm_plane_state *old_state)
{
	struct drm_plane_state *state;
	struct virtio_plane *vop_plane;
	struct virtio_drm_softc *sc;
	struct drm_gem_cma_object *bo;
	struct drm_fb_cma *fb;
	uint32_t src_w, src_h, dst_w, dst_h;
	dma_addr_t paddr;
	struct drm_crtc *crtc;
	struct drm_rect *src;
	struct drm_rect *dst;
	int id;

	dprintf("%s\n", __func__);

	state = plane->state;
	dst = &state->dst;
	src = &state->src;
	crtc = state->crtc;
	vop_plane = container_of(plane, struct virtio_plane, plane);
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

	/* TODO */
	if (!plane->state->visible) {
		while (1)
			printf("%s\n", __func__);
		panic("plane is not visible");
	}

	bo = drm_fb_cma_get_gem_obj(fb, 0);
	paddr = bo->pbase + fb->drm_fb.offsets[0];
	paddr += (state->src.x1 >> 16) * fb->drm_fb.format->cpp[0];
	paddr += (state->src.y1 >> 16) * fb->drm_fb.pitches[0];

	printf("%s: paddr %lx\n", __func__, paddr);

	struct virtio_gpu_mem_entry mem;
	mem.addr = bo->pbase;
	mem.length = bo->size;

	virtio_gpu_cmd_create_resource(sc);
	virtio_gpu_cmd_attach_backing(sc, &mem, 1);
	virtio_gpu_cmd_set_scanout(sc, 0, 22, src_w, src_h, 0, 0);
	virtio_gpu_cmd_transfer_to_host_2d(sc, 22, src_w, src_h, 0, 0);
	virtio_gpu_cmd_resource_flush(sc, 22, src_w, src_h, 0, 0);

	sc->src_w = src_w;
	sc->src_h = src_h;

	if (1 == 0)
		callout_reset(&sc->flush_ticker, hz / 25, virtio_tick, sc);
}

static struct drm_plane_helper_funcs virtio_plane_helper_funcs = {
	.atomic_check	= virtio_plane_atomic_check,
	.atomic_disable	= virtio_plane_atomic_disable,
	.atomic_update	= virtio_plane_atomic_update,
};

static const struct drm_plane_funcs virtio_plane_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.destroy		= drm_plane_cleanup,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= drm_atomic_helper_plane_reset,
	.update_plane		= drm_atomic_helper_update_plane,
};

int
virtio_plane_create(struct virtio_drm_softc *sc, struct drm_device *drm)
{
	enum drm_plane_type type;
	int error;
	int i;

	dprintf("%s\n", __func__);

	for (i = 0; i < 2; i++) {
		if (i == 0)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_CURSOR;

		error = drm_universal_plane_init(drm,
		    &sc->planes[i].plane,
		    0,
		    &virtio_plane_funcs,
		    virtio_plane_formats,
		    nitems(virtio_plane_formats),
		    NULL, type, NULL);
		if (error != 0) {
			device_printf(sc->dev, "Could not init plane.");
			return (error);
		}
		drm_plane_helper_add(&sc->planes[i].plane,
		    &virtio_plane_helper_funcs);

		sc->planes[i].sc = sc;
		sc->planes[i].id = i;
	}

	return (0);
}
