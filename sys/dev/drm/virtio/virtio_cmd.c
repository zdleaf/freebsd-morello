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
#include <sys/sglist.h>
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
#include <drm/drm_bridge.h>
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

int
virtio_cmd_get_edids(struct virtio_drm_softc *sc)
{
	struct virtio_gpu_cmd_get_edid cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[2];
	struct virtio_gpu_resp_edid resp;
	struct sglist *sg;
	int rdlen;
	int error;
	int i;
	int j;

	vq = sc->ctrlq;
	virtqueue_enable_intr(vq);

	//bzero(&sg, sizeof(struct sglist));
	sg = sglist_alloc(2, M_NOWAIT);

	for (i = 0; i < sc->gpucfg.num_scanouts; i++) {
		printf("%s: getting edid for scanout %d\n", __func__, i);
		bzero(&cmd, sizeof(struct virtio_gpu_cmd_get_edid));
		bzero(&resp, sizeof(struct virtio_gpu_resp_edid));

		cmd.hdr.type = VIRTIO_GPU_CMD_GET_EDID;
#if 0
		cmd.hdr.flags = VIRTIO_GPU_FLAG_FENCE;
		cmd.hdr.flags |= VIRTIO_GPU_FLAG_INFO_RING_IDX;
		cmd.hdr.fence_id = 0;
		cmd.hdr.ring_idx = 0;
#endif
		cmd.scanout = i;

		sglist_init(sg, 2, segs);

		printf("cmd addr %p sz %d resp %d\n", &cmd,
		    sizeof(struct virtio_gpu_cmd_get_edid),
		    sizeof(struct virtio_gpu_resp_edid));

		error = sglist_append(sg, &cmd,
		    sizeof(struct virtio_gpu_cmd_get_edid) + 1);
		error = sglist_append(sg, &resp,
		    sizeof(struct virtio_gpu_resp_edid));

		printf("%s: error %d, sg->sg_nseg %d\n", __func__, error,
		    sg->sg_nseg);

		error = virtqueue_enqueue(vq, &cmd, sg, 1, 1);
		if (error) {
		}
		printf("%s: error %d\n", __func__, error);

		virtqueue_notify(vq);

		virtqueue_poll(vq, &rdlen);
		printf("%s: rdlen %d\n", __func__, rdlen);

		for (j = 0; j < 1024; j++)
			printf("%d ", resp.edid[j]);
	}

	return (0);
}

int
virtio_gpu_cmd_get_display_info(struct virtio_drm_softc *sc)
{
	struct virtio_gpu_resp_display_info resp;
	struct virtio_gpu_ctrl_hdr hdr;
	struct sglist *sg;
	struct sglist_seg segs[2];
	struct virtqueue *vq;
	int rdlen;
	int error;
	int i;

	sg = sglist_alloc(2, M_NOWAIT);

	bzero(&hdr, sizeof(struct virtio_gpu_ctrl_hdr));
	bzero(&resp, sizeof(struct virtio_gpu_resp_display_info));

	vq = sc->ctrlq;

	hdr.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

	sglist_init(sg, 2, segs);
	error = sglist_append(sg, &hdr,
	    sizeof(struct virtio_gpu_cmd_get_edid)+10000);

	printf("%s: error %d\n", __func__, error);
	error = sglist_append(sg, &resp,
	    sizeof(struct virtio_gpu_resp_display_info));
	printf("%s: error %d\n", __func__, error);

	error = virtqueue_enqueue(vq, &hdr, sg, 1, 1);
	printf("%s: error %d\n", __func__, error);

	virtqueue_notify(vq);

	virtqueue_poll(vq, &rdlen);
	printf("%s: rdlen %d\n", __func__, rdlen);

	for (i = 0; i < sc->gpucfg.num_scanouts; i++) {
		if (resp.pmodes[i].enabled) {
			printf("output %d: %dx%d+%d+%d\n", i,
			    resp.pmodes[i].r.width,
			    resp.pmodes[i].r.height,
			    resp.pmodes[i].r.x,
			    resp.pmodes[i].r.y);
		} else
			printf("output %d disabled\n", i);
	}

	return (0);
}
