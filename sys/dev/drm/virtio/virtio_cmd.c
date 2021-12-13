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
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>

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

static int
virtio_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	struct virtio_gpu_resp_edid *resp;
	size_t start;

	resp = data;

	start = block * EDID_LENGTH;
	if (start + len > resp->size)
		return (-1);

	memcpy(buf, resp->edid + start, len);

	return 0;
}

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

#if 1
		struct edid *new_edid;
		new_edid = drm_do_get_edid(&sc->connector,
		    virtio_get_edid_block, &resp);
		drm_connector_update_edid_property(&sc->connector, new_edid);
		sc->edids[i] = new_edid;
#endif
	}

	sglist_free(sg);

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

printf("%s\n", __func__);

	bzero(&hdr, sizeof(struct virtio_gpu_ctrl_hdr));
	bzero(&resp, sizeof(struct virtio_gpu_resp_display_info));

	vq = sc->ctrlq;

	hdr.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

	sg = sglist_alloc(2, M_NOWAIT);
	sglist_init(sg, 2, segs);

	error = sglist_append(sg, &hdr,
	    sizeof(struct virtio_gpu_cmd_get_edid));
	if (error)
		return (error);

	error = sglist_append(sg, &resp,
	    sizeof(struct virtio_gpu_resp_display_info));
	if (error)
		return (error);

	error = virtqueue_enqueue(vq, &hdr, sg, 1, 1);
	if (error)
		return (error);

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

	sglist_free(sg);

	return (0);
}

int
virtio_gpu_cmd_create_resource(struct virtio_drm_softc *sc)
{
	struct virtio_gpu_resource_create_2d cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[1];
	struct sglist *sg;
	int rdlen;
	int error;

	bzero(&cmd, sizeof(struct virtio_gpu_resource_create_2d));

printf("%s\n", __func__);

	vq = sc->ctrlq;

	cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
	cmd.resource_id = 22;
	cmd.format = virtio_convert_format(DRM_FORMAT_XRGB8888);
	cmd.width = 1024;
	cmd.height = 768;

	sg = sglist_alloc(1, M_NOWAIT);
	sglist_init(sg, 1, segs);

	error = sglist_append(sg, &cmd,
	    sizeof(struct virtio_gpu_resource_create_2d));
	if (error)
		return (error);

	error = virtqueue_enqueue(vq, &cmd, sg, 1, 0);
	if (error)
		return (error);

	virtqueue_notify(vq);

	virtqueue_poll(vq, &rdlen);

	printf("%s: rdlen %d\n", __func__, rdlen);

	sglist_free(sg);

	return (0);
}

int
virtio_gpu_cmd_attach_backing(struct virtio_drm_softc *sc,
    struct virtio_gpu_mem_entry *mem, int nitems)
{
	struct virtio_gpu_resource_attach_backing cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[2];
	struct sglist *sg;
	int rdlen;
	int error;

printf("%s\n", __func__);

	bzero(&cmd, sizeof(struct virtio_gpu_resource_attach_backing));

	vq = sc->ctrlq;

	cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
	cmd.resource_id = 22;
	cmd.nr_entries = 1;

	sg = sglist_alloc(2, M_NOWAIT);
	sglist_init(sg, 2, segs);

	error = sglist_append(sg, &cmd,
	    sizeof(struct virtio_gpu_resource_attach_backing));
	if (error)
		return (error);

	error = sglist_append(sg, mem, sizeof(struct virtio_gpu_mem_entry));
	if (error)
		return (error);

	error = virtqueue_enqueue(vq, &cmd, sg, 2, 0);
	if (error)
		return (error);

	virtqueue_notify(vq);
	virtqueue_poll(vq, &rdlen);

	printf("%s: rdlen %d\n", __func__, rdlen);
	sglist_free(sg);

	return (0);
}

int
virtio_gpu_cmd_set_scanout(struct virtio_drm_softc *sc, uint32_t scanout_id,
    uint32_t resource_id, uint32_t width, uint32_t height, uint32_t x,
    uint32_t y)
{
	struct virtio_gpu_set_scanout cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[1];
	struct sglist *sg;
	int rdlen;
	int error;

printf("%s\n", __func__);

	bzero(&cmd, sizeof(struct virtio_gpu_resource_attach_backing));
	vq = sc->ctrlq;

	cmd.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
	cmd.resource_id = resource_id;
	cmd.scanout_id = scanout_id;
	cmd.r.width = width;
	cmd.r.height = height;
	cmd.r.x = x;
	cmd.r.y = y;

	sg = sglist_alloc(1, M_NOWAIT);
	sglist_init(sg, 1, segs);
	error = sglist_append(sg, &cmd,
	    sizeof(struct virtio_gpu_set_scanout));

	error = virtqueue_enqueue(vq, &cmd, sg, 1, 0);
	if (error)
		return (error);

	virtqueue_notify(vq);
	virtqueue_poll(vq, &rdlen);

	printf("%s: rdlen %d\n", __func__, rdlen);
	sglist_free(sg);

	return (0);
}

int
virtio_gpu_cmd_transfer_to_host_2d(struct virtio_drm_softc *sc,
    uint32_t resource_id, uint32_t width, uint32_t height, uint32_t x,
    uint32_t y)
{
	struct virtio_gpu_transfer_to_host_2d cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[1];
	struct sglist *sg;
	int rdlen;
	int error;

printf("%s\n", __func__);

	bzero(&cmd, sizeof(struct virtio_gpu_transfer_to_host_2d));
	vq = sc->ctrlq;

	cmd.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
	cmd.resource_id = resource_id;
	cmd.offset = 0;
	cmd.r.width = width;
	cmd.r.height = height;
	cmd.r.x = x;
	cmd.r.y = y;

	sg = sglist_alloc(1, M_NOWAIT);
	sglist_init(sg, 1, segs);
	error = sglist_append(sg, &cmd,
	    sizeof(struct virtio_gpu_transfer_to_host_2d));

	error = virtqueue_enqueue(vq, &cmd, sg, 1, 0);
	if (error)
		return (error);

	virtqueue_notify(vq);
	virtqueue_poll(vq, &rdlen);

	//printf("%s: rdlen %d\n", __func__, rdlen);
	sglist_free(sg);

	return (0);
}

int
virtio_gpu_cmd_resource_flush(struct virtio_drm_softc *sc,
    uint32_t resource_id, uint32_t width, uint32_t height, uint32_t x,
    uint32_t y)
{
	struct virtio_gpu_resource_flush cmd;
	struct virtqueue *vq;
	struct sglist_seg segs[1];
	struct sglist *sg;
	int rdlen;
	int error;

printf("%s\n", __func__);
	bzero(&cmd, sizeof(struct virtio_gpu_resource_flush));
	vq = sc->ctrlq;

	cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
	cmd.resource_id = resource_id;
	cmd.r.width = width;
	cmd.r.height = height;
	cmd.r.x = x;
	cmd.r.y = y;

	sg = sglist_alloc(1, M_NOWAIT);
	sglist_init(sg, 1, segs);
	error = sglist_append(sg, &cmd,
	    sizeof(struct virtio_gpu_resource_flush));

	error = virtqueue_enqueue(vq, &cmd, sg, 1, 0);
	if (error)
		return (error);

	virtqueue_notify(vq);
	virtqueue_poll(vq, &rdlen);

	//printf("%s: rdlen %d\n", __func__, rdlen);
	sglist_free(sg);

	return (0);
}
