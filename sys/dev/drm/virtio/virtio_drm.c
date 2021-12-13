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
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sglist.h>
#include <sys/stdatomic.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>

#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>

#include <dev/drm/virtio/virtio_plane.h>
#include <dev/drm/virtio/virtio_gpu.h>
#include <dev/drm/virtio/virtio_drm.h>
#include <dev/drm/virtio/virtio_cmd.h>
#include <dev/drm/virtio/virtgpu_drm.h>

#include <dev/drm/drmkpi/include/linux/dma-buf.h>

static int	vtgpu_modevent(module_t, int, void *);

static int	vtgpu_probe(device_t);
static int	vtgpu_attach(device_t);
static int	vtgpu_detach(device_t);

static int	vtgpu_negotiate_features(struct virtio_drm_softc *);
static int	vtgpu_setup_features(struct virtio_drm_softc *);
static int	vtgpu_alloc_virtqueue(struct virtio_drm_softc *);

#define VTGPU_FEATURES	(1 << VIRTIO_GPU_F_VIRGL)		|\
			(1 << VIRTIO_GPU_F_EDID)		|\
			(1 << VIRTIO_GPU_F_RESOURCE_UUID)	|\
			(1 << VIRTIO_GPU_F_RESOURCE_BLOB)	|\
			(1 << VIRTIO_GPU_F_CONTEXT_INIT)	|\
			(VIRTIO_RING_F_INDIRECT_DESC)

static struct virtio_feature_desc vtgpu_feature_desc[] = {
	{ 0, NULL }
};

/* Kludge for API limitations of random(4). */
static _Atomic(struct virtio_drm_softc *) g_virtio_drm_softc;

static device_method_t vtgpu_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		vtgpu_probe),
	DEVMETHOD(device_attach,	vtgpu_attach),
	DEVMETHOD(device_detach,	vtgpu_detach),

	DEVMETHOD_END
};

static driver_t vtgpu_driver = {
	"vtgpu",
	vtgpu_methods,
	sizeof(struct virtio_drm_softc)
};
static devclass_t vtgpu_devclass;

VIRTIO_DRIVER_MODULE(virtio_gpu, vtgpu_driver, vtgpu_devclass,
    vtgpu_modevent, 0);
MODULE_VERSION(virtio_gpu, 1);
MODULE_DEPEND(virtio_gpu, virtio, 1, 1, 1);
MODULE_DEPEND(virtio_gpu, random_device, 1, 1, 1);

VIRTIO_SIMPLE_PNPINFO(virtio_gpu, VIRTIO_ID_GPU,
    "VirtIO GPU Adapter");

static int
vtgpu_modevent(module_t mod, int type, void *unused)
{
	int error;

	switch (type) {
	case MOD_LOAD:
	case MOD_QUIESCE:
	case MOD_UNLOAD:
	case MOD_SHUTDOWN:
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static int
virtio_gem_mmap_buf(struct drm_gem_object *obj, struct vm_area_struct *vma)
{

	while (1)
		printf("%s\n", __func__);
	panic("implement me");

	return (0);
}

struct sg_table *
virtio_gem_prime_get_sg_table(struct drm_gem_object *obj)
{

	while (1)
		printf("%s\n", __func__);
	panic("implement me");

	return (NULL);
}

struct drm_gem_object *
virtio_gem_prime_import_sg_table(struct drm_device *dev,
    struct dma_buf_attachment *attach, struct sg_table *sg)
{

	while (1)
		printf("%s\n", __func__);
	panic("implement me");

	return (NULL);
}

/* DRM driver fops */
static const struct file_operations virtio_drm_drv_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.kqfilter = drm_kqfilter,
	/* .llseek = noop_llseek, */
	.mmap	= drm_gem_mmap,
};

static int
virtio_ioctl_map(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_execbuffer(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_getparam(struct drm_device *dev, void *data,
    struct drm_file *file)
{
	struct drm_virtgpu_getparam *param;
	struct virtio_drm_softc *sc;

	sc = dev->dev_private;
	param = data;

	printf("%s: param %d\n", __func__, param->param);

	switch (param->param) {
	case VIRTGPU_PARAM_3D_FEATURES:
		param->value = sc->has_virgl_3d ? 1 : 0;
		break;
	case VIRTGPU_PARAM_CAPSET_QUERY_FIX:
		param->value = 1;
		break;
	case VIRTGPU_PARAM_RESOURCE_BLOB:
		param->value = sc->has_resource_blob ? 1 : 0;
		break;
	case VIRTGPU_PARAM_HOST_VISIBLE:
		param->value = sc->has_host_visible ? 1 : 0;
		break;
	case VIRTGPU_PARAM_CROSS_DEVICE:
		param->value = sc->has_resource_assign_uuid ? 1 : 0;
		break;
	case VIRTGPU_PARAM_CONTEXT_INIT:
		param->value = sc->has_context_init ? 1 : 0;
		break;
	case VIRTGPU_PARAM_SUPPORTED_CAPSET_IDs:
		param->value = sc->capset_id_mask;
		break;
	default:
		return (EINVAL);
	}

	return (0);
}

static int
virtio_ioctl_resource_create(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_resource_info(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_transfer_from(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_transfer_to(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_wait(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_get_caps(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_resource_create_blob(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
virtio_ioctl_context_init(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static const struct drm_ioctl_desc virtio_gpu_ioctls[] = {
#define VCTL(name, func, flags) \
	DRM_IOCTL_DEF_DRV(VIRTGPU_##name, virtio_ioctl_##func, flags)

	VCTL(MAP,			map,		DRM_RENDER_ALLOW),
	VCTL(EXECBUFFER,		execbuffer,	DRM_RENDER_ALLOW),
	VCTL(GETPARAM,			getparam,	DRM_RENDER_ALLOW),
	VCTL(RESOURCE_CREATE,		resource_create,DRM_RENDER_ALLOW),
	VCTL(RESOURCE_INFO,		resource_info,	DRM_RENDER_ALLOW),
	VCTL(TRANSFER_FROM_HOST,	transfer_from,	DRM_RENDER_ALLOW),
	VCTL(TRANSFER_TO_HOST,		transfer_to,	DRM_RENDER_ALLOW),
	VCTL(WAIT,			wait,		DRM_RENDER_ALLOW),
	VCTL(GET_CAPS,			get_caps,	DRM_RENDER_ALLOW),
	VCTL(RESOURCE_CREATE_BLOB,	resource_create_blob, DRM_RENDER_ALLOW),
	VCTL(CONTEXT_INIT,		context_init,	DRM_RENDER_ALLOW),
};

static int
virtio_open(struct drm_device *dev, struct drm_file *file)
{

	printf("%s\n", __func__);
	return (EINVAL);
}

static void
virtio_postclose(struct drm_device *dev, struct drm_file *file)
{

	printf("%s\n", __func__);
}

static struct drm_driver virtio_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | \
	    DRIVER_ATOMIC | DRIVER_PRIME | DRIVER_SYNCOBJ | DRIVER_RENDER,

	/* Generic Operations */
	.lastclose = drm_fb_helper_lastclose,
	.fops = &virtio_drm_drv_fops,

	/* GEM Opeations */
	.dumb_create			= drm_gem_cma_dumb_create,
	//.gem_free_object		= drm_gem_cma_free_object,
	//.gem_vm_ops			= &drm_gem_cma_vm_ops,

	.prime_handle_to_fd		= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle		= drm_gem_prime_fd_to_handle,
	.gem_prime_get_sg_table		= virtio_gem_prime_get_sg_table,
	.gem_prime_import_sg_table	= virtio_gem_prime_import_sg_table,
	.gem_prime_mmap			= virtio_gem_mmap_buf,

	.ioctls				= virtio_gpu_ioctls,
	.num_ioctls			= ARRAY_SIZE(virtio_gpu_ioctls),

	.name				= "virtio_gpu",
	.desc				= "Virtio Display Subsystem",
	.date				= "20211109",
	.major				= 0,
	.minor				= 1,

	.open				= virtio_open,
	.postclose			= virtio_postclose,
};

static void
virtio_drm_output_poll_changed(struct drm_device *drm_dev)
{
	struct virtio_drm_softc *sc;

	sc = container_of(drm_dev, struct virtio_drm_softc, drm_dev);
	if (sc->fb != NULL)
		drm_fb_helper_hotplug_event(&sc->fb->fb_helper);
}

static const struct drm_mode_config_funcs virtio_drm_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.output_poll_changed	= virtio_drm_output_poll_changed,
	.fb_create		= drm_gem_fb_create,
};

static struct drm_fb_helper_funcs fb_helper_funcs = {
	.fb_probe = drm_fb_cma_probe,
};

static int
virtio_drm_fb_preinit(struct drm_device *drm_dev)
{
	struct drm_fb_cma *fb;
	struct virtio_drm_softc *sc;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct virtio_drm_softc, drm_dev);

	fb = malloc(sizeof(*fb), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	drm_fb_helper_prepare(drm_dev, &fb->fb_helper, &fb_helper_funcs);
	sc->fb = fb;

	return (0);
}

static int
virtio_drm_fb_init(struct drm_device *drm_dev)
{
	struct virtio_drm_softc *sc;
	int rv;

	sc = container_of(drm_dev, struct virtio_drm_softc, drm_dev);

	drm_dev->dev = sc->dev;

	rv = drm_fb_helper_init(drm_dev, &sc->fb->fb_helper,
	     drm_dev->mode_config.num_connector);
	if (rv != 0) {
		device_printf(drm_dev->dev,
		    "Cannot initialize frame buffer %d\n", rv);
		return (rv);
	}

	rv = drm_fb_helper_single_add_all_connectors(&sc->fb->fb_helper);
	if (rv != 0) {
		device_printf(drm_dev->dev, "Cannot add all connectors: %d\n",
		    rv);
		goto err_fini;
	}

	rv = drm_fb_helper_initial_config(&sc->fb->fb_helper, 32);
	if (rv != 0) {
		device_printf(drm_dev->dev,
		    "Cannot set initial config: %d\n", rv);
		goto err_fini;
	}

	return 0;

err_fini:
	drm_fb_helper_fini(&sc->fb->fb_helper);
	return (rv);
}

static void
virtio_drm_irq_hook(void *arg)
{
	struct virtio_drm_softc *sc;
	phandle_t node;
	int error;
	int rv;

	sc = arg;

	sc->drm_dev.dev_private = sc;

	node = ofw_bus_get_node(sc->dev);

	drm_mode_config_init(&sc->drm_dev);

	rv = drm_dev_init(&sc->drm_dev, &virtio_drm_driver,
	    sc->dev);
	if (rv != 0) {
		device_printf(sc->dev, "drm_dev_init(): %d\n", rv);
		return;
	}

	error = virtio_add_encoder(sc->dev, &sc->crtc, &sc->drm_dev);

	virtio_cmd_get_edids(sc);
	virtio_gpu_cmd_get_display_info(sc);

	//virtio_plane_create(sc, &sc->drm_dev);

	if (error != 0) {
		device_printf(sc->dev, "%s: could not add encoder\n",
		    __func__);
		return;
	}

	virtio_drm_fb_preinit(&sc->drm_dev);
 
	drm_mode_config_reset(&sc->drm_dev);

	sc->drm_dev.mode_config.max_width = 2560;
	sc->drm_dev.mode_config.max_height = 1600;
	sc->drm_dev.mode_config.funcs = &virtio_drm_mode_config_funcs;
	sc->drm_dev.mode_config.quirk_addfb_prefer_host_byte_order = true;

	virtio_drm_fb_init(&sc->drm_dev);
 
	//drm_kms_helper_poll_init(&sc->drm_dev);

	/* Finally register our drm device */
	rv = drm_dev_register(&sc->drm_dev, 0);
	if (rv < 0)
		goto fail;

	//sc->drm_dev.irq_enabled = true;

	return;
fail:
	device_printf(sc->dev, "drm_dev_register(): %d\n", rv);
}

#define VTGPU_GET_CONFIG(_dev, _field, _cfg)		\
	virtio_read_device_config(_dev,			\
	    offsetof(struct virtio_gpu_config, _field),	\
	    &(_cfg)->_field, sizeof((_cfg)->_field));

static void
vtgpu_read_config(struct virtio_drm_softc *sc,
    struct virtio_gpu_config *gpucfg)
{
	device_t dev;

	dev = sc->dev;

	bzero(gpucfg, sizeof(struct virtio_gpu_config));

	VTGPU_GET_CONFIG(dev, events_read, gpucfg);
	VTGPU_GET_CONFIG(dev, events_clear, gpucfg);
	VTGPU_GET_CONFIG(dev, num_scanouts, gpucfg);
	VTGPU_GET_CONFIG(dev, num_capsets, gpucfg);
}

#undef VTGPU_GET_CONFIG

static int
vtgpu_probe(device_t dev)
{

	return (VIRTIO_SIMPLE_PROBE(dev, virtio_gpu));
}

static int
vtgpu_attach(device_t dev)
{
	struct virtio_drm_softc *sc, *exp;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	virtio_set_feature_desc(dev, vtgpu_feature_desc);

printf("%s\n", __func__);

	error = vtgpu_setup_features(sc);
	if (error) {
		device_printf(dev, "cannot setup features\n");
		goto fail;
	}

	if (virtio_with_feature(dev, (1 << VIRTIO_GPU_F_VIRGL))) {
		printf("virtio has virgl\n");
		sc->has_virgl_3d = true;
	} else
		printf("virtio has no virgl\n");

	if (virtio_with_feature(dev, (1 << VIRTIO_GPU_F_RESOURCE_BLOB)))
		sc->has_resource_blob = true;

	if (virtio_with_feature(dev, (1 << VIRTIO_GPU_F_RESOURCE_UUID)))
		sc->has_resource_assign_uuid = true;

	if (virtio_with_feature(dev, (1 << VIRTIO_GPU_F_CONTEXT_INIT)))
		sc->has_context_init = true;

#if 0
        bool                    has_host_visible;
        int                     capset_id_mask;
#endif

	if (virtio_with_feature(dev, VIRTIO_RING_F_INDIRECT_DESC))
		printf("%s: indirect desc negotiated\n", __func__);

	error = vtgpu_alloc_virtqueue(sc);
	if (error) {
		device_printf(dev, "cannot allocate virtqueue\n");
		goto fail;
	}

	error = virtio_setup_intr(dev, INTR_TYPE_MISC);
	if (error) {
		device_printf(dev, "cannot setup virtqueue interrupts\n");
		goto fail;
	}

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->flush_ticker, &sc->sc_mtx, 0);

	vtgpu_read_config(sc, &sc->gpucfg);
	printf("%s: num_scanouts %d\n", __func__, sc->gpucfg.num_scanouts);

	exp = NULL;
	if (!atomic_compare_exchange_strong_explicit(&g_virtio_drm_softc, &exp, sc,
	    memory_order_release, memory_order_acquire)) {
		error = EEXIST;
		goto fail;
	}

	config_intrhook_oneshot(&virtio_drm_irq_hook, sc);

fail:
	if (error)
		vtgpu_detach(dev);

	return (error);
}

static int
vtgpu_detach(device_t dev)
{
	struct virtio_drm_softc *sc;

	sc = device_get_softc(dev);
	KASSERT(
	    atomic_load_explicit(&g_virtio_drm_softc, memory_order_acquire) ==
		sc, ("only one global instance at a time"));

	atomic_store_explicit(&g_virtio_drm_softc, NULL, memory_order_release);
	return (0);
}

static int
vtgpu_negotiate_features(struct virtio_drm_softc *sc)
{
	device_t dev;
	uint64_t features;

	dev = sc->dev;
	features = VTGPU_FEATURES;

	sc->vtgpu_features = virtio_negotiate_features(dev, features);
	return (virtio_finalize_features(dev));
}

static int
vtgpu_setup_features(struct virtio_drm_softc *sc)
{
	int error;

	error = vtgpu_negotiate_features(sc);
	if (error)
		return (error);

	return (0);
}

static void
vtgpu_ctrlq_intr(void *arg)
{

	printf("%s\n", __func__);
}

static void
vtgpu_cursor_intr(void *arg)
{

	printf("%s\n", __func__);
}

static int
vtgpu_alloc_virtqueue(struct virtio_drm_softc *sc)
{
	struct vq_alloc_info vq_info[2];
	device_t dev;
	int nvqs;

	dev = sc->dev;

	nvqs = 2;

	VQ_ALLOC_INFO_INIT(&vq_info[0], 0, vtgpu_ctrlq_intr, sc, &sc->ctrlq,
	    "%s control", device_get_nameunit(dev));

	VQ_ALLOC_INFO_INIT(&vq_info[1], 0, vtgpu_cursor_intr, sc, &sc->cursorq,
	    "%s cursor", device_get_nameunit(dev));

	return (virtio_alloc_virtqueues(dev, 0, nvqs, vq_info));
}
