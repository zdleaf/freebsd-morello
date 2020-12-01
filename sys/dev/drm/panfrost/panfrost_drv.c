/*-
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
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
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_vblank.h>

#include "fb_if.h"
#include "panfrost_drm.h"
#include "panfrost_drv.h"
#include "panfrost_device.h"
#include "panfrost_regs.h"
#include "panfrost_gem.h"
#include "panfrost_mmu.h"

static struct resource_spec mali_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE | RF_SHAREABLE },
	{ SYS_RES_IRQ,		2,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

static struct ofw_compat_data compat_data[] = {
	{ "arm,mali-t860",	1 },
	{ NULL,			0 }
};

static int panfrost_probe(device_t dev);
static int panfrost_attach(device_t dev);
static int panfrost_detach(device_t dev);

static const struct file_operations panfrost_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
	.compat_ioctl	= drm_compat_ioctl,
	.poll		= drm_poll,
	.read		= drm_read,
	/*.llseek	= noop_llseek,*/
	.mmap		= drm_gem_mmap,
};

static void
panfrost_drm_mm_color_adjust(const struct drm_mm_node *node,
    unsigned long color, u64 *start, u64 *end)
{

	panic("implement me");
}

static int
panfrost_open(struct drm_device *dev, struct drm_file *file)
{
	struct panfrost_file *pfile;
	struct panfrost_softc *sc;
	struct drm_mm mm;
	int error;

	printf("%s\n", __func__);

	sc = dev->dev_private;

	pfile = malloc(sizeof(*pfile), M_DEVBUF, M_WAITOK | M_ZERO);
	pfile->sc = sc;
	file->driver_priv = pfile;

	mtx_init(&pfile->mm_lock, "mm", NULL, MTX_SPIN);

	drm_mm_init(&mm, 32*1024*1024 >> PAGE_SHIFT,
	    (4*1024*1024*1024ULL - 32*1024*1024) >> PAGE_SHIFT);
	pfile->mm.color_adjust = panfrost_drm_mm_color_adjust;

	error = panfrost_mmu_pgtable_alloc(pfile);
	if (error != 0) {
		drm_mm_takedown(&pfile->mm);
		free(pfile, M_DEVBUF);
		return (error);
	}

	return (0);
}

static void
panfrost_postclose(struct drm_device *dev, struct drm_file *file)
{

	printf("%s\n", __func__);
}

static struct drm_gem_object *
panfrost_gem_prime_import_sg_table(struct drm_device *dev,
    struct dma_buf_attachment *attach, struct sg_table *sgt)
{

	printf("%s\n", __func__);

	return (NULL);
}

static int
panfrost_ioctl_submit(struct drm_device *dev, void *data,
    struct drm_file *file)
{

	printf("%s\n", __func__);

	return (0);
}

static int
panfrost_ioctl_wait_bo(struct drm_device *dev, void *data,
    struct drm_file *file_priv)
{

	printf("%s\n", __func__);

	return (0);
}

static int
panfrost_ioctl_create_bo(struct drm_device *dev, void *data,
    struct drm_file *file)
{
	struct drm_panfrost_create_bo *args;

	args = data;

	printf("%s: size %d flags %d handle %d pad %d offset %jd\n",
	    __func__, args->size, args->flags, args->handle, args->pad,
	    args->offset);

	panfrost_gem_create_object(file, dev, args->size, args->flags,
	    &args->handle);

	return (0);
}

static int
panfrost_ioctl_mmap_bo(struct drm_device *dev, void *data,
    struct drm_file *file_priv)
{

	printf("%s\n", __func__);

	return (0);
}

static int
panfrost_ioctl_get_param(struct drm_device *ddev, void *data,
    struct drm_file *file)
{
	struct drm_panfrost_get_param *param;
	struct panfrost_softc *sc;

	sc = ddev->dev_private;
	param = data;

	if (param->pad != 0)
		return (EINVAL);

	printf("%s: param %d\n", __func__, param->param);

	switch (param->param) {
	case DRM_PANFROST_PARAM_GPU_PROD_ID:
		param->value = sc->features.id;
		break;
	case DRM_PANFROST_PARAM_GPU_REVISION:
		param->value = sc->features.revision;
		break;
	case DRM_PANFROST_PARAM_SHADER_PRESENT:
		param->value = sc->features.shader_present;
		break;
	case DRM_PANFROST_PARAM_TILER_PRESENT:
		param->value = sc->features.tiler_present;
		break;
	case DRM_PANFROST_PARAM_L2_PRESENT:
		param->value = sc->features.l2_present;
		break;
	case DRM_PANFROST_PARAM_STACK_PRESENT:
		param->value = sc->features.stack_present;
		break;
	case DRM_PANFROST_PARAM_AS_PRESENT:
		param->value = sc->features.as_present;
		break;
	case DRM_PANFROST_PARAM_JS_PRESENT:
		param->value = sc->features.js_present;
		break;
	case DRM_PANFROST_PARAM_L2_FEATURES:
		param->value = sc->features.l2_features;
		break;
	case DRM_PANFROST_PARAM_CORE_FEATURES:
		param->value = sc->features.core_features;
		break;
	case DRM_PANFROST_PARAM_TILER_FEATURES:
		param->value = sc->features.tiler_features;
		break;
	case DRM_PANFROST_PARAM_MEM_FEATURES:
		param->value = sc->features.mem_features;
		break;
	case DRM_PANFROST_PARAM_MMU_FEATURES:
		param->value = sc->features.mmu_features;
		break;
	case DRM_PANFROST_PARAM_THREAD_FEATURES:
		param->value = sc->features.thread_features;
		break;
	case DRM_PANFROST_PARAM_MAX_THREADS:
		param->value = sc->features.thread_max_threads;
		break;
	case DRM_PANFROST_PARAM_THREAD_MAX_WORKGROUP_SZ:
		param->value = sc->features.thread_max_workgroup_size;
		break;
	case DRM_PANFROST_PARAM_THREAD_MAX_BARRIER_SZ:
		param->value = sc->features.thread_max_barrier_size;
		break;
	case DRM_PANFROST_PARAM_COHERENCY_FEATURES:
		param->value = sc->features.coherency_features;
		break;
	case DRM_PANFROST_PARAM_NR_CORE_GROUPS:
		param->value = sc->features.nr_core_groups;
		break;
	case DRM_PANFROST_PARAM_THREAD_TLS_ALLOC:
		param->value = sc->features.thread_tls_alloc;
		break;
	case DRM_PANFROST_PARAM_TEXTURE_FEATURES0 ...
	    DRM_PANFROST_PARAM_TEXTURE_FEATURES3:
		param->value = sc->features.texture_features[param->param -
		    DRM_PANFROST_PARAM_TEXTURE_FEATURES0];
		break;
	case DRM_PANFROST_PARAM_JS_FEATURES0 ...
	    DRM_PANFROST_PARAM_JS_FEATURES15:
		param->value = sc->features.js_features[param->param -
		    DRM_PANFROST_PARAM_JS_FEATURES0];
		break;
	default:
		return (EINVAL);
	}

	return (0);
}

static int
panfrost_ioctl_get_bo_offset(struct drm_device *dev, void *data,
    struct drm_file *file_priv)
{

	printf("%s\n", __func__);

	return (0);
}

static int
panfrost_ioctl_madvise(struct drm_device *dev, void *data,
    struct drm_file *file_priv)
{

	printf("%s\n", __func__);

	return (0);
}

static const struct drm_ioctl_desc panfrost_drm_driver_ioctls[] = {
#define	PANFROST_IOCTL(name, func, flags) \
	DRM_IOCTL_DEF_DRV(PANFROST_##name, panfrost_ioctl_##func, flags)

	PANFROST_IOCTL(SUBMIT,		submit,		DRM_RENDER_ALLOW),
	PANFROST_IOCTL(WAIT_BO,		wait_bo,	DRM_RENDER_ALLOW),
	PANFROST_IOCTL(CREATE_BO,	create_bo,	DRM_RENDER_ALLOW),
	PANFROST_IOCTL(MMAP_BO,		mmap_bo,	DRM_RENDER_ALLOW),
	PANFROST_IOCTL(GET_PARAM,	get_param,	DRM_RENDER_ALLOW),
	PANFROST_IOCTL(GET_BO_OFFSET,	get_bo_offset,	DRM_RENDER_ALLOW),
	//PANFROST_IOCTL(PERFCNT_ENABLE,perfcnt_enable,	DRM_RENDER_ALLOW),
	//PANFROST_IOCTL(PERFCNT_DUMP,	perfcnt_dump,	DRM_RENDER_ALLOW),
	PANFROST_IOCTL(MADVISE,		madvise,	DRM_RENDER_ALLOW),
};

static struct drm_driver panfrost_drm_driver = {
	.driver_features = DRIVER_RENDER | DRIVER_GEM | DRIVER_SYNCOBJ,

	.open			= panfrost_open,
	.postclose		= panfrost_postclose,
	.ioctls			= panfrost_drm_driver_ioctls,
	.num_ioctls		= ARRAY_SIZE(panfrost_drm_driver_ioctls),
	.fops			= &panfrost_drm_driver_fops,

#if 0
	.gem_create_object	= panfrost_gem_create_object,
#endif
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = panfrost_gem_prime_import_sg_table,
	.gem_prime_mmap		= drm_gem_prime_mmap,

	.name			= "panfrost",
	.desc			= "panfrost DRM",
	.date			= "20201124",
	.major			= 1,
	.minor			= 0,
};

#if 0
static void
panfrost_output_poll_changed(struct drm_device *drm_dev)
{
	struct panfrost_softc *sc;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct panfrost_softc, drm_dev);
	if (sc->fb != NULL)
		drm_fb_helper_hotplug_event(&sc->fb->fb_helper);
}

static const struct drm_mode_config_funcs panfrost_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.output_poll_changed	= panfrost_output_poll_changed,
	.fb_create		= drm_gem_fb_create,
};

static struct drm_mode_config_helper_funcs panfrost_mode_config_helpers = {
	.atomic_commit_tail	= drm_atomic_helper_commit_tail_rpm,
};
#endif

static struct fb_info *
drm_fb_cma_helper_getinfo(device_t dev)
{
	struct panfrost_softc *sc;

	printf("%s\n", __func__);

	sc = device_get_softc(dev);
	if (sc->fb == NULL)
		return (NULL);
	return (sc->fb->fb_helper.fbdev);
}

static struct drm_fb_helper_funcs fb_helper_funcs = {
	.fb_probe = drm_fb_cma_probe,
};

static int
panfrost_fb_preinit(struct drm_device *drm_dev)
{
	struct drm_fb_cma *fb;
	struct panfrost_softc *sc;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct panfrost_softc, drm_dev);

	fb = malloc(sizeof(*fb), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	drm_fb_helper_prepare(drm_dev, &fb->fb_helper, &fb_helper_funcs);
	sc->fb = fb;

	return (0);
}

static int
panfrost_fb_init(struct drm_device *drm_dev)
{
	struct panfrost_softc *sc;
	int rv;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct panfrost_softc, drm_dev);

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
panfrost_fb_destroy(struct drm_device *drm_dev)
{
	struct fb_info *info;
	struct drm_fb_cma *fb;
	struct panfrost_softc *sc;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct panfrost_softc, drm_dev);
	fb = sc->fb;
	if (fb == NULL)
		return;
	info = fb->fb_helper.fbdev;

	drm_framebuffer_remove(&fb->drm_fb);
	framebuffer_release(info);
	drm_fb_helper_fini(&fb->fb_helper);
	drm_framebuffer_cleanup(&fb->drm_fb);

	free(fb, DRM_MEM_DRIVER);
	sc->fb = NULL;
}

#if 0
static void
panfrost_irq_hook(void *arg)
{
	struct panfrost_softc *sc;
	phandle_t node;
	int rv;

	sc = arg;

	printf("%s\n", __func__);

	node = ofw_bus_get_node(sc->dev);

	drm_mode_config_init(&sc->drm_dev);

	rv = drm_dev_init(&sc->drm_dev, &panfrost_drm_driver,
	    sc->dev);
	if (rv != 0) {
		device_printf(sc->dev, "drm_dev_init(): %d\n", rv);
		return;
	}

	panfrost_device_init(sc);

	panfrost_fb_preinit(&sc->drm_dev);

	drm_vblank_init(&sc->drm_dev, sc->drm_dev.mode_config.num_crtc);

	drm_mode_config_reset(&sc->drm_dev);
	sc->drm_dev.mode_config.max_width = 1920;
	sc->drm_dev.mode_config.max_height = 1080;
	sc->drm_dev.mode_config.funcs = &panfrost_mode_config_funcs;
	sc->drm_dev.mode_config.helper_private = &panfrost_mode_config_helpers;

	panfrost_fb_init(&sc->drm_dev);

	drm_kms_helper_poll_init(&sc->drm_dev);

	/* Finally register our drm device */
	rv = drm_dev_register(&sc->drm_dev, 0);
	if (rv < 0)
		goto fail;

	sc->drm_dev.irq_enabled = true;

	return;
fail:
	device_printf(sc->dev, "drm_dev_register(): %d\n", rv);
}
#endif

static void
panfrost_job_intr(void *arg)
{

	printf("%s\n", __func__);
}

static void
panfrost_mmu_intr(void *arg)
{

	printf("%s\n", __func__);
}

static void
panfrost_gpu_intr(void *arg)
{
	struct panfrost_softc *sc;
	uint32_t pending;

	sc = arg;

	pending = GPU_READ(sc, GPU_INT_STAT);

	printf("%s: pending %x\n", __func__, pending);

	if (pending & GPU_IRQ_POWER_CHANGED ||
	    pending & GPU_IRQ_POWER_CHANGED_ALL) {
		/* Ignore power events. */
	}

	GPU_WRITE(sc, GPU_INT_CLEAR, pending);
}

static void
panfrost_irq_hook(void *arg)
{
	struct panfrost_softc *sc;
	int err;

	sc = arg;

	printf("%s\n", __func__);

	drm_mode_config_init(&sc->drm_dev);

	err = drm_dev_init(&sc->drm_dev, &panfrost_drm_driver,
	    sc->dev);
	if (err != 0) {
		device_printf(sc->dev, "drm_dev_init(): %d\n", err);
		return;
	}

	sc->drm_dev.dev_private = sc;

	panfrost_device_init(sc);

	err = drm_dev_register(&sc->drm_dev, 0);
	if (err < 0) {
		device_printf(sc->dev, "drm_dev_register(): %d\n", err);
		return;
	}
}

static int
panfrost_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Mali Midgard/Bifrost GPU");
	return (BUS_PROBE_DEFAULT);
}

static int
panfrost_attach(device_t dev)
{
	struct panfrost_softc *sc;
	phandle_t node;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(sc->dev);

	if (bus_alloc_resources(dev, mali_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, panfrost_job_intr, sc,
	    &sc->intrhand[0])) {
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->res[2],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, panfrost_mmu_intr, sc,
	    &sc->intrhand[1])) {
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->res[3],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, panfrost_gpu_intr, sc,
	    &sc->intrhand[2])) {
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_index(sc->dev, 0, 0, &sc->clk) != 0) {
		device_printf(dev, "cannot get clock\n");
		return (ENXIO);
	}

	err = clk_enable(sc->clk);
	if (err != 0) {
		device_printf(sc->dev, "could not enable clock: %d\n", err);
		return (ENXIO);
	}

	uint64_t rate;
	clk_get_freq(sc->clk, &rate);

	device_printf(dev, "Mali GPU clock rate %jd Hz\n", rate);

	config_intrhook_oneshot(&panfrost_irq_hook, sc);

	return (0);
}

static int
panfrost_detach(device_t dev)
{
	struct panfrost_softc *sc;

	sc = device_get_softc(dev);

	drm_dev_unregister(&sc->drm_dev);
	drm_kms_helper_poll_fini(&sc->drm_dev);
	drm_atomic_helper_shutdown(&sc->drm_dev);
	drm_mode_config_cleanup(&sc->drm_dev);

	return (0);
}

static device_method_t panfrost_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		panfrost_probe),
	DEVMETHOD(device_attach,	panfrost_attach),
	DEVMETHOD(device_detach,	panfrost_detach),

	DEVMETHOD_END
};

static driver_t panfrost_driver = {
	"panfrost",
	panfrost_methods,
	sizeof(struct panfrost_softc),
};

static devclass_t panfrost_devclass;

EARLY_DRIVER_MODULE(panfrost, simplebus, panfrost_driver, panfrost_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);

#if 0
/* Bindings for fbd device. */
extern devclass_t fbd_devclass;
extern driver_t fbd_driver;
DRIVER_MODULE(fbd, panfrost, fbd_driver, fbd_devclass, 0, 0);
#endif
