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

#include <dev/drm/komeda/komeda_gem.h>

#include <dev/drm/drmkpi/include/linux/dma-buf.h>

#include "fb_if.h"
//#include "komeda_vop_if.h"

struct komeda_drm_softc {
	device_t		dev;

	struct drm_device	drm_dev;
	struct drm_fb_cma	*fb;
};

static struct ofw_compat_data compat_data[] = {
	{ "arm,mali-d32",		1 },
	{ NULL,				0 }
};

static int komeda_drm_probe(device_t dev);
static int komeda_drm_attach(device_t dev);
static int komeda_drm_detach(device_t dev);

MALLOC_DECLARE(M_KOMEDA);
MALLOC_DEFINE(M_KOMEDA, "komeda", "Komeda DRM");

/* DRM driver fops */
static const struct file_operations komeda_drm_drv_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.mmap = drm_gem_cma_mmap,
	.kqfilter = drm_kqfilter,
	/* .llseek = noop_llseek, */
};

static struct drm_driver komeda_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | \
	    DRIVER_ATOMIC | DRIVER_PRIME,

	/* Generic Operations */
	.lastclose = drm_fb_helper_lastclose,
	.fops = &komeda_drm_drv_fops,

	/* GEM Opeations */
	.dumb_create = drm_gem_cma_dumb_create,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,

	.gem_prime_get_sg_table	= komeda_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = komeda_gem_prime_import_sg_table,
	.gem_prime_mmap		= komeda_gem_mmap_buf,

	.name			= "mali-dp",
	.desc			= "ARM Komeda Display Subsystem",
	.date			= "20211401",
	.major			= 1,
	.minor			= 0,
};

static void
komeda_drm_output_poll_changed(struct drm_device *drm_dev)
{
	struct komeda_drm_softc *sc;

	sc = container_of(drm_dev, struct komeda_drm_softc, drm_dev);
	if (sc->fb != NULL)
		drm_fb_helper_hotplug_event(&sc->fb->fb_helper);
}

static const struct drm_mode_config_funcs komeda_drm_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.output_poll_changed	= komeda_drm_output_poll_changed,
	.fb_create		= drm_gem_fb_create,
};

static struct drm_mode_config_helper_funcs komeda_drm_mode_config_helpers = {
	.atomic_commit_tail	= drm_atomic_helper_commit_tail_rpm,
};

static struct fb_info *
drm_fb_cma_helper_getinfo(device_t dev)
{
	struct komeda_drm_softc *sc;

	sc = device_get_softc(dev);
	if (sc->fb == NULL)
		return (NULL);

	return (sc->fb->fb_helper.fbdev);
}

static struct drm_fb_helper_funcs fb_helper_funcs = {
	.fb_probe = drm_fb_cma_probe,
};

static int
komeda_drm_fb_preinit(struct drm_device *drm_dev)
{
	struct drm_fb_cma *fb;
	struct komeda_drm_softc *sc;

	printf("%s\n", __func__);

	sc = container_of(drm_dev, struct komeda_drm_softc, drm_dev);

	fb = malloc(sizeof(*fb), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	drm_fb_helper_prepare(drm_dev, &fb->fb_helper, &fb_helper_funcs);
	sc->fb = fb;

	return (0);
}

static int
komeda_drm_fb_init(struct drm_device *drm_dev)
{
	struct komeda_drm_softc *sc;
	int rv;

	sc = container_of(drm_dev, struct komeda_drm_softc, drm_dev);

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

#if 0
static void
komeda_drm_fb_destroy(struct drm_device *drm_dev)
{
	struct fb_info *info;
	struct drm_fb_cma *fb;
	struct komeda_drm_softc *sc;

	sc = container_of(drm_dev, struct komeda_drm_softc, drm_dev);
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
#endif

static void
komeda_drm_irq_hook(void *arg)
{
	struct komeda_drm_softc *sc;
	phandle_t node;
	device_t portdev;
	uint32_t *ports;
	int rv, nports, i;

	sc = arg;

	node = ofw_bus_get_node(sc->dev);

	drm_mode_config_init(&sc->drm_dev);

	rv = drm_dev_init(&sc->drm_dev, &komeda_drm_driver,
	    sc->dev);
	if (rv != 0) {
		device_printf(sc->dev, "drm_dev_init(): %d\n", rv);
		return;
	}

	/* Query pipelines. */
	uint32_t pipeline_reg;
	phandle_t child;
	char *name;
	int ret;

	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		ret = OF_getprop_alloc(child, "name", (void **)&name);
		if (ret == -1)
			continue;

		if (strcasecmp(name, "pipeline") ||
		    strncasecmp(name, "pipeline@", 10)) {
			pipeline_reg = -1;
			OF_getencprop(child, "reg", (void *)&pipeline_reg,
			    sizeof(pipeline_reg));
			printf("%s: pipeline found, reg %x\n", __func__,
			    pipeline_reg);
		}

	}

	nports = OF_getencprop_alloc_multi(node, "ports", sizeof(*ports),
	    (void **)&ports);
	if (nports <= 0) {
		device_printf(sc->dev, "Cannot find ports property\n");
		goto fail;
	}

	/* Attach the port(s) */
	for (i = 0; i < nports; i++) {
		if (bootverbose)
			device_printf(sc->dev, "Lookup port with phandle %x\n",
			    ports[i]);
		portdev = OF_device_from_xref(ports[i]);
		if (portdev != NULL) {
			device_printf(sc->dev, "port found\n");
			//KOMEDA_CREATE_PIPELINE(portdev, &sc->drm_dev);
		} else
			device_printf(sc->dev,
			    "Cannot find port with phandle %x\n", ports[i]);
	}

	komeda_drm_fb_preinit(&sc->drm_dev);

	drm_vblank_init(&sc->drm_dev, sc->drm_dev.mode_config.num_crtc);

	drm_mode_config_reset(&sc->drm_dev);
	sc->drm_dev.mode_config.max_width = 4096;
	sc->drm_dev.mode_config.max_height = 4096;
	sc->drm_dev.mode_config.funcs = &komeda_drm_mode_config_funcs;
	sc->drm_dev.mode_config.helper_private =
	    &komeda_drm_mode_config_helpers;

	komeda_drm_fb_init(&sc->drm_dev);

	drm_kms_helper_poll_init(&sc->drm_dev);

	/* Finally register our drm device */
	rv = drm_dev_register(&sc->drm_dev, 0);
	if (rv < 0)
		goto fail;

	sc->drm_dev.irq_enabled = true;

printf("%s: DRM device registered\n", __func__);

	return;
fail:
	device_printf(sc->dev, "drm_dev_register(): %d\n", rv);
}

static int
komeda_drm_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "ARM Komeda Display Subsystem");
	return (BUS_PROBE_DEFAULT);
}

static int
komeda_drm_attach(device_t dev)
{
	struct komeda_drm_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	config_intrhook_oneshot(&komeda_drm_irq_hook, sc);

	return (0);
}

static int
komeda_drm_detach(device_t dev)
{
	struct komeda_drm_softc *sc;

	sc = device_get_softc(dev);

	drm_dev_unregister(&sc->drm_dev);
	drm_kms_helper_poll_fini(&sc->drm_dev);
	drm_atomic_helper_shutdown(&sc->drm_dev);
	drm_mode_config_cleanup(&sc->drm_dev);

	return (0);
}

static device_method_t komeda_drm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		komeda_drm_probe),
	DEVMETHOD(device_attach,	komeda_drm_attach),
	DEVMETHOD(device_detach,	komeda_drm_detach),

	DEVMETHOD(fb_getinfo,		drm_fb_cma_helper_getinfo),

	DEVMETHOD_END
};

static driver_t komeda_driver = {
	"komeda_drm",
	komeda_drm_methods,
	sizeof(struct komeda_drm_softc),
};

static devclass_t komeda_drm_devclass;

EARLY_DRIVER_MODULE(komeda_drm, simplebus, komeda_driver, komeda_drm_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_FIRST);
//MODULE_DEPEND(komeda_drm, komeda_vop, 1, 1, 1);

/* Bindings for fbd device. */
extern devclass_t fbd_devclass;
extern driver_t fbd_driver;
DRIVER_MODULE(fbd, komeda_drm, fbd_driver, fbd_devclass, 0, 0);
