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

#include "panfrost_drv.h"
#include "panfrost_drm.h"
#include "panfrost_device.h"
#include "panfrost_gem.h"
#include "panfrost_regs.h"
#include "panfrost_features.h"
#include "panfrost_issues.h"
#include "panfrost_mmu.h"

static void
panfrost_gem_free_object(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);
}

int
panfrost_gem_open(struct drm_gem_object *obj, struct drm_file *file_priv)
{
	struct panfrost_gem_mapping *mapping;
	struct panfrost_gem_object *bo;
	struct panfrost_file *pfile;
	uint32_t align;
	int error;
	int color;

	bo = (struct panfrost_gem_object *)obj;
	pfile = file_priv->driver_priv;

	mapping = malloc(sizeof(*mapping), M_DEVBUF, M_ZERO | M_WAITOK);
	mapping->obj = bo;
	mapping->mmu = &pfile->mmu;

	drm_gem_object_get(obj);

	if (bo->noexec) {
		align = obj->size >= 0x200000 ? 0x200000 >> PAGE_SHIFT : 0;
		color = PANFROST_BO_NOEXEC;
	} else {
		align = obj->size >> PAGE_SHIFT;
		color = 0;
	}

	printf("%s\n", __func__);

	mtx_lock_spin(&pfile->mm_lock);
	error = drm_mm_insert_node_generic(&pfile->mm, &mapping->mmnode,
	    obj->size >> PAGE_SHIFT, align, color, 0 /* mode */);
	mtx_unlock_spin(&pfile->mm_lock);
	if (error) {
		printf("Failed to insert node\n");
		/* put mapping */
		return (error);
	}

	printf("%s: mapping->mmnode.start page %lx va %lx\n", __func__,
	    mapping->mmnode.start, mapping->mmnode.start << PAGE_SHIFT);

#if 0
	struct page **pages;
	pages = drm_gem_get_pages(obj);
#endif

	if (!bo->is_heap) {
		error = panfrost_mmu_map(mapping);
		if (error)
			return (error);
	}

	printf("%s: return 0\n", __func__);

	return (0);
}

void
panfrost_gem_close(struct drm_gem_object *obj, struct drm_file *file_priv)
{

	printf("%s\n", __func__);
}

void
drm_gem_shmem_print_info(struct drm_printer *p, unsigned int indent,
    const struct drm_gem_object *obj)
{

	printf("%s\n", __func__);
}

static int
panfrost_gem_pin(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (0);
}

void drm_gem_shmem_unpin(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);
}

struct sg_table *
drm_gem_shmem_get_sg_table(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (NULL);
}

void *
drm_gem_shmem_vmap(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (0);
}

void
drm_gem_shmem_vunmap(struct drm_gem_object *obj, void *vaddr)
{

	printf("%s\n", __func__);
}

int
drm_gem_shmem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{

	printf("%s\n", __func__);

	return (0);
}

static const struct drm_gem_object_funcs panfrost_gem_funcs = {
	.free = panfrost_gem_free_object,
	.open = panfrost_gem_open,
	.close = panfrost_gem_close,
	.print_info = drm_gem_shmem_print_info,
	.pin = panfrost_gem_pin,
	.unpin = drm_gem_shmem_unpin,
	.get_sg_table = drm_gem_shmem_get_sg_table,
	.vmap = drm_gem_shmem_vmap,
	.vunmap = drm_gem_shmem_vunmap,
	.mmap = drm_gem_shmem_mmap,
};

struct panfrost_gem_object *
panfrost_gem_create_object(struct drm_file *file, struct drm_device *dev,
    size_t size, uint32_t flags, uint32_t *handle)
{
	struct panfrost_gem_object *obj;
	int error;

printf("%s\n", __func__);

	obj = malloc(sizeof(*obj), M_DEVBUF, M_ZERO | M_WAITOK);
	obj->base.funcs = &panfrost_gem_funcs;

	size = PAGE_ALIGN(size);

	drm_gem_object_init(dev, &obj->base, size);
	error = drm_gem_create_mmap_offset(&obj->base);
	if (error != 0) {
		printf("Failed to create mmap offset\n");
		return (NULL);
	}
	obj->noexec = !!(flags & PANFROST_BO_NOEXEC);
	obj->is_heap = !!(flags & PANFROST_BO_HEAP);

	error = drm_gem_handle_create(file, &obj->base, handle);
	drm_gem_object_put(&obj->base);
	if (error) {
		printf("Failed to create handle\n");
		return (NULL);
	}

	return (NULL);
}

struct panfrost_gem_mapping *
panfrost_gem_mapping_get(struct panfrost_gem_object *bo,
    struct panfrost_file *priv)
{

	return (NULL);
}
