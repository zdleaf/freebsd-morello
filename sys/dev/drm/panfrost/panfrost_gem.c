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
#include <drm/gpu_scheduler.h>

#include <dev/drm/drmkpi/include/linux/dma-buf.h>

#include "panfrost_drv.h"
#include "panfrost_job.h"
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

	printf("%s: TODO: unmap needed\n", __func__);
}

int
panfrost_gem_open(struct drm_gem_object *obj, struct drm_file *file_priv)
{
	struct panfrost_softc *sc;
	struct panfrost_gem_mapping *mapping;
	struct panfrost_gem_object *bo;
	struct panfrost_file *pfile;
	uint32_t align;
	int error;
	int color;

	bo = (struct panfrost_gem_object *)obj;
	pfile = file_priv->driver_priv;
	sc = pfile->sc;

	mapping = malloc(sizeof(*mapping), M_DEVBUF, M_ZERO | M_WAITOK);
	mapping->obj = bo;
	mapping->mmu = &pfile->mmu;

	drm_gem_object_get(obj);

	if (!bo->noexec) {
		align = obj->size >> PAGE_SHIFT;
		color = 0;
	} else {
		align = obj->size >= 0x200000 ? 0x200000 >> PAGE_SHIFT : 0;
		color = PANFROST_BO_NOEXEC;
	}

	dprintf("%s\n", __func__);

	mtx_lock_spin(&pfile->mm_lock);
	error = drm_mm_insert_node_generic(&pfile->mm, &mapping->mmnode,
	    obj->size >> PAGE_SHIFT, align, color, 0 /* mode */);
	mtx_unlock_spin(&pfile->mm_lock);
	if (error) {
		printf("Failed to insert node\n");
		/* put mapping */
		return (error);
	}

	dprintf("%s: mapping->mmnode.start page %lx va %lx\n", __func__,
	    mapping->mmnode.start, mapping->mmnode.start << PAGE_SHIFT);

#if 0
	struct page **pages;
	pages = drm_gem_get_pages(obj);
#endif

	if (!bo->is_heap) {
		error = panfrost_mmu_map(sc, mapping);
		if (error)
			return (error);
	} else {
		printf("%s: is heap\n", __func__);
		panic("ok");
	}

	dprintf("%s: return 0\n", __func__);

	mtx_lock(&bo->mappings_lock);
	TAILQ_INSERT_TAIL(&bo->mappings, mapping, next);
	mtx_unlock(&bo->mappings_lock);

	return (0);
}

void
panfrost_gem_close(struct drm_gem_object *obj, struct drm_file *file_priv)
{

	//printf("%s: TODO unmap needed\n", __func__);
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

static vm_fault_t
panfrost_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct panfrost_gem_object *bo;
	struct drm_gem_object *gem_obj;
	vm_object_t obj;
	vm_pindex_t pidx;
	vm_page_t m;
	struct page *page;
	int i;

	obj = vma->vm_obj;
	gem_obj = vma->vm_private_data;
	bo = (struct panfrost_gem_object *)gem_obj;
	m = bo->pages;

	pidx = OFF_TO_IDX(vmf->address - vma->vm_start);
	if (pidx >= bo->npages) {
		printf("%s: error: requested page is out of range (%d/%d)\n",
		    __func__, pidx, bo->npages);
		return (VM_FAULT_SIGBUS);
	}

	dprintf("%s: bo %p pidx %d, m %p, pgoff %d\n",
	    __func__, bo, pidx, m, vmf->pgoff);

	VM_OBJECT_WLOCK(obj);
	for (i = 0; i < bo->npages; i++) {
		page = m++;
		//printf("%s: busied page %d\n", __func__, i);
		if (vm_page_busied(page))
			goto fail_unlock;
		//printf("%s: insert page %d\n", __func__, i);
		if (vm_page_insert(page, obj, i))
			goto fail_unlock;
		vm_page_xbusy(page);
		page->valid = VM_PAGE_BITS_ALL;
	}
	VM_OBJECT_WUNLOCK(obj);

	vma->vm_pfn_first = 0;
	vma->vm_pfn_count = bo->npages;

	dprintf("%s: pidx: %llu, start: 0x%08x, addr: 0x%08lx\n",
	    __func__, pidx, vma->vm_start, vmf->address);

	return (VM_FAULT_NOPAGE);

fail_unlock:
	VM_OBJECT_WUNLOCK(obj);
	printf("%s: insert failed\n", __func__);
	panic("failed");

	return (VM_FAULT_SIGBUS);
}

static void
panfrost_gem_vm_open(struct vm_area_struct *vma)
{

	dprintf("%s\n", __func__);
	drm_gem_vm_open(vma);
}

static void
panfrost_gem_vm_close(struct vm_area_struct *vma)
{

	dprintf("%s\n", __func__);
	drm_gem_vm_close(vma);
}

static const struct vm_operations_struct panfrost_gem_vm_ops = {
	.fault = panfrost_gem_fault,
	.open = panfrost_gem_vm_open,
	.close = panfrost_gem_vm_close,
};

static int
dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma,
    unsigned long pgoff)
{
	int error;

	if (dmabuf == NULL || vma == NULL)
		return (EINVAL);

	if (dmabuf->ops->mmap == NULL)
		return (EINVAL);

	if (pgoff + vma_pages(vma) < pgoff)
		return (EOVERFLOW);

	if (pgoff + vma_pages(vma) > dmabuf->size >> PAGE_SHIFT)
		return (EINVAL);

	error = dmabuf->ops->mmap(dmabuf, vma);

	printf("%s: error %d\n", __func__, error);

	return (error);
}

int
drm_gem_shmem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct panfrost_gem_object *bo;
	int error;

	dprintf("%s\n", __func__);

	bo = (struct panfrost_gem_object *)obj;

	vma->vm_pgoff -= drm_vma_node_start(&obj->vma_node);

	if (obj->import_attach) {
		//drm_gem_object_put(obj);
		vma->vm_private_data = NULL;
		return dma_buf_mmap(obj->dma_buf, vma, 0);
	}

	error = panfrost_gem_get_pages(bo);
	if (error != 0) {
		printf("failed to get pages\n");
		return (-1);
	}

	vma->vm_flags |= VM_MIXEDMAP | VM_DONTEXPAND;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	if (!bo->map_cached)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &panfrost_gem_vm_ops;

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

static struct panfrost_gem_object *
panfrost_gem_create_object0(struct drm_device *dev, size_t size, bool private)
{
	struct panfrost_gem_object *obj;
	int error;

	obj = malloc(sizeof(*obj), M_DEVBUF, M_ZERO | M_WAITOK);
	obj->base.funcs = &panfrost_gem_funcs;
	TAILQ_INIT(&obj->mappings);
	mtx_init(&obj->mappings_lock, "mappings", NULL, MTX_DEF);

//printf("%s: private %d\n", __func__, private);

	if (private)
		drm_gem_private_object_init(dev, &obj->base, size);
	else
		drm_gem_object_init(dev, &obj->base, size);

	error = drm_gem_create_mmap_offset(&obj->base);
	if (error != 0) {
		printf("Failed to create mmap offset\n");
		return (NULL);
	}

	return (obj);
}

struct panfrost_gem_object *
panfrost_gem_create_object_with_handle(struct drm_file *file,
    struct drm_device *dev, size_t size, uint32_t flags, uint32_t *handle)
{
	struct panfrost_gem_object *obj;
	int error;

dprintf("%s\n", __func__);

	size = PAGE_ALIGN(size);

	obj = panfrost_gem_create_object0(dev, size, false);
	obj->noexec = !!(flags & PANFROST_BO_NOEXEC);
	obj->is_heap = !!(flags & PANFROST_BO_HEAP);

	error = drm_gem_handle_create(file, &obj->base, handle);
	drm_gem_object_put(&obj->base);
	if (error) {
		printf("Failed to create handle\n");
		return (NULL);
	}

	return (obj);
}

struct panfrost_gem_mapping *
panfrost_gem_mapping_get(struct panfrost_gem_object *bo,
    struct panfrost_file *file)
{
	struct panfrost_gem_mapping *mapping, *result;

	result = NULL;

	mtx_lock(&bo->mappings_lock);
	TAILQ_FOREACH(mapping, &bo->mappings, next) {
		if (mapping->mmu == &file->mmu) {
			result = mapping;
			break;
		}
	}
	mtx_unlock(&bo->mappings_lock);

	return (result);
}

int
panfrost_gem_get_pages(struct panfrost_gem_object *bo)
{
	vm_paddr_t low, high, boundary;
	struct drm_gem_object *obj;
	vm_memattr_t memattr;
	vm_page_t m;
	int alignment;
	int pflags;
	int npages;

	if (bo->pages != NULL)
		return (0);

	obj = &bo->base;
	npages = obj->size / PAGE_SIZE;

	alignment = PAGE_SIZE;
	low = 0;
	high = -1UL;
	boundary = 0;
	pflags = VM_ALLOC_NORMAL | VM_ALLOC_NOOBJ | VM_ALLOC_NOBUSY |
	    VM_ALLOC_WIRED | VM_ALLOC_ZERO;
	memattr = VM_MEMATTR_WRITE_COMBINING;

	m = vm_page_alloc_contig(NULL, 0, pflags, npages, low, high,
	    alignment, boundary, memattr);
	if (m == NULL)
		panic("could not allocate %d physical pages\n", npages);

	bo->pages = m;
	bo->npages = npages;

	int i;

	for (i = 0; i < npages; i++, m++) {
		if ((m->flags & PG_ZERO) == 0)
			pmap_zero_page(m);
		m->valid = VM_PAGE_BITS_ALL;
		m->oflags &= ~VPO_UNMANAGED;
		m->flags |= PG_FICTITIOUS;
	}

	wmb();

	return (0);
}

#if 0
static struct drm_gem_object *
panfrost_gem_create_object(struct drm_device *dev, size_t size)
{
	struct panfrost_gem_object *obj;

	obj = malloc(sizeof(*obj), M_DEVBUF, M_ZERO | M_WAITOK);
	obj->base.funcs = &panfrost_gem_funcs;
	TAILQ_INIT(&obj->mappings);
	mtx_init(&obj->mappings_lock, "mappings", NULL, MTX_DEF);

	return (&obj->base);
}
#endif

struct drm_gem_object *
panfrost_gem_prime_import_sg_table(struct drm_device *dev,
    struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	struct panfrost_gem_object *bo;
	size_t size;
	vm_page_t *m;
	int error;

	printf("%s size %d\n", __func__, attach->dmabuf->size);

	size = PAGE_ALIGN(attach->dmabuf->size);

	bo = panfrost_gem_create_object0(dev, size, true);
	printf("%s: bo %p\n", __func__, bo);

	m = malloc(sizeof(vm_page_t) * 4096, M_DEVBUF, M_ZERO | M_WAITOK);

	error = drm_prime_sg_to_page_addr_arrays(sgt, m, NULL, 4096);
	bo->pages = m[0];

	unsigned count;
	struct scatterlist *sg;
	u32 len, index;
 
	index = 0;
	for_each_sg(sgt->sgl, sg, sgt->nents, count) {
		len = sg_dma_len(sg);
		while (len > 0) {
			len -= PAGE_SIZE;
			index++;
		}
	}

	bo->npages = index;

	printf("npages %d\n", index);

	bo->noexec = true;

	return (&bo->base);
}
