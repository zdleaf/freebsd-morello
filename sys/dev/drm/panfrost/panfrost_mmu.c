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

int
panfrost_mmu_pgtable_alloc(struct panfrost_file *pfile)
{
	struct panfrost_mmu *mmu;
	pmap_t p;

	mmu = &pfile->mmu;
	p = &mmu->p;

	printf("p is %p\n", p);
	p->pm_l0_paddr = 1;
	printf("p is %p\n", p);

	pmap_pinit(p);
	PMAP_LOCK_INIT(p);

	return (0);
}

int
panfrost_mmu_map(struct panfrost_gem_mapping *mapping)
{
	struct panfrost_gem_object *bo;
	vm_paddr_t low, high, boundary;
	struct drm_gem_object *obj;
	struct panfrost_mmu *mmu;
	vm_memattr_t memattr;
	vm_prot_t prot;
	vm_offset_t va;
	vm_page_t m;
	vm_page_t *ma;
	int pflags;
	int alignment;
	int npages;
	vm_paddr_t pa;
	int error;
	int i;

	bo = mapping->obj;
	obj = &bo->base;
	mmu = mapping->mmu;

	alignment = PAGE_SIZE;
	low = 0;
	high = -1UL;
	boundary = 0;
	pflags = VM_ALLOC_NORMAL  | VM_ALLOC_NOOBJ | VM_ALLOC_NOBUSY |
	    VM_ALLOC_WIRED | VM_ALLOC_ZERO;
	memattr = VM_MEMATTR_DEFAULT;

	npages = obj->size / PAGE_SIZE;

	m = vm_page_alloc_contig(NULL, 0, pflags, npages, low, high,
	    alignment, boundary, memattr);
	if (m == NULL)
		panic("could not allocate %d physical pages\n", npages);
	bo->pages = m;

	ma = &m;

	va = mapping->mmnode.start << PAGE_SHIFT;
	prot = VM_PROT_READ | VM_PROT_WRITE;
	if (bo->noexec == 0)
		prot |= VM_PROT_EXECUTE;

	/* map pages */
	for (i = 0; i < npages; i++) {
		pa = VM_PAGE_TO_PHYS(ma[i]);
		error = pmap_senter(&mmu->p, va, pa, prot, 0);
		va += PAGE_SIZE;
	}

	mapping->active = true;

	return (0);
}
