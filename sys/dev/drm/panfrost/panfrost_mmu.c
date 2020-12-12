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

void
panfrost_mmu_intr(void *arg)
{
	struct panfrost_softc *sc;
	uint32_t fault_status;
	uint32_t exception_type;
	uint32_t access_type;
	uint32_t source_id;
	uint32_t status;
	uint64_t addr;
	int i;

	sc = arg;

	printf("%s\n", __func__);

	status = GPU_READ(sc, MMU_INT_RAWSTAT);

	printf("%s: status %x\n", __func__, status);

	i = 0;

	fault_status = GPU_READ(sc, AS_FAULTSTATUS(i));
	printf("%s: fault status %x\n", __func__, fault_status);

	addr = GPU_READ(sc, AS_FAULTADDRESS_LO(i));
	addr |= (uint64_t)GPU_READ(sc, AS_FAULTADDRESS_HI(i)) << 32;

	exception_type = fault_status & 0xFF;
	access_type = (fault_status >> 8) & 0x3;
	source_id = (fault_status >> 16);

	if ((exception_type & 0xF8) == 0xC0)
		printf("%s: page fault at %lx\n", __func__, addr);
	else
		printf("%s: fault at %lx\n", __func__, addr);

	printf("%s: exception type %x, access type %x, source id %x \n",
	    __func__, exception_type, access_type, source_id);
}

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

	mmu->as = -1;

	return (0);
}

int
panfrost_mmu_enable(struct panfrost_softc *sc, struct panfrost_mmu *mmu)
{

	return (0);
}

uint32_t
panfrost_mmu_as_get(struct panfrost_softc *sc, struct panfrost_mmu *mmu)
{
	int as;

	if (mmu->as >= 0) {
		printf("mmu is running\n");
		panic("mmu is running");
	}

	mtx_lock_spin(&sc->as_mtx);
	as = ffz(sc->as_alloc_set);
	sc->as_alloc_set |= (1 << as);
	mtx_unlock_spin(&sc->as_mtx);

	mmu->as = as;

	return (as);
}

int
panfrost_mmu_map(struct panfrost_gem_mapping *mapping)
{
	struct panfrost_gem_object *bo;
	struct panfrost_mmu *mmu;
	vm_prot_t prot;
	vm_offset_t va;
	vm_page_t *ma;
	vm_page_t m;
	vm_paddr_t pa;
	int error;
	int i;

	bo = mapping->obj;
	mmu = mapping->mmu;

	error = panfrost_gem_get_pages(bo);
	if (error != 0)
		panic("could not get pages");

	m = bo->pages;
	ma = &m;

	va = mapping->mmnode.start << PAGE_SHIFT;
	prot = VM_PROT_READ | VM_PROT_WRITE;
	if (bo->noexec == 0)
		prot |= VM_PROT_EXECUTE;

	/* map pages */
	for (i = 0; i < bo->npages; i++) {
		pa = VM_PAGE_TO_PHYS(ma[i]);
		error = pmap_senter(&mmu->p, va, pa, prot, 0);
		va += PAGE_SIZE;
	}

	mapping->active = true;

	return (0);
}

int
panfrost_mmu_init(struct panfrost_softc *sc)
{

	/* Enable interrupts. */
	GPU_WRITE(sc, MMU_INT_CLEAR, ~0);
	GPU_WRITE(sc, MMU_INT_MASK, ~0);

	return (0);
}
