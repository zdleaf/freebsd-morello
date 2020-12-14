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

#define	ARM_MALI_LPAE_TTBR_ADRMODE_TABLE	(3 << 0)
#define	ARM_MALI_LPAE_TTBR_READ_INNER		(1 << 2)
#define	ARM_MALI_LPAE_TTBR_SHARE_OUTER		(1 << 4)

#define	ARM_LPAE_MAIR_ATTR_SHIFT(n)		((n) << 3)
#define	ARM_LPAE_MAIR_ATTR_MASK			0xff
#define	ARM_LPAE_MAIR_ATTR_DEVICE		0x04
#define	ARM_LPAE_MAIR_ATTR_NC			0x44
#define	ARM_LPAE_MAIR_ATTR_INC_OWBRWA		0xf4
#define	ARM_LPAE_MAIR_ATTR_WBRWA		0xff
#define	ARM_LPAE_MAIR_ATTR_IDX_NC		0
#define	ARM_LPAE_MAIR_ATTR_IDX_CACHE		1
#define	ARM_LPAE_MAIR_ATTR_IDX_DEV		2
#define	ARM_LPAE_MAIR_ATTR_IDX_INC_OCACHE	3

#define	ARM_MALI_LPAE_MEMATTR_IMP_DEF		0x88ULL
#define	ARM_MALI_LPAE_MEMATTR_WRITE_ALLOC	0x8DULL

static const char *
panfrost_mmu_exception_name(uint32_t exc_code)
{

	switch (exc_code) {
	case 0x00: return "NOT_STARTED/IDLE/OK";
	case 0x01: return "DONE";
	case 0x02: return "INTERRUPTED";
	case 0x03: return "STOPPED";
	case 0x04: return "TERMINATED";
	case 0x08: return "ACTIVE";

	case 0xC1: return "TRANSLATION_FAULT_LEVEL1";
	case 0xC2: return "TRANSLATION_FAULT_LEVEL2";
	case 0xC3: return "TRANSLATION_FAULT_LEVEL3";
	case 0xC4: return "TRANSLATION_FAULT_LEVEL4";
	case 0xC8: return "PERMISSION_FAULT";
	case 0xC9 ... 0xCF: return "PERMISSION_FAULT";
	case 0xD1: return "TRANSTAB_BUS_FAULT_LEVEL1";
	case 0xD2: return "TRANSTAB_BUS_FAULT_LEVEL2";
	case 0xD3: return "TRANSTAB_BUS_FAULT_LEVEL3";
	case 0xD4: return "TRANSTAB_BUS_FAULT_LEVEL4";
	case 0xD8: return "ACCESS_FLAG";
	case 0xD9 ... 0xDF: return "ACCESS_FLAG";
	case 0xE0 ... 0xE7: return "ADDRESS_SIZE_FAULT";
	case 0xE8 ... 0xEF: return "MEMORY_ATTRIBUTES_FAULT";
	}

	return "UNKNOWN";
}

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

	status = GPU_READ(sc, MMU_INT_RAWSTAT);

	printf("%s: status %x\n", __func__, status);

	i = 0;

	fault_status = GPU_READ(sc, AS_FAULTSTATUS(i));

	addr = GPU_READ(sc, AS_FAULTADDRESS_LO(i));
	addr |= (uint64_t)GPU_READ(sc, AS_FAULTADDRESS_HI(i)) << 32;

	exception_type = fault_status & 0xFF;
	access_type = (fault_status >> 8) & 0x3;
	source_id = (fault_status >> 16);

	if ((exception_type & 0xF8) == 0xC0)
		printf("%s: page fault at %lx\n", __func__, addr);
	else
		printf("%s: %s fault at %lx\n", __func__,
		    panfrost_mmu_exception_name(exception_type), addr);

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

	pmap_pinit(p);
	PMAP_LOCK_INIT(p);

	mmu->as = -1;

	return (0);
}

static int
wait_ready(struct panfrost_softc *sc, uint32_t as)
{
	uint32_t reg;
	int timeout;

	timeout = 1000;

	do {
		reg = GPU_READ(sc, AS_STATUS(as));
		if ((reg & AS_STATUS_AS_ACTIVE) == 0)
			break;
	} while (timeout--);

	if (timeout <= 0)
		panic("failed to read");

	return (0);
}

static int
write_cmd(struct panfrost_softc *sc, uint32_t as, uint32_t cmd)
{
	int status;

	status = wait_ready(sc, as);
	if (status == 0)
		GPU_WRITE(sc, AS_COMMAND(as), cmd);

	return (status);
}

static void
lock_region(struct panfrost_softc *sc, uint32_t as, vm_offset_t va,
    size_t size)
{
	uint8_t region_width;
	uint64_t region;

	region = va & PAGE_MASK;

	size = round_up(size, PAGE_SIZE);

	region_width = 10 + fls(size >> PAGE_SHIFT);
	if ((size >> PAGE_SHIFT) != (1ul << (region_width - 11)))
		region_width += 1;
	region |= region_width;

	GPU_WRITE(sc, AS_LOCKADDR_LO(as), region & 0xFFFFFFFFUL);
	GPU_WRITE(sc, AS_LOCKADDR_HI(as), (region >> 32) & 0xFFFFFFFFUL);
	write_cmd(sc, as, AS_COMMAND_LOCK);
}

static int
mmu_hw_do_operation_locked(struct panfrost_softc *sc, uint32_t as,
    vm_offset_t va, size_t size, uint32_t op)
{
	int error;

	if (op != AS_COMMAND_UNLOCK)
		lock_region(sc, as, va, size);

	write_cmd(sc, as, op);

	error = wait_ready(sc, as);

	return (0);
}

#if 0
static int
mmu_hw_do_operation(struct panfrost_softc *sc,
    struct panfrost_mmu *mmu, vm_offset_t va, size_t size, uint32_t op)
{

	mtx_lock(sc->as_mtx);
	mtx_unlock(sc->as_mtx);
}
#endif

int
panfrost_mmu_enable(struct panfrost_softc *sc, struct panfrost_mmu *mmu)
{
	vm_paddr_t paddr;
	uint64_t memattr;
	pmap_t p;
	int as;

	as = mmu->as;
	p = &mmu->p;

	paddr = p->pm_l0_paddr;
	paddr |= ARM_MALI_LPAE_TTBR_READ_INNER;
	paddr |= ARM_MALI_LPAE_TTBR_ADRMODE_TABLE;

	memattr = (ARM_MALI_LPAE_MEMATTR_IMP_DEF
	     << ARM_LPAE_MAIR_ATTR_SHIFT(ARM_LPAE_MAIR_ATTR_IDX_NC)) |
	    (ARM_MALI_LPAE_MEMATTR_WRITE_ALLOC
	     << ARM_LPAE_MAIR_ATTR_SHIFT(ARM_LPAE_MAIR_ATTR_IDX_CACHE)) |
	    (ARM_MALI_LPAE_MEMATTR_IMP_DEF
	     << ARM_LPAE_MAIR_ATTR_SHIFT(ARM_LPAE_MAIR_ATTR_IDX_DEV));

	mmu_hw_do_operation_locked(sc, as, 0, ~0UL, AS_COMMAND_FLUSH_MEM);

	GPU_WRITE(sc, AS_TRANSTAB_LO(as), paddr & 0xffffffffUL);
	GPU_WRITE(sc, AS_TRANSTAB_HI(as), paddr >> 32);

	GPU_WRITE(sc, AS_MEMATTR_LO(as), memattr & 0xffffffffUL);
	GPU_WRITE(sc, AS_MEMATTR_HI(as), memattr >> 32);

	write_cmd(sc, as, AS_COMMAND_UPDATE);

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

	panfrost_mmu_enable(sc, mmu);

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
