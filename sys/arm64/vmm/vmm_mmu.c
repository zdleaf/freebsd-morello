/*
 * Copyright (C) 2017 Alexandru Elisei <alexandru.elisei@gmail.com>
 * All rights reserved.
 *
 * This software was developed by Alexandru Elisei under sponsorship
 * from the FreeBSD Foundation.
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>
#include <vm/vm_param.h>
#include <vm/vm_phys.h>

#include <machine/machdep.h>
#include <machine/vm.h>
#include <machine/vmm.h>
#include <machine/vmparam.h>
#include <machine/pmap.h>

#include "mmu.h"
#include "arm64.h"

MALLOC_DECLARE(M_HYP);

struct pmap		hyp_pmap_store;
#define	hyp_pmap	(&hyp_pmap_store)

void
vmmpmap_init(void)
{
	pmap_pinit_stage(hyp_pmap, PM_STAGE1_EL2, 4);
	PMAP_LOCK_INIT(hyp_pmap);
}

void
vmmpmap_fini(void)
{
	/* TODO */
}

uint64_t
vmmpmap_to_ttbr0(void)
{

	return (hyp_pmap->pm_ttbr);
}

/*
 * Creates an EL2 entry in the hyp_pmap. Similar to pmap_kenter.
 */
void
vmmpmap_enter(vm_offset_t va, vm_size_t size, vm_paddr_t pa, vm_prot_t prot)
{
	struct vm_page m;

	KASSERT((pa & L3_OFFSET) == 0,
	   ("%s: Invalid physical address", __func__));
	KASSERT((va & L3_OFFSET) == 0,
	   ("%s: Invalid virtual address", __func__));
	KASSERT((size & PAGE_MASK) == 0,
	    ("%s: Mapping is not page-sized", __func__));

	memset(&m, 0, sizeof(m));
	m.oflags = VPO_UNMANAGED;
	m.md.pv_memattr = VM_MEMATTR_DEFAULT;
	while (size > 0) {
		m.phys_addr = pa;
		pmap_enter(hyp_pmap, va, &m, prot, PMAP_ENTER_WIRED, prot);
		size -= PAGE_SIZE;
		pa += PAGE_SIZE;
		va += PAGE_SIZE;
	}
}

void
vmmpmap_remove(vm_offset_t va, vm_size_t size)
{
	KASSERT((va & L3_OFFSET) == 0,
	   ("%s: Invalid virtual address", __func__));
	KASSERT((size & PAGE_MASK) == 0,
	    ("%s: Mapping is not page-sized", __func__));

	pmap_remove(hyp_pmap, va, va + size);
	/* TODO: Invalidate the EL2 TLB */
}
