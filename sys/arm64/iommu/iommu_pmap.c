/*-
 * Copyright (c) 1991 Regents of the University of California.
 * All rights reserved.
 * Copyright (c) 1994 John S. Dyson
 * All rights reserved.
 * Copyright (c) 1994 David Greenman
 * All rights reserved.
 * Copyright (c) 2003 Peter Wemm
 * All rights reserved.
 * Copyright (c) 2005-2010 Alan L. Cox <alc@cs.rice.edu>
 * All rights reserved.
 * Copyright (c) 2014 Andrew Turner
 * All rights reserved.
 * Copyright (c) 2014-2016 The FreeBSD Foundation
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * the Systems Programming Group of the University of Utah Computer
 * Science Department and William Jolitz of UUNET Technologies Inc.
 *
 * This software was developed by Andrew Turner under sponsorship from
 * the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	from:	@(#)pmap.c	7.7 (Berkeley)	5/12/91
 */
/*-
 * Copyright (c) 2003 Networks Associates Technology, Inc.
 * All rights reserved.
 *
 * This software was developed for the FreeBSD Project by Jake Burkholder,
 * Safeport Network Services, and Network Associates Laboratories, the
 * Security Research Division of Network Associates, Inc. under
 * DARPA/SPAWAR contract N66001-01-C-8035 ("CBOSS"), as part of the DARPA
 * CHATS research program.
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

/*
 *	Manages physical address maps.
 *
 *	Since the information managed by this module is
 *	also stored by the logical address mapping module,
 *	this module may throw away valid virtual-to-physical
 *	mappings at almost any time.  However, invalidations
 *	of virtual-to-physical mappings must be done as
 *	requested.
 *
 *	In order to cope with hardware architectures which
 *	make virtual-to-physical map invalidates expensive,
 *	this module may delay invalidate or reduced protection
 *	operations until such time as they are actually
 *	necessary.  This module is given full information as
 *	to which processors are currently using which maps,
 *	and to when physical maps must be made correct.
 */

#include "opt_vm.h"

#include <sys/param.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mman.h>
#include <sys/msgbuf.h>
#include <sys/mutex.h>
#include <sys/physmem.h>
#include <sys/proc.h>
#include <sys/rwlock.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/vmem.h>
#include <sys/vmmeter.h>
#include <sys/sched.h>
#include <sys/sysctl.h>
#include <sys/_unrhdr.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_kern.h>
#include <vm/vm_page.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_extern.h>
#include <vm/vm_pageout.h>
#include <vm/vm_pager.h>
#include <vm/vm_phys.h>
#include <vm/vm_radix.h>
#include <vm/vm_reserv.h>
#include <vm/vm_dumpset.h>
#include <vm/uma.h>

#include <machine/machdep.h>
#include <machine/md_var.h>
#include <machine/pcb.h>

#define	PMAP_ASSERT_STAGE1(pmap)	MPASS((pmap)->pm_stage == PM_STAGE1)
#define	PMAP_ASSERT_STAGE2(pmap)	MPASS((pmap)->pm_stage == PM_STAGE2)

#define	NL0PG		(PAGE_SIZE/(sizeof (pd_entry_t)))
#define	NL1PG		(PAGE_SIZE/(sizeof (pd_entry_t)))
#define	NL2PG		(PAGE_SIZE/(sizeof (pd_entry_t)))
#define	NL3PG		(PAGE_SIZE/(sizeof (pt_entry_t)))

#define	NUL0E		L0_ENTRIES
#define	NUL1E		(NUL0E * NL1PG)
#define	NUL2E		(NUL1E * NL2PG)

#define	pmap_l0_pindex(v)	(NUL2E + NUL1E + ((v) >> L0_SHIFT))
#define	pmap_l1_pindex(v)	(NUL2E + ((v) >> L1_SHIFT))
#define	pmap_l2_pindex(v)	((v) >> L2_SHIFT)

/* This code assumes all L1 DMAP entries will be used */
CTASSERT((DMAP_MIN_ADDRESS  & ~L0_OFFSET) == DMAP_MIN_ADDRESS);
CTASSERT((DMAP_MAX_ADDRESS  & ~L0_OFFSET) == DMAP_MAX_ADDRESS);

/*
 * This ASID allocator uses a bit vector ("asid_set") to remember which ASIDs
 * that it has currently allocated to a pmap, a cursor ("asid_next") to
 * optimize its search for a free ASID in the bit vector, and an epoch number
 * ("asid_epoch") to indicate when it has reclaimed all previously allocated
 * ASIDs that are not currently active on a processor.
 *
 * The current epoch number is always in the range [0, INT_MAX).  Negative
 * numbers and INT_MAX are reserved for special cases that are described
 * below.
 */
struct asid_set {
	int asid_bits;
	bitstr_t *asid_set;
	int asid_set_size;
	int asid_next;
	int asid_epoch;
	struct mtx asid_set_mutex;
};

static struct asid_set asids;
static struct asid_set vmids;

/*
 * A pmap's cookie encodes an ASID and epoch number.  Cookies for reserved
 * ASIDs have a negative epoch number, specifically, INT_MIN.  Cookies for
 * dynamically allocated ASIDs have a non-negative epoch number.
 *
 * An invalid ASID is represented by -1.
 *
 * There are two special-case cookie values: (1) COOKIE_FROM(-1, INT_MIN),
 * which indicates that an ASID should never be allocated to the pmap, and
 * (2) COOKIE_FROM(-1, INT_MAX), which indicates that an ASID should be
 * allocated when the pmap is next activated.
 */
#define	COOKIE_FROM(asid, epoch)	((long)((u_int)(asid) |	\
					    ((u_long)(epoch) << 32)))
#define	COOKIE_TO_ASID(cookie)		((int)(cookie))
#define	COOKIE_TO_EPOCH(cookie)		((int)((u_long)(cookie) >> 32))

static void pmap_alloc_asid(pmap_t pmap);

static vm_page_t _pmap_alloc_l3(pmap_t pmap, vm_pindex_t ptepindex,
		struct rwlock **lockp);

static void _pmap_unwire_l3(pmap_t pmap, vm_offset_t va, vm_page_t m,
    struct spglist *free);
static __inline vm_page_t pmap_remove_pt_page(pmap_t pmap, vm_offset_t va);

/*
 * These load the old table data and store the new value.
 * They need to be atomic as the System MMU may write to the table at
 * the same time as the CPU.
 */
#define	pmap_clear(table)		atomic_store_64(table, 0)
#define	pmap_clear_bits(table, bits)	atomic_clear_64(table, bits)
#define	pmap_load(table)		(*table)
#define	pmap_load_clear(table)		atomic_swap_64(table, 0)
#define	pmap_load_store(table, entry)	atomic_swap_64(table, entry)
#define	pmap_set_bits(table, bits)	atomic_set_64(table, bits)
#define	pmap_store(table, entry)	atomic_store_64(table, entry)

/********************/
/* Inline functions */
/********************/

static __inline void
pagecopy(void *s, void *d)
{

	memcpy(d, s, PAGE_SIZE);
}

static __inline pd_entry_t *
pmap_l0(pmap_t pmap, vm_offset_t va)
{

	return (&pmap->pm_l0[pmap_l0_index(va)]);
}

static __inline pd_entry_t *
pmap_l0_to_l1(pd_entry_t *l0, vm_offset_t va)
{
	pd_entry_t *l1;

	l1 = (pd_entry_t *)PHYS_TO_DMAP(pmap_load(l0) & ~ATTR_MASK);
	return (&l1[pmap_l1_index(va)]);
}

static __inline pd_entry_t *
pmap_l1(pmap_t pmap, vm_offset_t va)
{
	pd_entry_t *l0;

	l0 = pmap_l0(pmap, va);
	if ((pmap_load(l0) & ATTR_DESCR_MASK) != L0_TABLE)
		return (NULL);

	return (pmap_l0_to_l1(l0, va));
}

static __inline pd_entry_t *
pmap_l1_to_l2(pd_entry_t *l1p, vm_offset_t va)
{
	pd_entry_t l1, *l2p;

	l1 = pmap_load(l1p);

	/*
	 * The valid bit may be clear if pmap_update_entry() is concurrently
	 * modifying the entry, so for KVA only the entry type may be checked.
	 */
	KASSERT(va >= VM_MAX_USER_ADDRESS || (l1 & ATTR_DESCR_VALID) != 0,
	    ("%s: L1 entry %#lx for %#lx is invalid", __func__, l1, va));
	KASSERT((l1 & ATTR_DESCR_TYPE_MASK) == ATTR_DESCR_TYPE_TABLE,
	    ("%s: L1 entry %#lx for %#lx is a leaf", __func__, l1, va));
	l2p = (pd_entry_t *)PHYS_TO_DMAP(l1 & ~ATTR_MASK);
	return (&l2p[pmap_l2_index(va)]);
}

static __inline pd_entry_t *
pmap_l2(pmap_t pmap, vm_offset_t va)
{
	pd_entry_t *l1;

	l1 = pmap_l1(pmap, va);
	if ((pmap_load(l1) & ATTR_DESCR_MASK) != L1_TABLE)
		return (NULL);

	return (pmap_l1_to_l2(l1, va));
}

static __inline pt_entry_t *
pmap_l2_to_l3(pd_entry_t *l2p, vm_offset_t va)
{
	pd_entry_t l2;
	pt_entry_t *l3p;

	l2 = pmap_load(l2p);

	/*
	 * The valid bit may be clear if pmap_update_entry() is concurrently
	 * modifying the entry, so for KVA only the entry type may be checked.
	 */
	KASSERT(va >= VM_MAX_USER_ADDRESS || (l2 & ATTR_DESCR_VALID) != 0,
	    ("%s: L2 entry %#lx for %#lx is invalid", __func__, l2, va));
	KASSERT((l2 & ATTR_DESCR_TYPE_MASK) == ATTR_DESCR_TYPE_TABLE,
	    ("%s: L2 entry %#lx for %#lx is a leaf", __func__, l2, va));
	l3p = (pt_entry_t *)PHYS_TO_DMAP(l2 & ~ATTR_MASK);
	return (&l3p[pmap_l3_index(va)]);
}

/*
 * Returns the lowest valid pde for a given virtual address.
 * The next level may or may not point to a valid page or block.
 */
static __inline pd_entry_t *
pmap_pde(pmap_t pmap, vm_offset_t va, int *level)
{
	pd_entry_t *l0, *l1, *l2, desc;

	l0 = pmap_l0(pmap, va);
	desc = pmap_load(l0) & ATTR_DESCR_MASK;
	if (desc != L0_TABLE) {
		*level = -1;
		return (NULL);
	}

	l1 = pmap_l0_to_l1(l0, va);
	desc = pmap_load(l1) & ATTR_DESCR_MASK;
	if (desc != L1_TABLE) {
		*level = 0;
		return (l0);
	}

	l2 = pmap_l1_to_l2(l1, va);
	desc = pmap_load(l2) & ATTR_DESCR_MASK;
	if (desc != L2_TABLE) {
		*level = 1;
		return (l1);
	}

	*level = 2;
	return (l2);
}

/*
 * Returns the lowest valid pte block or table entry for a given virtual
 * address. If there are no valid entries return NULL and set the level to
 * the first invalid level.
 */
static __inline pt_entry_t *
pmap_pte(pmap_t pmap, vm_offset_t va, int *level)
{
	pd_entry_t *l1, *l2, desc;
	pt_entry_t *l3;

	l1 = pmap_l1(pmap, va);
	if (l1 == NULL) {
		*level = 0;
		return (NULL);
	}
	desc = pmap_load(l1) & ATTR_DESCR_MASK;
	if (desc == L1_BLOCK) {
		*level = 1;
		return (l1);
	}

	if (desc != L1_TABLE) {
		*level = 1;
		return (NULL);
	}

	l2 = pmap_l1_to_l2(l1, va);
	desc = pmap_load(l2) & ATTR_DESCR_MASK;
	if (desc == L2_BLOCK) {
		*level = 2;
		return (l2);
	}

	if (desc != L2_TABLE) {
		*level = 2;
		return (NULL);
	}

	*level = 3;
	l3 = pmap_l2_to_l3(l2, va);
	if ((pmap_load(l3) & ATTR_DESCR_MASK) != L3_PAGE)
		return (NULL);

	return (l3);
}

static __inline int
pmap_l3_valid(pt_entry_t l3)
{

	return ((l3 & ATTR_DESCR_MASK) == L3_PAGE);
}

CTASSERT(L1_BLOCK == L2_BLOCK);

static __inline void
pmap_resident_count_inc(pmap_t pmap, int count)
{

	PMAP_LOCK_ASSERT(pmap, MA_OWNED);
	pmap->pm_stats.resident_count += count;
}

static __inline void
pmap_resident_count_dec(pmap_t pmap, int count)
{

	PMAP_LOCK_ASSERT(pmap, MA_OWNED);
	KASSERT(pmap->pm_stats.resident_count >= count,
	    ("pmap %p resident count underflow %ld %d", pmap,
	    pmap->pm_stats.resident_count, count));
	pmap->pm_stats.resident_count -= count;
}

#if 0
/*
 *	Initialize the pmap module.
 *	Called by vm_init, to initialize any structures that the pmap
 *	system needs to map virtual memory.
 */
void
pmap_init(void)
{
	struct vm_phys_seg *seg, *next_seg;
	vm_size_t s;
#if 0
	struct md_page *pvh;
	uint64_t mmfr1;
	int i, pv_npg, vmid_bits;
#endif

	/*
	 * Are large page mappings enabled?
	 */
	TUNABLE_INT_FETCH("vm.pmap.superpages_enabled", &superpages_enabled);
	if (superpages_enabled) {
		KASSERT(MAXPAGESIZES > 1 && pagesizes[1] == 0,
		    ("pmap_init: can't assign to pagesizes[1]"));
		pagesizes[1] = L2_SIZE;
		KASSERT(MAXPAGESIZES > 2 && pagesizes[2] == 0,
		    ("pmap_init: can't assign to pagesizes[2]"));
		pagesizes[2] = L1_SIZE;
	}

#if 0
	/*
	 * Initialize the ASID allocator.
	 */
	pmap_init_asids(&asids,
	    (READ_SPECIALREG(tcr_el1) & TCR_ASID_16) != 0 ? 16 : 8);

	if (has_hyp()) {
		mmfr1 = READ_SPECIALREG(id_aa64mmfr1_el1);
		vmid_bits = 8;

		if (ID_AA64MMFR1_VMIDBits_VAL(mmfr1) ==
		    ID_AA64MMFR1_VMIDBits_16)
			vmid_bits = 16;
		pmap_init_asids(&vmids, vmid_bits);
	}
#endif
}
#endif

/***************************************************
 * Page table page management routines.....
 ***************************************************/
/*
 * Schedule the specified unused page table page to be freed.  Specifically,
 * add the page to the specified list of pages that will be released to the
 * physical memory manager after the TLB has been updated.
 */
static __inline void
pmap_add_delayed_free_list(vm_page_t m, struct spglist *free,
    boolean_t set_PG_ZERO)
{

	if (set_PG_ZERO)
		m->flags |= PG_ZERO;
	else
		m->flags &= ~PG_ZERO;
	SLIST_INSERT_HEAD(free, m, plinks.s.ss);
}

/***************************************************
 * Low level mapping routines.....
 ***************************************************/

/*
 * Decrements a page table page's reference count, which is used to record the
 * number of valid page table entries within the page.  If the reference count
 * drops to zero, then the page table page is unmapped.  Returns TRUE if the
 * page table page was unmapped and FALSE otherwise.
 */
static inline boolean_t
pmap_unwire_l3(pmap_t pmap, vm_offset_t va, vm_page_t m, struct spglist *free)
{

	--m->ref_count;
	if (m->ref_count == 0) {
		_pmap_unwire_l3(pmap, va, m, free);
		return (TRUE);
	} else
		return (FALSE);
}

static void
_pmap_unwire_l3(pmap_t pmap, vm_offset_t va, vm_page_t m, struct spglist *free)
{

	PMAP_LOCK_ASSERT(pmap, MA_OWNED);
	/*
	 * unmap the page table page
	 */
	if (m->pindex >= (NUL2E + NUL1E)) {
		/* l1 page */
		pd_entry_t *l0;

		l0 = pmap_l0(pmap, va);
		pmap_clear(l0);
	} else if (m->pindex >= NUL2E) {
		/* l2 page */
		pd_entry_t *l1;

		l1 = pmap_l1(pmap, va);
		pmap_clear(l1);
	} else {
		/* l3 page */
		pd_entry_t *l2;

		l2 = pmap_l2(pmap, va);
		pmap_clear(l2);
	}
	pmap_resident_count_dec(pmap, 1);
	if (m->pindex < NUL2E) {
		/* We just released an l3, unhold the matching l2 */
		pd_entry_t *l1, tl1;
		vm_page_t l2pg;

		l1 = pmap_l1(pmap, va);
		tl1 = pmap_load(l1);
		l2pg = PHYS_TO_VM_PAGE(tl1 & ~ATTR_MASK);
		pmap_unwire_l3(pmap, va, l2pg, free);
	} else if (m->pindex < (NUL2E + NUL1E)) {
		/* We just released an l2, unhold the matching l1 */
		pd_entry_t *l0, tl0;
		vm_page_t l1pg;

		l0 = pmap_l0(pmap, va);
		tl0 = pmap_load(l0);
		l1pg = PHYS_TO_VM_PAGE(tl0 & ~ATTR_MASK);
		pmap_unwire_l3(pmap, va, l1pg, free);
	}
#if 0
	pmap_invalidate_page(pmap, va);
#endif

	/*
	 * Put page on a list so that it is released after
	 * *ALL* TLB shootdown is done
	 */
	pmap_add_delayed_free_list(m, free, TRUE);
}

static int
iommu_pmap_pinit_stage(pmap_t pmap, enum pmap_stage stage, int levels)
{
	vm_page_t m;

	/*
	 * allocate the l0 page
	 */
	while ((m = vm_page_alloc(NULL, 0, VM_ALLOC_NORMAL |
	    VM_ALLOC_NOOBJ | VM_ALLOC_WIRED | VM_ALLOC_ZERO)) == NULL)
		vm_wait(NULL);

	pmap->pm_l0_paddr = VM_PAGE_TO_PHYS(m);
	pmap->pm_l0 = (pd_entry_t *)PHYS_TO_DMAP(pmap->pm_l0_paddr);

	if ((m->flags & PG_ZERO) == 0)
		pagezero(pmap->pm_l0);

	pmap->pm_root.rt_root = 0;
	bzero(&pmap->pm_stats, sizeof(pmap->pm_stats));
	pmap->pm_cookie = COOKIE_FROM(-1, INT_MAX);

	MPASS(levels == 3 || levels == 4);
	pmap->pm_levels = levels;
	pmap->pm_stage = stage;
	switch (stage) {
	case PM_STAGE1:
		pmap->pm_asid_set = &asids;
		break;
	case PM_STAGE2:
		pmap->pm_asid_set = &vmids;
		break;
	default:
		panic("%s: Invalid pmap type %d", __func__, stage);
		break;
	}

#if 0
	/* XXX Temporarily disable deferred ASID allocation. */
	pmap_alloc_asid(pmap);
#endif

	/*
	 * Allocate the level 1 entry to use as the root. This will increase
	 * the refcount on the level 1 page so it won't be removed until
	 * pmap_release() is called.
	 */
	if (pmap->pm_levels == 3) {
		PMAP_LOCK(pmap);
		m = _pmap_alloc_l3(pmap, NUL2E + NUL1E, NULL);
		PMAP_UNLOCK(pmap);
	}
	pmap->pm_ttbr = VM_PAGE_TO_PHYS(m);

	return (1);
}

int
iommu_pmap_pinit(pmap_t pmap)
{

	return (iommu_pmap_pinit_stage(pmap, PM_STAGE1, 4));
}

/*
 * This routine is called if the desired page table page does not exist.
 *
 * If page table page allocation fails, this routine may sleep before
 * returning NULL.  It sleeps only if a lock pointer was given.
 *
 * Note: If a page allocation fails at page table level two or three,
 * one or two pages may be held during the wait, only to be released
 * afterwards.  This conservative approach is easily argued to avoid
 * race conditions.
 */
static vm_page_t
_pmap_alloc_l3(pmap_t pmap, vm_pindex_t ptepindex, struct rwlock **lockp)
{
	vm_page_t m, l1pg, l2pg;

	PMAP_LOCK_ASSERT(pmap, MA_OWNED);

	/*
	 * Allocate a page table page.
	 */
	if ((m = vm_page_alloc(NULL, ptepindex, VM_ALLOC_NOOBJ |
	    VM_ALLOC_WIRED | VM_ALLOC_ZERO)) == NULL) {
		if (lockp != NULL) {
#if 0
			RELEASE_PV_LIST_LOCK(lockp);
#endif
			PMAP_UNLOCK(pmap);
			vm_wait(NULL);
			PMAP_LOCK(pmap);
		}

		/*
		 * Indicate the need to retry.  While waiting, the page table
		 * page may have been allocated.
		 */
		return (NULL);
	}
	if ((m->flags & PG_ZERO) == 0)
		pmap_zero_page(m);

	/*
	 * Because of AArch64's weak memory consistency model, we must have a
	 * barrier here to ensure that the stores for zeroing "m", whether by
	 * pmap_zero_page() or an earlier function, are visible before adding
	 * "m" to the page table.  Otherwise, a page table walk by another
	 * processor's MMU could see the mapping to "m" and a stale, non-zero
	 * PTE within "m".
	 */
	dmb(ishst);

	/*
	 * Map the pagetable page into the process address space, if
	 * it isn't already there.
	 */

	if (ptepindex >= (NUL2E + NUL1E)) {
		pd_entry_t *l0;
		vm_pindex_t l0index;

		l0index = ptepindex - (NUL2E + NUL1E);
		l0 = &pmap->pm_l0[l0index];
		pmap_store(l0, VM_PAGE_TO_PHYS(m) | L0_TABLE);
	} else if (ptepindex >= NUL2E) {
		vm_pindex_t l0index, l1index;
		pd_entry_t *l0, *l1;
		pd_entry_t tl0;

		l1index = ptepindex - NUL2E;
		l0index = l1index >> L0_ENTRIES_SHIFT;

		l0 = &pmap->pm_l0[l0index];
		tl0 = pmap_load(l0);
		if (tl0 == 0) {
			/* recurse for allocating page dir */
			if (_pmap_alloc_l3(pmap, NUL2E + NUL1E + l0index,
			    lockp) == NULL) {
				vm_page_unwire_noq(m);
				vm_page_free_zero(m);
				return (NULL);
			}
		} else {
			l1pg = PHYS_TO_VM_PAGE(tl0 & ~ATTR_MASK);
			l1pg->ref_count++;
		}

		l1 = (pd_entry_t *)PHYS_TO_DMAP(pmap_load(l0) & ~ATTR_MASK);
		l1 = &l1[ptepindex & Ln_ADDR_MASK];
		pmap_store(l1, VM_PAGE_TO_PHYS(m) | L1_TABLE);
	} else {
		vm_pindex_t l0index, l1index;
		pd_entry_t *l0, *l1, *l2;
		pd_entry_t tl0, tl1;

		l1index = ptepindex >> Ln_ENTRIES_SHIFT;
		l0index = l1index >> L0_ENTRIES_SHIFT;

		l0 = &pmap->pm_l0[l0index];
		tl0 = pmap_load(l0);
		if (tl0 == 0) {
			/* recurse for allocating page dir */
			if (_pmap_alloc_l3(pmap, NUL2E + l1index,
			    lockp) == NULL) {
				vm_page_unwire_noq(m);
				vm_page_free_zero(m);
				return (NULL);
			}
			tl0 = pmap_load(l0);
			l1 = (pd_entry_t *)PHYS_TO_DMAP(tl0 & ~ATTR_MASK);
			l1 = &l1[l1index & Ln_ADDR_MASK];
		} else {
			l1 = (pd_entry_t *)PHYS_TO_DMAP(tl0 & ~ATTR_MASK);
			l1 = &l1[l1index & Ln_ADDR_MASK];
			tl1 = pmap_load(l1);
			if (tl1 == 0) {
				/* recurse for allocating page dir */
				if (_pmap_alloc_l3(pmap, NUL2E + l1index,
				    lockp) == NULL) {
					vm_page_unwire_noq(m);
					vm_page_free_zero(m);
					return (NULL);
				}
			} else {
				l2pg = PHYS_TO_VM_PAGE(tl1 & ~ATTR_MASK);
				l2pg->ref_count++;
			}
		}

		l2 = (pd_entry_t *)PHYS_TO_DMAP(pmap_load(l1) & ~ATTR_MASK);
		l2 = &l2[ptepindex & Ln_ADDR_MASK];
		pmap_store(l2, VM_PAGE_TO_PHYS(m) | L2_TABLE);
	}

	pmap_resident_count_inc(pmap, 1);

	return (m);
}

static pd_entry_t *
pmap_alloc_l2(pmap_t pmap, vm_offset_t va, vm_page_t *l2pgp,
    struct rwlock **lockp)
{
	pd_entry_t *l1, *l2;
	vm_page_t l2pg;
	vm_pindex_t l2pindex;

retry:
	l1 = pmap_l1(pmap, va);
	if (l1 != NULL && (pmap_load(l1) & ATTR_DESCR_MASK) == L1_TABLE) {
		l2 = pmap_l1_to_l2(l1, va);
		if (va < VM_MAXUSER_ADDRESS) {
			/* Add a reference to the L2 page. */
			l2pg = PHYS_TO_VM_PAGE(pmap_load(l1) & ~ATTR_MASK);
			l2pg->ref_count++;
		} else
			l2pg = NULL;
	} else if (va < VM_MAXUSER_ADDRESS) {
		/* Allocate a L2 page. */
		l2pindex = pmap_l2_pindex(va) >> Ln_ENTRIES_SHIFT;
		l2pg = _pmap_alloc_l3(pmap, NUL2E + l2pindex, lockp);
		if (l2pg == NULL) {
			if (lockp != NULL)
				goto retry;
			else
				return (NULL);
		}
		l2 = (pd_entry_t *)PHYS_TO_DMAP(VM_PAGE_TO_PHYS(l2pg));
		l2 = &l2[pmap_l2_index(va)];
	} else
		panic("pmap_alloc_l2: missing page table page for va %#lx",
		    va);
	*l2pgp = l2pg;
	return (l2);
}

static vm_page_t
pmap_alloc_l3(pmap_t pmap, vm_offset_t va, struct rwlock **lockp)
{
	vm_pindex_t ptepindex;
	pd_entry_t *pde, tpde;
#ifdef INVARIANTS
	pt_entry_t *pte;
#endif
	vm_page_t m;
	int lvl;

	/*
	 * Calculate pagetable page index
	 */
	ptepindex = pmap_l2_pindex(va);
retry:
	/*
	 * Get the page directory entry
	 */
	pde = pmap_pde(pmap, va, &lvl);

	/*
	 * If the page table page is mapped, we just increment the hold count,
	 * and activate it. If we get a level 2 pde it will point to a level 3
	 * table.
	 */
	switch (lvl) {
	case -1:
		break;
	case 0:
#ifdef INVARIANTS
		pte = pmap_l0_to_l1(pde, va);
		KASSERT(pmap_load(pte) == 0,
		    ("pmap_alloc_l3: TODO: l0 superpages"));
#endif
		break;
	case 1:
#ifdef INVARIANTS
		pte = pmap_l1_to_l2(pde, va);
		KASSERT(pmap_load(pte) == 0,
		    ("pmap_alloc_l3: TODO: l1 superpages"));
#endif
		break;
	case 2:
		tpde = pmap_load(pde);
		if (tpde != 0) {
			m = PHYS_TO_VM_PAGE(tpde & ~ATTR_MASK);
			m->ref_count++;
			return (m);
		}
		break;
	default:
		panic("pmap_alloc_l3: Invalid level %d", lvl);
	}

	/*
	 * Here if the pte page isn't mapped, or if it has been deallocated.
	 */
	m = _pmap_alloc_l3(pmap, ptepindex, lockp);
	if (m == NULL && lockp != NULL)
		goto retry;

	return (m);
}

/***************************************************
 * Pmap allocation/deallocation routines.
 ***************************************************/

/*
 * Release any resources held by the given physical map.
 * Called when a pmap initialized by pmap_pinit is being released.
 * Should only be called if the map contains no valid mappings.
 */
void
iommu_pmap_release(pmap_t pmap)
{
	boolean_t rv;
	struct spglist free;
	struct asid_set *set;
	vm_page_t m;
	int asid;

	if (pmap->pm_levels != 4) {
		PMAP_ASSERT_STAGE2(pmap);
		KASSERT(pmap->pm_stats.resident_count == 1,
		    ("pmap_release: pmap resident count %ld != 0",
		    pmap->pm_stats.resident_count));
		KASSERT((pmap->pm_l0[0] & ATTR_DESCR_VALID) == ATTR_DESCR_VALID,
		    ("pmap_release: Invalid l0 entry: %lx", pmap->pm_l0[0]));

		SLIST_INIT(&free);
		m = PHYS_TO_VM_PAGE(pmap->pm_ttbr);
		PMAP_LOCK(pmap);
		rv = pmap_unwire_l3(pmap, 0, m, &free);
		PMAP_UNLOCK(pmap);
		MPASS(rv == TRUE);
		vm_page_free_pages_toq(&free, true);
	}

	KASSERT(pmap->pm_stats.resident_count == 0,
	    ("pmap_release: pmap resident count %ld != 0",
	    pmap->pm_stats.resident_count));
	KASSERT(vm_radix_is_empty(&pmap->pm_root),
	    ("pmap_release: pmap has reserved page table page(s)"));

	set = pmap->pm_asid_set;
	KASSERT(set != NULL, ("%s: NULL asid set", __func__));

	/*
	 * Allow the ASID to be reused. In stage 2 VMIDs we don't invalidate
	 * the entries when removing them so rely on a later tlb invalidation.
	 * this will happen when updating the VMID generation. Because of this
	 * we don't reuse VMIDs within a generation.
	 */
	if (pmap->pm_stage == PM_STAGE1) {
		mtx_lock_spin(&set->asid_set_mutex);
		if (COOKIE_TO_EPOCH(pmap->pm_cookie) == set->asid_epoch) {
			asid = COOKIE_TO_ASID(pmap->pm_cookie);
			KASSERT(asid >= ASID_FIRST_AVAILABLE &&
			    asid < set->asid_set_size,
			    ("pmap_release: pmap cookie has out-of-range asid"));
			bit_clear(set->asid_set, asid);
		}
		mtx_unlock_spin(&set->asid_set_mutex);
	}

	m = PHYS_TO_VM_PAGE(pmap->pm_l0_paddr);
	vm_page_unwire_noq(m);
	vm_page_free_zero(m);
}

/***************************************************
 * page management routines.
 ***************************************************/

/*
 * Add a single Mali GPU entry. This function does not sleep.
 */
int
pmap_gpu_enter(pmap_t pmap, vm_offset_t va, vm_paddr_t pa,
    vm_prot_t prot, u_int flags)
{
	pd_entry_t *pde;
	pt_entry_t new_l3, orig_l3;
	pt_entry_t *l3;
	vm_page_t mpte;
	pd_entry_t *l1p;
	pd_entry_t *l2p;
	int lvl;
	int rv;

	PMAP_ASSERT_STAGE1(pmap);
	KASSERT(pmap != kernel_pmap, ("kernel pmap used for GPU"));
	KASSERT(va < VM_MAXUSER_ADDRESS, ("wrong address space"));
	KASSERT((va & PAGE_MASK) == 0, ("va is misaligned"));
	KASSERT((pa & PAGE_MASK) == 0, ("pa is misaligned"));

	new_l3 = (pt_entry_t)(pa | ATTR_SH(ATTR_SH_IS) | L3_BLOCK);

	if ((prot & VM_PROT_WRITE) != 0)
		new_l3 |= ATTR_S2_S2AP(ATTR_S2_S2AP_WRITE);
	if ((prot & VM_PROT_READ) != 0)
		new_l3 |= ATTR_S2_S2AP(ATTR_S2_S2AP_READ);
	if ((prot & VM_PROT_EXECUTE) == 0)
		new_l3 |= ATTR_S2_XN(ATTR_S2_XN_ALL);

	CTR2(KTR_PMAP, "pmap_gpu_enter: %.16lx -> %.16lx", va, pa);

	PMAP_LOCK(pmap);

	/*
	 * In the case that a page table page is not
	 * resident, we are creating it here.
	 */
retry:
	pde = pmap_pde(pmap, va, &lvl);
	if (pde != NULL && lvl == 2) {
		l3 = pmap_l2_to_l3(pde, va);
	} else {
		mpte = _pmap_alloc_l3(pmap, pmap_l2_pindex(va), NULL);
		if (mpte == NULL) {
			CTR0(KTR_PMAP, "pmap_enter: mpte == NULL");
			rv = KERN_RESOURCE_SHORTAGE;
			goto out;
		}

		/*
		 * Ensure newly created l1, l2 are visible to GPU.
		 * l0 is already visible by similar call in panfrost driver.
		 * The cache entry for l3 handled below.
		 */

		l1p = pmap_l1(pmap, va);
		l2p = pmap_l2(pmap, va);
		cpu_dcache_wb_range((vm_offset_t)l1p, sizeof(pd_entry_t));
		cpu_dcache_wb_range((vm_offset_t)l2p, sizeof(pd_entry_t));

		goto retry;
	}

	orig_l3 = pmap_load(l3);
	KASSERT(!pmap_l3_valid(orig_l3), ("l3 is valid"));

	/* New mapping */
	pmap_store(l3, new_l3);

	cpu_dcache_wb_range((vm_offset_t)l3, sizeof(pt_entry_t));

	pmap_resident_count_inc(pmap, 1);
	dsb(ishst);

	rv = KERN_SUCCESS;
out:
	PMAP_UNLOCK(pmap);

	return (rv);
}

/*
 * Remove a single Mali GPU entry.
 */
int
pmap_gpu_remove(pmap_t pmap, vm_offset_t va)
{
	pd_entry_t *pde;
	pt_entry_t *pte;
	int lvl;
	int rc;

	KASSERT((va & PAGE_MASK) == 0, ("va is misaligned"));
	KASSERT(pmap != kernel_pmap, ("kernel pmap used for GPU"));

	PMAP_LOCK(pmap);

	pde = pmap_pde(pmap, va, &lvl);
	if (pde == NULL || lvl != 2) {
		rc = KERN_FAILURE;
		goto out;
	}

	pte = pmap_l2_to_l3(pde, va);

	pmap_resident_count_dec(pmap, 1);
	pmap_clear(pte);
	cpu_dcache_wb_range((vm_offset_t)pte, sizeof(pt_entry_t));
	rc = KERN_SUCCESS;

out:
	PMAP_UNLOCK(pmap);

	return (rc);
}

/*
 * Add a single SMMU entry. This function does not sleep.
 */
int
pmap_senter(pmap_t pmap, vm_offset_t va, vm_paddr_t pa,
    vm_prot_t prot, u_int flags)
{
	pd_entry_t *pde;
	pt_entry_t new_l3, orig_l3;
	pt_entry_t *l3;
	vm_page_t mpte;
	int lvl;
	int rv;

	PMAP_ASSERT_STAGE1(pmap);
	KASSERT(va < VM_MAXUSER_ADDRESS, ("wrong address space"));

	va = trunc_page(va);
	new_l3 = (pt_entry_t)(pa | ATTR_DEFAULT |
	    ATTR_S1_IDX(VM_MEMATTR_DEVICE) | L3_PAGE);
	if ((prot & VM_PROT_WRITE) == 0)
		new_l3 |= ATTR_S1_AP(ATTR_S1_AP_RO);
	new_l3 |= ATTR_S1_XN; /* Execute never. */
	new_l3 |= ATTR_S1_AP(ATTR_S1_AP_USER);
	new_l3 |= ATTR_S1_nG; /* Non global. */

	CTR2(KTR_PMAP, "pmap_senter: %.16lx -> %.16lx", va, pa);

	PMAP_LOCK(pmap);

	/*
	 * In the case that a page table page is not
	 * resident, we are creating it here.
	 */
retry:
	pde = pmap_pde(pmap, va, &lvl);
	if (pde != NULL && lvl == 2) {
		l3 = pmap_l2_to_l3(pde, va);
	} else {
		mpte = _pmap_alloc_l3(pmap, pmap_l2_pindex(va), NULL);
		if (mpte == NULL) {
			CTR0(KTR_PMAP, "pmap_enter: mpte == NULL");
			rv = KERN_RESOURCE_SHORTAGE;
			goto out;
		}
		goto retry;
	}

	orig_l3 = pmap_load(l3);
	KASSERT(!pmap_l3_valid(orig_l3), ("l3 is valid"));

	/* New mapping */
	pmap_store(l3, new_l3);
	pmap_resident_count_inc(pmap, 1);
	dsb(ishst);

	rv = KERN_SUCCESS;
out:
	PMAP_UNLOCK(pmap);

	return (rv);
}

/*
 * Remove a single SMMU entry.
 */
int
pmap_sremove(pmap_t pmap, vm_offset_t va)
{
	pt_entry_t *pte;
	int lvl;
	int rc;

	PMAP_LOCK(pmap);

	pte = pmap_pte(pmap, va, &lvl);
	KASSERT(lvl == 3,
	    ("Invalid SMMU pagetable level: %d != 3", lvl));

	if (pte != NULL) {
		pmap_resident_count_dec(pmap, 1);
		pmap_clear(pte);
		rc = KERN_SUCCESS;
	} else
		rc = KERN_FAILURE;

	PMAP_UNLOCK(pmap);

	return (rc);
}

/*
 * Remove all the allocated L1, L2 pages from SMMU pmap.
 * All the L3 entires must be cleared in advance, otherwise
 * this function panics.
 */
void
pmap_sremove_pages(pmap_t pmap)
{
	pd_entry_t l0e, *l1, l1e, *l2, l2e;
	pt_entry_t *l3, l3e;
	vm_page_t m, m0, m1;
	vm_offset_t sva;
	vm_paddr_t pa;
	vm_paddr_t pa0;
	vm_paddr_t pa1;
	int i, j, k, l;

	PMAP_LOCK(pmap);

	for (sva = VM_MINUSER_ADDRESS, i = pmap_l0_index(sva);
	    (i < Ln_ENTRIES && sva < VM_MAXUSER_ADDRESS); i++) {
		l0e = pmap->pm_l0[i];
		if ((l0e & ATTR_DESCR_VALID) == 0) {
			sva += L0_SIZE;
			continue;
		}
		pa0 = l0e & ~ATTR_MASK;
		m0 = PHYS_TO_VM_PAGE(pa0);
		l1 = (pd_entry_t *)PHYS_TO_DMAP(pa0);

		for (j = pmap_l1_index(sva); j < Ln_ENTRIES; j++) {
			l1e = l1[j];
			if ((l1e & ATTR_DESCR_VALID) == 0) {
				sva += L1_SIZE;
				continue;
			}
			if ((l1e & ATTR_DESCR_MASK) == L1_BLOCK) {
				sva += L1_SIZE;
				continue;
			}
			pa1 = l1e & ~ATTR_MASK;
			m1 = PHYS_TO_VM_PAGE(pa1);
			l2 = (pd_entry_t *)PHYS_TO_DMAP(pa1);

			for (k = pmap_l2_index(sva); k < Ln_ENTRIES; k++) {
				l2e = l2[k];
				if ((l2e & ATTR_DESCR_VALID) == 0) {
					sva += L2_SIZE;
					continue;
				}
				pa = l2e & ~ATTR_MASK;
				m = PHYS_TO_VM_PAGE(pa);
				l3 = (pt_entry_t *)PHYS_TO_DMAP(pa);

				for (l = pmap_l3_index(sva); l < Ln_ENTRIES;
				    l++, sva += L3_SIZE) {
					l3e = l3[l];
					if ((l3e & ATTR_DESCR_VALID) == 0)
						continue;
					panic("%s: l3e found for va %jx\n",
					    __func__, sva);
				}

				vm_page_unwire_noq(m1);
				vm_page_unwire_noq(m);
				pmap_resident_count_dec(pmap, 1);
				vm_page_free(m);
				pmap_clear(&l2[k]);
			}

			vm_page_unwire_noq(m0);
			pmap_resident_count_dec(pmap, 1);
			vm_page_free(m1);
			pmap_clear(&l1[j]);
		}

		pmap_resident_count_dec(pmap, 1);
		vm_page_free(m0);
		pmap_clear(&pmap->pm_l0[i]);
	}

	KASSERT(pmap->pm_stats.resident_count == 0,
	    ("Invalid resident count %jd", pmap->pm_stats.resident_count));

	PMAP_UNLOCK(pmap);
}

#if 0
/*
 *	pmap_zero_page zeros the specified hardware page by mapping
 *	the page into KVM and using bzero to clear its contents.
 */
void
pmap_zero_page(vm_page_t m)
{
	vm_offset_t va = PHYS_TO_DMAP(VM_PAGE_TO_PHYS(m));

	pagezero((void *)va);
}
#endif
