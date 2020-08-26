/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 :* Copyright (c) 2020 Andrew Turner <andrew@FreeBSD.org>
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>

#include <arm64/arm64/gicv3_its.h>

#include <arm64/vmm/hyp.h>
#include <arm64/vmm/arm64.h>

#define	ITS_LOCK(its)		sx_xlock(&its->its_mtx)
#define	ITS_UNLOCK(its)		sx_xunlock(&its->its_mtx)
#define	ITS_ASSERT_LOCKED(its)	sx_assert(&its->its_mtx, SA_XLOCKED)

MALLOC_DEFINE(M_VITS, "ARM VMM VITS", "ARM VMM VITS");

static void
its_table_unmap(void *vm, struct vgic_its_table *tab)
{

	if (tab->valid) {
		vmm_unmap_gpa(vm, tab->vaddr, tab->size / PAGE_SIZE, tab->ma);
		free(tab->ma, M_VITS);
		kva_free(tab->vaddr, tab->size);
		tab->size = 0;
		tab->pbase = 0;
		tab->valid = 0;
	}
}

static void
its_table_map(void *vm, struct vgic_its_table *tab, bool valid, uint64_t paddr,
    size_t size)
{

	if (valid) {
		KASSERT((size & PAGE_MASK) == 0,
		   ("its_table_map: Size must be a multiple of the page size"));

		/*
		 * If the existing table is valid, the correct size, and
		 * points at the correct memory there is nothing to do.
		 */
		if (tab->valid && size == tab->size && paddr == tab->pbase)
			return;

		if (tab->valid) {
			/* Unmap the old table before remapping it */
			vmm_unmap_gpa(vm, tab->vaddr, tab->size / PAGE_SIZE,
			    tab->ma);
			free(tab->ma, M_VITS);
			kva_free(tab->vaddr, tab->size);
		}
		tab->pbase = paddr;
		tab->size = size;
		tab->vaddr = kva_alloc(tab->size);
		tab->ma = malloc((tab->size / PAGE_SIZE) * sizeof(*tab->ma),
		    M_VITS, M_ZERO | M_WAITOK);

		vmm_map_gpa(vm, tab->vaddr, tab->pbase, tab->size / PAGE_SIZE,
		    tab->ma);
		tab->valid = 1;
	} else if (tab->valid) {
		/* Move from valid -> invalid */
		vmm_unmap_gpa(vm, tab->vaddr, tab->size / PAGE_SIZE, tab->ma);
		free(tab->ma, M_VITS);
		kva_free(tab->vaddr, tab->size);
		tab->size = 0;
		tab->pbase = 0;
		tab->valid = 0;
	}
}

static bool
its_check_id(uint32_t id, int table, int entries)
{

	/* Check the ID ids valid based on what we report */
	switch(table) {
	case GITS_BASER_TYPE_DEV:
		if (id >= (1ul << 16))
			return (false);
		break;
	default:
		return (false);
	}


	/* Check the ID is valid based on what the guest has allocated */
	/* TODO: Add indirect support */
	if (id >= entries)
		return (false);

	return (true);
}

/*
 * Map a collection to a redistributor. We expect the processor number in
 * RDbase aas GITS_TYPER.PTA == 0.
 */
static bool
its_cmd_mapc(struct vm *vm, struct vgic_its *its, struct its_cmd *cmd)
{
	uint64_t rdbase;
	int col_idx;

	ITS_ASSERT_LOCKED(its);
	col_idx = CMD_COL_GET(cmd);
	if (col_idx >= its->col_entries)
		return (false);

	if (CMD_VALID_GET(cmd)) {
		rdbase = CMD_TARGET_GET(cmd);
		/* Check the redistributor is valid */
		if (rdbase >= vm_get_maxcpus(vm))
			return (false);

		if (its->collection[col_idx] != -1)
			return (false);

		its->collection[col_idx] = rdbase;
	} else {
		its->collection[col_idx] = -1;
	}

	return (true);
}

/*
 * Map a device table entry to an ITT (interrupt translation table).
 * We don't use the ITT, however the guest kernel still needs to map it.
 */
static bool
its_cmd_mapd(struct vgic_its *its, struct its_cmd *cmd)
{
	uint32_t devid;

	devid = CMD_DEVID_GET(cmd);
	if (!its_check_id(devid, GITS_BASER_TYPE_DEV, its->dev_tab_entries))
		return (false);

	/* TODO */

	return (true);
}

/* Map a device & event to an interrupt */
static bool
its_cmd_mapi(struct vgic_its *its, struct its_cmd *cmd)
{
	struct vgic_its_msi *msi;

	msi = malloc(sizeof(*msi), M_VITS, M_WAITOK | M_ZERO);
	msi->devid = CMD_DEVID_GET(cmd);
	msi->eventid = CMD_ID_GET(cmd);
	if (CMD_COMMAND_GET(cmd) == ITS_CMD_MAPTI)
		msi->pintr = CMD_PID_GET(cmd);
	else
		msi->pintr = CMD_ID_GET(cmd);
	/* TODO: Check col_idx is valid */
	msi->col_idx = CMD_COL_GET(cmd);
	SLIST_INSERT_HEAD(&its->its_msi, msi, next);

	return (true);
}

/*
 * Move an event to a new collection.
 */
static bool
its_cmd_movi(struct vm *vm, struct vgic_its *its, struct its_cmd *cmd)
{
	struct vgic_its_msi *msi;
	uint32_t devid, eventid;

	devid = CMD_DEVID_GET(cmd);
	eventid = CMD_ID_GET(cmd);
	SLIST_FOREACH(msi, &its->its_msi, next) {
		if (msi->devid == devid && msi->eventid == eventid) {
			/* TODO: Check col_idx is valid */
			msi->col_idx = CMD_COL_GET(cmd);
			return (true);
		}
	}
	return (false);
}

static void
its_process_cmds(struct vm *vm, struct vgic_its *its)
{
	struct its_cmd *cmdp, cmd;
	int i;
	bool success;

	ITS_ASSERT_LOCKED(its);
	while (GITS_CMD_OFFSET(its->gits_creadr) !=
	    GITS_CMD_OFFSET(its->gits_cwriter)) {
		cmdp = (struct its_cmd *)(its->cmd_tab.vaddr +
		    GITS_CMD_OFFSET(its->gits_creadr));

		/* Read the command, ensuring the endian is correct */
		for (i = 0; i < nitems(cmdp->cmd_dword); i++)
			cmd.cmd_dword[i] = le64toh(cmdp->cmd_dword[i]);

		success = false;
		switch(CMD_COMMAND_GET(&cmd)) {
		case ITS_CMD_MOVI:
			success = its_cmd_movi(vm, its, &cmd);
			break;
		case ITS_CMD_SYNC:
			success = true;
			break;
		case ITS_CMD_MAPD:
			success = its_cmd_mapd(its, &cmd);
			break;
		case ITS_CMD_MAPC:
			success = its_cmd_mapc(vm, its, &cmd);
			break;
		case ITS_CMD_MAPTI:
			success = its_cmd_mapi(its, &cmd);
			break;
		case ITS_CMD_MAPI:
			success = its_cmd_mapi(its, &cmd);
			break;
		case ITS_CMD_INV:
			success = true;
			break;
		case ITS_CMD_INVALL:
			success = true;
			break;
		default:
			break;
		}

		/* If there was an error stall command processing */
		if (!success) {
			its->gits_creadr |= GITS_CREADR_STALL;
			return;
		}

		/* Move to the next command */
		its->gits_creadr += GITS_CMD_SIZE;
		if (its->gits_creadr == its->cmd_tab.size)
			its->gits_creadr = 0;
	}
	/* Clear any previous stall */
	its->gits_creadr &= ~GITS_CREADR_STALL;
}

static int
its_read(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t *rval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_its *its = &hyp->vgic_its;
	bool *retu = arg;
	uint64_t reg;

	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < its->start || fault_ipa + size > its->end ||
	    (size != 4 && size != 8)) {
		return (EINVAL);
	}

	reg = fault_ipa - its->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch(reg) {
	case GITS_CTLR:
	case GITS_IIDR:
		/* TODO: Report a useful ID register */
		*rval = 0;
		*retu = false;
		return (0);
	case GITS_TYPER:
		ITS_LOCK(its);
		*rval = (its->col_entries << GITS_TYPER_HCC_SHIFT) |
		    (16 << GITS_TYPER_DEVB_SHIFT) | 1;
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_CBASER:
		ITS_LOCK(its);
		*rval = its->gits_cbaser;
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_CREADR:
		ITS_LOCK(its);
		*rval = its->gits_creadr;
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_BASER(0):
		/* Device table */
		ITS_LOCK(its);
		*rval = (its->dev_tab_valid ? GITS_BASER_VALID : 0) |
		    (GITS_BASER_CACHE_WAWB << GITS_BASER_CACHE_SHIFT) |
		    (GITS_BASER_TYPE_DEV << GITS_BASER_TYPE_SHIFT) |
		    (8ul << GITS_BASER_ESIZE_SHIFT) | /* 8 bytes per entry */
		    its->dev_tab.pbase |
		    (GITS_BASER_SHARE_OS << GITS_BASER_SHARE_SHIFT) |
		    (GITS_BASER_PSZ_64K << GITS_BASER_PSZ_SHIFT) |
		    (MAX(its->dev_tab_pages, 1) - 1);
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_BASER(1):
	case GITS_BASER(2):
	case GITS_BASER(3):
	case GITS_BASER(4):
	case GITS_BASER(5):
	case GITS_BASER(6):
	case GITS_BASER(7):
		*rval = 0;
		*retu = false;
		return (0);
	}

	panic("%s: %lx\n", __func__, fault_ipa - its->start);
}

static int
its_write(void *vm, int vcpuid, uint64_t fault_ipa, uint64_t wval,
    int size, void *arg)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_its *its = &hyp->vgic_its;
	bool *retu = arg;
	uint64_t reg;

	/* Check the register is one of ours and is the correct size */
	if (fault_ipa < its->start || fault_ipa + size > its->end ||
	    (size != 4 && size != 8)) {
		return (EINVAL);
	}

	reg = fault_ipa - its->start;
	/* Check the register is correctly aligned */
	if ((reg & (size - 1)) != 0)
		return (EINVAL);

	switch(reg) {
	case GITS_CTLR:
		*retu = false;
		return (0);
	case GITS_CBASER:
		/* Force outer-shareable */
		wval &= ~GITS_CBASER_SHARE_MASK;
		wval |= GITS_CBASER_SHARE_OS;

		ITS_LOCK(its);
		its_table_map(vm, &its->cmd_tab,
		    (wval & GITS_CBASER_VALID) != 0,
		    wval & GITS_CBASER_PA_MASK, GITS_CBASER_SIZE(wval));

		its->gits_cbaser = wval;
		/* GITS_CREADR is cleared to 0 on a write to GITS_CBASER */
		its->gits_creadr = 0;
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_CWRITER:
		ITS_LOCK(its);
		its->gits_cwriter = wval;
		its_process_cmds(vm, its);
		ITS_UNLOCK(its);
		*retu = false;
		return (0);
	case GITS_BASER(0):

		ITS_LOCK(its);
		/*
		 * If the address is valid, and the page size is correct
		 * set the page count so we can map it later.
		 */
		if ((wval & (GITS_BASER_VALID | GITS_BASER_PSZ_MASK)) ==
		    (GITS_BASER_VALID |
		     (GITS_BASER_PSZ_64K << GITS_BASER_PSZ_SHIFT))) {
			its->dev_tab_valid = 1;
			its->dev_tab_pages = (wval & GITS_BASER_SIZE_MASK) + 1;
			its->dev_tab_entries =
			    (its->dev_tab_pages * PAGE_SIZE_64K) / 8;
		} else {
			its->dev_tab_valid = 0;
			its->dev_tab_pages = 0;
			its->dev_tab_entries = 0;
		}

		/* Update the table mapping */
		its_table_map(vm, &its->dev_tab, its->dev_tab_valid,
		    wval & GITS_BASER_PA_MASK,
		    (size_t)its->dev_tab_pages * PAGE_SIZE_64K);

		ITS_UNLOCK(its);

		*retu = false;
		return (0);
	case GITS_BASER(1):
	case GITS_BASER(2):
	case GITS_BASER(3):
	case GITS_BASER(4):
	case GITS_BASER(5):
	case GITS_BASER(6):
	case GITS_BASER(7):
		/* Ignore writes */
		*retu = false;
		return (0);
	}
	panic("%s: %lx\n", __func__, fault_ipa - its->start);
}

int
vgic_its_raise_msi(struct vm *vm, uint64_t msg, uint64_t addr, uint32_t devid)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_its *its = &hyp->vgic_its;
	struct vgic_its_msi *msi;
	int error;
	int vcpu;

	error = EINVAL;
	ITS_LOCK(its);
	SLIST_FOREACH(msi, &its->its_msi, next) {
		if (msi->devid == devid && msi->eventid == msg) {
			vcpu = its->collection[msi->col_idx];
			if (vcpu != -1)
				error = vgic_v3_inject_lpi(&hyp->ctx[vcpu],
				    msi->pintr);
			break;
		}
	}
	ITS_UNLOCK(its);

	return (error);
}

void
vgic_its_vminit(void *arg)
{
	struct hyp *hyp = arg;
	struct vgic_its *its = &hyp->vgic_its;

	memset(its, 0, sizeof(*its));
	sx_init(&its->its_mtx, "vits lock");
}

int
vgic_its_attach_to_vm(struct vm *vm, uint64_t start, size_t size)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_its *its = &hyp->vgic_its;
	int i;

	its->start = start;
	its->end = start + size;
	SLIST_INIT(&its->its_msi);

	its->col_entries = vm_get_maxcpus(vm);
	its->collection = mallocarray(its->col_entries,
	    sizeof(*its->collection), M_VITS, M_WAITOK);

	for (i = 0; i < its->col_entries; i++)
		its->collection[i] = -1;

	vm_register_inst_handler(vm, start, size, its_read, its_write);

	return (0);
}

void
vgic_its_detach_from_vm(struct vm *vm)
{
	struct hyp *hyp = vm_get_cookie(vm);
	struct vgic_its *its = &hyp->vgic_its;
	struct vgic_its_msi *msi, *msi_tmp;

	/* TODO: Check if the VITS is attached */

	vm_deregister_inst_handler(vm, its->start, its->end - its->start);

	its_table_unmap(vm, &its->cmd_tab);
	its_table_unmap(vm, &its->dev_tab);

	free(its->collection, M_VITS);
	SLIST_FOREACH_SAFE(msi, &its->its_msi, next, msi_tmp) {
		SLIST_REMOVE(&its->its_msi, msi, vgic_its_msi, next);
		free(msi, M_VITS);
	}
}
