/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
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
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/priv.h>
#include <sys/proc.h>
#include <sys/signalvar.h>
#include <sys/systm.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_phys.h>
#include <vm/vm_extern.h>

static struct cdev *fmemdev;

struct fmem_request {
	uint32_t addr;
	uint32_t data;
	uint32_t access_width;
};

#define	FMEM_READ	_IOWR('X', 1, struct fmem_request)
#define FMEM_WRITE	_IOWR('X', 2, struct fmem_request)

static int
fmemopen(struct cdev *dev __unused, int flags, int fmt __unused,
    struct thread *td)
{

	return (0);
}

static int
fmemioctl(struct cdev *dev, u_long cmd, caddr_t data, int flags,
    struct thread *td)
{
	struct fmem_request *req;
	uint64_t vaddr;
	uint32_t offset;
	uint32_t base;

	req = (struct fmem_request *)data;
	base = trunc_page(req->addr);
	offset = req->addr & (PAGE_SIZE - 1);

	vaddr = kva_alloc(PAGE_SIZE);
	pmap_kenter(vaddr, PAGE_SIZE, base, VM_MEMATTR_DEVICE);

	switch (cmd) {
	case FMEM_READ:
		switch (req->access_width) {
		case 1:
			req->data = *(volatile uint8_t *)(vaddr + offset);
			break;
		case 2:
			req->data = *(volatile uint16_t *)(vaddr + offset);
			break;
		case 4:
			req->data = *(volatile uint32_t *)(vaddr + offset);
			break;
		}
		break;
	case FMEM_WRITE:
		switch (req->access_width) {
		case 1:
			*(volatile uint8_t *)(vaddr + offset) = req->data;
			break;
		case 2:
			*(volatile uint16_t *)(vaddr + offset) = req->data;
			break;
		case 4:
			*(volatile uint32_t *)(vaddr + offset) = req->data;
			break;
		}
	}

	pmap_kremove(vaddr);
	kva_free(vaddr, PAGE_SIZE);

	return (0);
}

static struct cdevsw fmem_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	0,
	.d_open =	fmemopen,
	.d_read =	NULL,
	.d_write =	NULL,
	.d_ioctl =	fmemioctl,
	.d_mmap =	NULL,
	.d_name =	"fmem",
};

static int
fmem_modevent(module_t mod __unused, int type, void *data __unused)
{

	switch(type) {
	case MOD_LOAD:
		fmemdev = make_dev(&fmem_cdevsw, 0, UID_ROOT, GID_KMEM, 0640,
		    "fmem");
		break;
	case MOD_UNLOAD:
		destroy_dev(fmemdev);
		break;
	case MOD_SHUTDOWN:
		break;
	default:
		return(EOPNOTSUPP);
	}

	return (0);
}

DEV_MODULE(fmem, fmem_modevent, NULL);
MODULE_VERSION(fmem, 1);
