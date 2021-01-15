/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2018 Mellanox Technologies, Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/sysctl.h>
#include <sys/vnode.h>
#include <sys/file.h>

#include <machine/vmparam.h>

#include <drmkpi/uaccess.h>

SYSCTL_NODE(_compat, OID_AUTO, drmkpi, CTLFLAG_RW, 0, "DRMKPI parameters");

MALLOC_DEFINE(M_DRMKMALLOC, "drmkpi", "DRM kmalloc compat");

unsigned long linux_timer_hz_mask;

#include <linux/rbtree.h>
/* Undo Linux compat changes. */
#undef RB_ROOT
#define	RB_ROOT(head)	(head)->rbh_root

int
drmkpi_panic_cmp(struct rb_node *one, struct rb_node *two)
{
	panic("no cmp");
}

RB_GENERATE(drmkpi_root, rb_node, __entry, drmkpi_panic_cmp);

int
drmkpi_copyin(const void *uaddr, void *kaddr, size_t len)
{

	return (-copyin(uaddr, kaddr, len));
}

int
drmkpi_copyout(const void *kaddr, void *uaddr, size_t len)
{

	return (-copyout(kaddr, uaddr, len));
}

size_t
drmkpi_clear_user(void *_uaddr, size_t _len)
{
	uint8_t *uaddr = _uaddr;
	size_t len = _len;

	/* make sure uaddr is aligned before going into the fast loop */
	while (((uintptr_t)uaddr & 7) != 0 && len > 7) {
		if (subyte(uaddr, 0))
			return (_len);
		uaddr++;
		len--;
	}

	/* zero 8 bytes at a time */
	while (len > 7) {
#ifdef __LP64__
		if (suword64(uaddr, 0))
			return (_len);
#else
		if (suword32(uaddr, 0))
			return (_len);
		if (suword32(uaddr + 4, 0))
			return (_len);
#endif
		uaddr += 8;
		len -= 8;
	}

	/* zero fill end, if any */
	while (len > 0) {
		if (subyte(uaddr, 0))
			return (_len);
		uaddr++;
		len--;
	}
	return (0);
}

int
drmkpi_access_ok(const void *uaddr, size_t len)
{
	uintptr_t saddr;
	uintptr_t eaddr;

	/* get start and end address */
	saddr = (uintptr_t)uaddr;
	eaddr = (uintptr_t)uaddr + len;

	/* verify addresses are valid for userspace */
	return ((saddr == eaddr) ||
	    (eaddr > saddr && eaddr <= VM_MAXUSER_ADDRESS));
}

struct inode;
unsigned int
drmkpi_iminor(struct inode *inode)
{
	struct vnode *vnode;

	vnode = (struct vnode *)inode;
	if (vnode == NULL || vnode->v_rdev == NULL)
		return (-1U);

	return (vnode->v_rdev->si_drv0);
}

/*
 * NOTE: Linux frequently uses "unsigned long" for pointer to integer
 * conversion and vice versa, where in FreeBSD "uintptr_t" would be
 * used. Assert these types have the same size, else some parts of the
 * DRMKPI may not work like expected:
 */
CTASSERT(sizeof(unsigned long) == sizeof(uintptr_t));

/* greatest common divisor, Euclid equation */
static uint64_t
lkpi_gcd_64(uint64_t a, uint64_t b)
{
	uint64_t an;
	uint64_t bn;

	while (b != 0) {
		an = b;
		bn = a % b;
		a = an;
		b = bn;
	}
	return (a);
}

uint64_t lkpi_nsec2hz_rem;
uint64_t lkpi_nsec2hz_div = 1000000000ULL;
uint64_t lkpi_nsec2hz_max;

uint64_t lkpi_usec2hz_rem;
uint64_t lkpi_usec2hz_div = 1000000ULL;
uint64_t lkpi_usec2hz_max;

uint64_t lkpi_msec2hz_rem;
uint64_t lkpi_msec2hz_div = 1000ULL;
uint64_t lkpi_msec2hz_max;

static void
linux_timer_init(void *arg)
{
	uint64_t gcd;

	/*
	 * Compute an internal HZ value which can divide 2**32 to
	 * avoid timer rounding problems when the tick value wraps
	 * around 2**32:
	 */
	linux_timer_hz_mask = 1;
	while (linux_timer_hz_mask < (unsigned long)hz)
		linux_timer_hz_mask *= 2;
	linux_timer_hz_mask--;

	/* compute some internal constants */

	lkpi_nsec2hz_rem = hz;
	lkpi_usec2hz_rem = hz;
	lkpi_msec2hz_rem = hz;

	gcd = lkpi_gcd_64(lkpi_nsec2hz_rem, lkpi_nsec2hz_div);
	lkpi_nsec2hz_rem /= gcd;
	lkpi_nsec2hz_div /= gcd;
	lkpi_nsec2hz_max = -1ULL / lkpi_nsec2hz_rem;

	gcd = lkpi_gcd_64(lkpi_usec2hz_rem, lkpi_usec2hz_div);
	lkpi_usec2hz_rem /= gcd;
	lkpi_usec2hz_div /= gcd;
	lkpi_usec2hz_max = -1ULL / lkpi_usec2hz_rem;

	gcd = lkpi_gcd_64(lkpi_msec2hz_rem, lkpi_msec2hz_div);
	lkpi_msec2hz_rem /= gcd;
	lkpi_msec2hz_div /= gcd;
	lkpi_msec2hz_max = -1ULL / lkpi_msec2hz_rem;
}
SYSINIT(linux_timer, SI_SUB_DRIVERS, SI_ORDER_FIRST, linux_timer_init, NULL);
