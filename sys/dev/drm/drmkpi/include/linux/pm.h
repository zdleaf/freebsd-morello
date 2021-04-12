/*-
 * Copyright (c) 2017 Mellanox Technologies, Ltd.
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
 *
 * $FreeBSD$
 */

#ifndef __DRMKPI_LINUX_PM_H__
#define	__DRMKPI_LINUX_PM_H__

struct _device;
struct dev_pm_ops {
    int (*prepare)(struct _device *dev);
    void (*complete)(struct _device *dev);
    int (*suspend)(struct _device *dev);
    int (*resume)(struct _device *dev);
    int (*freeze)(struct _device *dev);
    int (*thaw)(struct _device *dev);
    int (*poweroff)(struct _device *dev);
    int (*restore)(struct _device *dev);
    int (*suspend_late)(struct _device *dev);
    int (*resume_early)(struct _device *dev);
    int (*freeze_late)(struct _device *dev);
    int (*thaw_early)(struct _device *dev);
    int (*poweroff_late)(struct _device *dev);
    int (*restore_early)(struct _device *dev);
    int (*suspend_noirq)(struct _device *dev);
    int (*resume_noirq)(struct _device *dev);
    int (*freeze_noirq)(struct _device *dev);
    int (*thaw_noirq)(struct _device *dev);
    int (*poweroff_noirq)(struct _device *dev);
    int (*restore_noirq)(struct _device *dev);
    int (*runtime_suspend)(struct _device *dev);
    int (*runtime_resume)(struct _device *dev);
    int (*runtime_idle)(struct _device *dev);
};

#endif	/* __DRMKPI_LINUX_PM_H__ */
