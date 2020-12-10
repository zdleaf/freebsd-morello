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
#include "panfrost_job.h"

static void
panfrost_acquire_object_fences(struct drm_gem_object **bos,
    int bo_count, struct dma_fence **implicit_fences)
{
	int i;

	for (i = 0; i < bo_count; i++)
		implicit_fences[i] =
		    reservation_object_get_excl_rcu(bos[i]->resv);
}

static int
panfrost_job_get_slot(struct panfrost_job *job)
{

	if (job->requirements & PANFROST_JD_REQ_FS)
		return (0);	/* fragment job */

	return (1); /* vertex job */
}

static void
panfrost_job_hw_submit(struct panfrost_job *job, int slot)
{
	struct panfrost_softc *sc;
	uint32_t cfg;
	uint64_t jc_head;

	sc = job->sc;
	jc_head = job->jc;

	cfg = panfrost_mmu_as_get(sc, &job->pfile->mmu);

	GPU_WRITE(sc, JS_HEAD_NEXT_LO(slot), jc_head & 0xFFFFFFFF);
	GPU_WRITE(sc, JS_HEAD_NEXT_HI(slot), jc_head >> 32);

	cfg |= JS_CONFIG_THREAD_PRI(8) |
	    JS_CONFIG_START_FLUSH_CLEAN_INVALIDATE |
	    JS_CONFIG_END_FLUSH_CLEAN_INVALIDATE;

	if (panfrost_has_hw_feature(sc, HW_FEATURE_FLUSH_REDUCTION))
		cfg |= JS_CONFIG_ENABLE_FLUSH_REDUCTION;

	if (panfrost_has_hw_issue(sc, HW_ISSUE_10649))
		cfg |= JS_CONFIG_START_MMU;

	GPU_WRITE(sc, JS_CONFIG_NEXT(slot), cfg);

	if (panfrost_has_hw_feature(sc, HW_FEATURE_FLUSH_REDUCTION))
		GPU_WRITE(sc, JS_FLUSH_ID_NEXT(slot), job->flush_id);

	GPU_WRITE(sc, JS_COMMAND_NEXT(slot), JS_COMMAND_START);
}

int
panfrost_job_push(struct panfrost_job *job)
{
	struct ww_acquire_ctx acquire_ctx;
	struct panfrost_softc *sc;
	int slot;
	int error;

	sc = job->sc;

	error = drm_gem_lock_reservations(job->bos, job->bo_count,
	    &acquire_ctx);
	if (error)
		return (error);

	/* Acquire a reference to fence. */
	//job->render_done_fence = dma_fence_get(&job->base.s_fence->finished);

	panfrost_acquire_object_fences(job->bos, job->bo_count,
	    job->implicit_fences);

	drm_gem_unlock_reservations(job->bos, job->bo_count, &acquire_ctx);

	slot = panfrost_job_get_slot(job);
	panfrost_job_hw_submit(job, slot);

	return (0);
}
