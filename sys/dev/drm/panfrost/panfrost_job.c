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

#define	NUM_JOB_SLOTS	3

static void panfrost_job_wakeup(struct panfrost_softc *sc, int slot);

void
panfrost_job_intr(void *arg)
{
	struct panfrost_softc *sc;
	uint32_t stat;
	uint32_t status;
	int mask;
	int i;

	sc = arg;

	printf("%s\n", __func__);

	stat = GPU_READ(sc, JOB_INT_STAT);

	for (i = 0; stat; i++) {
		mask = (1 << i) | (1 << (16 + i));
		if ((stat & mask) == 0)
			continue;

		GPU_WRITE(sc, JOB_INT_CLEAR, mask);

		if (stat & (1 << (16 + i))) {
			GPU_WRITE(sc, JS_COMMAND_NEXT(i), JS_COMMAND_NOP);
			status = GPU_READ(sc, JS_STATUS(i));
			printf("%s: job error, slot %d status %x\n",
			    __func__, i, status);
			printf("%s: head %x tail %x\n", __func__,
			    GPU_READ(sc, JS_HEAD_LO(i)),
			    GPU_READ(sc, JS_TAIL_LO(i)));
			//panic("job error");



			printf("%s: job at slot %d completed wih error\n",
			    __func__, i);
			mtx_lock(&sc->job_lock);
			sc->slot_status[i].running = 0;
			//sc->running = 0;
			mtx_unlock(&sc->job_lock);

			panfrost_job_wakeup(sc, i);
		}

		if (stat & (1 << i)) {

			printf("%s: job at slot %d completed\n", __func__, i);
			mtx_lock(&sc->job_lock);
			sc->slot_status[i].running = 0;
			//sc->running = 0;
			mtx_unlock(&sc->job_lock);

			panfrost_job_wakeup(sc, i);
		}

		stat &= ~mask;
	}
}

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
panfrost_job_write_affinity(struct panfrost_softc *sc, uint32_t requirements,
    int js)
{
	uint64_t affinity;

	affinity = sc->features.shader_present;

	GPU_WRITE(sc, JS_AFFINITY_NEXT_LO(js), affinity & 0xFFFFFFFF);
	GPU_WRITE(sc, JS_AFFINITY_NEXT_HI(js), affinity >> 32);
}

static void
panfrost_job_hw_submit(struct panfrost_job *job, int slot)
{
	struct panfrost_softc *sc;
	uint32_t cfg;
	uint64_t jc_head;

	sc = job->sc;
	jc_head = job->jc;

	printf("%s: HW submitting new job to slot %d, jc %lx\n",
	    __func__, slot, jc_head);

	cfg = panfrost_mmu_as_get(sc, &job->pfile->mmu);

	GPU_WRITE(sc, JS_HEAD_NEXT_LO(slot), jc_head & 0xFFFFFFFF);
	GPU_WRITE(sc, JS_HEAD_NEXT_HI(slot), jc_head >> 32);

	panfrost_job_write_affinity(sc, job->requirements, slot);

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

static void
panfrost_job_wakeup(struct panfrost_softc *sc, int slot)
{
	struct panfrost_job *job, *job1;

	mtx_lock(&sc->job_lock);

	if (sc->slot_status[slot].running)
		goto out;

	TAILQ_FOREACH_SAFE(job, &sc->job_queue, next, job1) {
		if (job->slot == slot) {
			TAILQ_REMOVE(&sc->job_queue, job, next);
			sc->slot_status[job->slot].running = 1;
			//sc->running = 1;
			panfrost_job_hw_submit(job, job->slot);
			break;
		}
	}

out:
	mtx_unlock(&sc->job_lock);
}

int
panfrost_job_push(struct panfrost_job *job)
{
	struct ww_acquire_ctx acquire_ctx;
	struct panfrost_softc *sc;
	int slot;
	int error;

	sc = job->sc;

	slot = panfrost_job_get_slot(job);
	job->slot = slot;

printf("%s: new job for slot %d\n", __func__, slot);

	error = drm_gem_lock_reservations(job->bos, job->bo_count,
	    &acquire_ctx);
	if (error)
		return (error);

	/* Acquire a reference to fence. */
	//job->render_done_fence = dma_fence_get(&job->base.s_fence->finished);

	panfrost_acquire_object_fences(job->bos, job->bo_count,
	    job->implicit_fences);

	drm_gem_unlock_reservations(job->bos, job->bo_count, &acquire_ctx);

	mtx_lock(&sc->job_lock);
	TAILQ_INSERT_TAIL(&sc->job_queue, job, next);
	mtx_unlock(&sc->job_lock);

	panfrost_job_wakeup(sc, slot);

	return (0);
}

static void
panfrost_job_enable_interrupts(struct panfrost_softc *sc)
{
	uint32_t irq_msk;
	int i;

	irq_msk = 0;

	for (i = 0; i < NUM_JOB_SLOTS; i++)
		irq_msk |= MK_JS_MASK(i);

	GPU_WRITE(sc, JOB_INT_CLEAR, irq_msk);
	GPU_WRITE(sc, JOB_INT_MASK, irq_msk);
}

int
panfrost_job_init(struct panfrost_softc *sc)
{

	panfrost_job_enable_interrupts(sc);

	return (0);
}
