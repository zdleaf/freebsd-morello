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
#include <drm/gpu_scheduler.h>

#include <compat/linuxkpi/common/include/linux/jiffies.h>

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
#define	JOB_TIMEOUT_MS	500

static void panfrost_job_enable_interrupts(struct panfrost_softc *sc);

struct panfrost_queue_state {
	struct drm_gpu_scheduler sched;
	atomic_t status;
	struct mutex lock;
	uint64_t fence_context;
	uint64_t emit_seqno;
};

struct panfrost_job_slot {
	struct panfrost_queue_state queue[NUM_JOB_SLOTS];
	spinlock_t job_lock;
};

struct panfrost_fence {
	struct dma_fence base;
	struct drm_device *dev;
	int queue;
	uint64_t seqno;
};

void
panfrost_job_intr(void *arg)
{
	struct panfrost_softc *sc;
	uint32_t stat;
	uint32_t status;
	int mask;
	int i;
	struct panfrost_job *job;

	sc = arg;

	stat = GPU_READ(sc, JOB_INT_STAT);
	dprintf("%s: stat %x\n", __func__, stat);

	for (i = 0; stat; i++) {
		mask = (1 << i) | (1 << (16 + i));
		if ((stat & mask) == 0)
			continue;

		GPU_WRITE(sc, JOB_INT_CLEAR, mask);

		if (stat & (1 << (16 + i))) {
			GPU_WRITE(sc, JS_COMMAND_NEXT(i), JS_COMMAND_NOP);
			status = GPU_READ(sc, JS_STATUS(i));
			dprintf("%s: job error, slot %d status %x\n",
			    __func__, i, status);
			dprintf("%s: head %x tail %x\n", __func__,
			    GPU_READ(sc, JS_HEAD_LO(i)),
			    GPU_READ(sc, JS_TAIL_LO(i)));

			//drm_sched_fault(&sc->js->queue[i].sched);

			printf("%s: job at slot %d completed with error\n",
			    __func__, i);

			panic("error");

			mtx_lock(&sc->job_lock);
			sc->slot_status[i].running = 0;
			sc->running = 0;
			mtx_unlock(&sc->job_lock);

			job = sc->jobs[i];
			dma_fence_signal_locked(job->done_fence);
		}

		if (stat & (1 << i)) {

			//printf(".");
			dprintf("%s: job at slot %d completed\n", __func__, i);
			mtx_lock(&sc->job_lock);
			sc->slot_status[i].running = 0;
			sc->running = 0;
			mtx_unlock(&sc->job_lock);

			job = sc->jobs[i];
			panfrost_mmu_as_put(sc, &job->pfile->mmu);
			dma_fence_signal_locked(job->done_fence);
		}

		stat &= ~mask;
	}

	dprintf("%s: done\n", __func__);
}

int
panfrost_job_open(struct panfrost_file *pfile)
{
	struct panfrost_job_slot *js;
	struct panfrost_softc *sc;
	struct drm_gpu_scheduler *sched;
	int error;
	int i;

	sc = pfile->sc;

	js = sc->js;

	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		sched = &js->queue[i].sched;
		error = drm_sched_entity_init(&pfile->sched_entity[i],
		    DRM_SCHED_PRIORITY_NORMAL, &sched, 1, NULL);
		if (error)
			panic("error");
	}

	return (0);
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
	int status;

	sc = job->sc;
	jc_head = job->jc;

	dprintf("%s: HW submitting job %d to slot %d, jc %lx\n",
	    __func__, sc->job_count++, slot, jc_head);

	status = GPU_READ(sc, JS_COMMAND_NEXT(slot));
	if (status)
		panic("fault");

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

	wmb();
	GPU_WRITE(sc, JS_COMMAND_NEXT(slot), JS_COMMAND_START);
	wmb();

	dprintf("%s: job %d submitted to HW\n", __func__, sc->job_count - 1);
}

static void
panfrost_attach_object_fences(struct drm_gem_object **bos,
    int bo_count, struct dma_fence *fence)
{
	int i;

	for (i = 0; i < bo_count; i++)
		reservation_object_add_excl_fence(bos[i]->resv, fence);
}

int flag = 0;

int
panfrost_job_push(struct panfrost_job *job)
{
	struct ww_acquire_ctx acquire_ctx;
	struct panfrost_softc *sc;
	struct drm_sched_entity *entity;
	int slot;
	int error;

	sc = job->sc;

	mtx_lock(&sc->sched_lock);

	slot = panfrost_job_get_slot(job);
	entity = &job->pfile->sched_entity[slot];

	job->slot = slot;

	error = drm_gem_lock_reservations(job->bos, job->bo_count,
	    &acquire_ctx);
	if (error) {
		mtx_unlock(&sc->sched_lock);
		panic("could not lock reserv");
	}

	error = drm_sched_job_init(&job->base, entity, NULL);
	if (error)
		panic("coult not init job");

	refcount_acquire(&job->refcount);

	/* Acquire a reference to fence. */
	job->render_done_fence = dma_fence_get(&job->base.s_fence->finished);

	panfrost_acquire_object_fences(job->bos, job->bo_count,
	    job->implicit_fences);

	drm_sched_entity_push_job(&job->base, entity);

	mtx_unlock(&sc->sched_lock);

	panfrost_attach_object_fences(job->bos, job->bo_count,
	    job->render_done_fence);

	drm_gem_unlock_reservations(job->bos, job->bo_count, &acquire_ctx);

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

static struct dma_fence *
panfrost_job_dependency(struct drm_sched_job *sched_job,
    struct drm_sched_entity *s_entity)
{
	struct panfrost_job *job;
	struct dma_fence *fence;
	int i;

	job = (struct panfrost_job *)sched_job;

	dprintf("%s\n", __func__);

	for (i = 0; i < job->in_fence_count; i++) {
		if (job->in_fences[i]) {
			fence = job->in_fences[i];
			job->in_fences[i] = NULL;
			return (fence);
		}
	}

	for (i = 0; i < job->bo_count; i++) {
		if (job->implicit_fences[i]) {
			fence = job->implicit_fences[i];
			job->implicit_fences[i] = NULL;
			return (fence);
		}
	}

	dprintf("%s: no more\n", __func__);

	return (NULL);
}

static const char *
panfrost_fence_get_driver_name(struct dma_fence *fence)
{

	return "panfrost";
}

static const char *
panfrost_fence_get_timeline_name(struct dma_fence *fence)
{
	struct panfrost_fence *f;

	f = (struct panfrost_fence *)fence;

	switch (f->queue) {
	case 0: return "panfrost-js-0";
	case 1: return "panfrost-js-1";
	case 2: return "panfrost-js-2";
	default:
		return NULL;
	}
}

static bool
dma_fence_chain_enable_signaling(struct dma_fence *fence)
{

	dprintf("%s\n", __func__);

	return (true);
}

static void
panfrost_fence_release(struct dma_fence *fence)
{

	printf("%s\n", __func__);
}

static const struct dma_fence_ops panfrost_fence_ops = {
	.get_driver_name = panfrost_fence_get_driver_name,
	.get_timeline_name = panfrost_fence_get_timeline_name,
	.enable_signaling = dma_fence_chain_enable_signaling,
	.release = panfrost_fence_release,
};

static struct dma_fence *
panfrost_fence_create(struct panfrost_softc *sc, int js_num)
{
	struct panfrost_job_slot *js;
	struct panfrost_fence *fence;

	js = sc->js;

	fence = malloc(sizeof(*fence), M_PANFROST, M_ZERO | M_WAITOK);
	if (!fence)
		return (NULL);

	fence->dev = &sc->drm_dev;
	fence->queue = js_num;
	fence->seqno = ++js->queue[js_num].emit_seqno;
	dma_fence_init(&fence->base, &panfrost_fence_ops, &js->job_lock,
	    js->queue[js_num].fence_context, fence->seqno);

        return (&fence->base);
}

static struct dma_fence *
panfrost_job_run(struct drm_sched_job *sched_job)
{
	struct panfrost_softc *sc;
	struct panfrost_job *job;
	struct dma_fence *fence;

	dprintf("%s\n", __func__);

	job = (struct panfrost_job *)sched_job;
	sc = job->sc;

	fence = panfrost_fence_create(sc, job->slot);
	if (!fence)
                return (NULL);

	if (job->done_fence)
		dma_fence_put(job->done_fence);
	job->done_fence = dma_fence_get(fence);

	//if (unlikely(job->base.s_fence->finished.error))
	//	return (NULL);

	sc->jobs[job->slot] = job;

	panfrost_job_hw_submit(job, job->slot);

	return (fence);
}

static void
panfrost_job_timedout(struct drm_sched_job *sched_job)
{
	struct panfrost_softc *sc;
	struct panfrost_job *job;
	uint32_t stat;

	job = (struct panfrost_job *)sched_job;
	sc = job->sc;

	stat = GPU_READ(sc, JOB_INT_STAT);
	printf("%s: stat %x\n", __func__, stat);
	panic("timedout");
	//if (dma_fence_is_signaled(job->done_fence))
	//	return;
	//printf("%s: not signalled\n", __func__);
}

static void
panfrost_job_cleanup(struct panfrost_job *job)
{
	struct panfrost_gem_object *obj;
	struct drm_gem_object *bo;
	struct drm_device *dev;
	int i;

	dprintf("%s\n", __func__);

	if (job->in_fences) {
		for (i = 0; i < job->in_fence_count; i++)
			dma_fence_put(job->in_fences[i]);
		free(job->in_fences, M_PANFROST);
	}

	if (job->implicit_fences) {
		for (i = 0; i < job->bo_count; i++)
			dma_fence_put(job->implicit_fences[i]);
		free(job->implicit_fences, M_PANFROST);
	}

	dma_fence_put(job->done_fence);
	dma_fence_put(job->render_done_fence);

	if (job->mappings) {
		for (i = 0; i < job->bo_count; i++) {
			if (!job->mappings[i])
				break;

			obj = job->mappings[i]->obj;
			atomic_add_int(&obj->gpu_usecount, -1);

			panfrost_gem_mapping_put(job->mappings[i]);
		}
		free(job->mappings, M_PANFROST);
	}

	if (job->bos) {
		for (i = 0; i < job->bo_count; i++) {
			bo = job->bos[i];
			dev = bo->dev;
			mutex_lock(&dev->struct_mutex);
			drm_gem_object_put(bo);
			mutex_unlock(&dev->struct_mutex);
		}

		free(job->bos, M_PANFROST);
	}

	free(job, M_PANFROST);

	dprintf("%s done\n", __func__);
}

void
panfrost_job_put(struct panfrost_job *job)
{

	if (refcount_release(&job->refcount))
		panfrost_job_cleanup(job);
}

static void
panfrost_job_free(struct drm_sched_job *sched_job)
{
	struct panfrost_job *job;

	dprintf("%s\n", __func__);

	drm_sched_job_cleanup(sched_job);

	job = (struct panfrost_job *)sched_job;
	panfrost_job_put(job);
}

static const struct drm_sched_backend_ops panfrost_sched_ops = {
	.dependency = panfrost_job_dependency,
	.run_job = panfrost_job_run,
	.timedout_job = panfrost_job_timedout,
	.free_job = panfrost_job_free
};

int
panfrost_job_init(struct panfrost_softc *sc)
{
	struct panfrost_job_slot *js;
	int error, i;

	sc->js = js = malloc(sizeof(*js), M_PANFROST, M_ZERO | M_WAITOK);

	spin_lock_init(&js->job_lock);

	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		js->queue[i].fence_context = dma_fence_context_alloc(1);
		error = drm_sched_init(&js->queue[i].sched,
		    &panfrost_sched_ops, 1, 0,
		    msecs_to_jiffies(JOB_TIMEOUT_MS), "pan_js");
		printf("drm_sched_init error %d\n", error);
	}

	panfrost_job_enable_interrupts(sc);

	return (0);
}
