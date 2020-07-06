/*-
 * Copyright (c) 2018-2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory under DARPA/AFRL contract FA8750-10-C-0237
 * ("CTSRD"), as part of the DARPA CRASH research programme.
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <arm64/coresight/coresight.h>

#include "coresight_if.h"

extern struct coresight_device_list cs_devs;

static struct coresight_device *
coresight_next_device(struct coresight_device *cs_dev,
    struct coresight_event *event)
{
	struct coresight_device *out;
	struct endpoint *out_endp;
	struct endpoint *endp;
	struct endpoint *endp1;

	TAILQ_FOREACH(endp, &cs_dev->pdata->endpoints, link) {
		if (endp->input != 0)
			continue;

		out = coresight_get_output_device(endp, &out_endp);
		if (out != NULL) {
			if (TAILQ_EMPTY(&event->endplist)) {
				if (bootverbose)
					printf("Adding source device %s\n",
					    device_get_nameunit(out->dev));
				/* Add source device */
				endp1 = malloc(sizeof(struct endpoint),
				    M_CORESIGHT, M_WAITOK);
				memcpy(endp1, endp, sizeof(struct endpoint));
				endp1->cs_dev = cs_dev;
				TAILQ_INSERT_HEAD(&event->endplist, endp1,
				    endplink);
			}

			/* Add output device */
			if (bootverbose)
				printf("Adding device %s to the chain\n",
				    device_get_nameunit(out->dev));
			endp1 = malloc(sizeof(struct endpoint),
			    M_CORESIGHT, M_WAITOK);
			memcpy(endp1, out_endp, sizeof(struct endpoint));
			endp1->cs_dev = out;
			TAILQ_INSERT_HEAD(&event->endplist, endp1, endplink);

			return (out);
		}
	}

	return (NULL);
}

static int
coresight_build_list(struct coresight_device *cs_dev,
    struct coresight_event *event)
{
	struct coresight_device *out;

	out = cs_dev;
	while (out != NULL)
		out = coresight_next_device(out, event);

	return (0);
}

int
coresight_init_event(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	/* Start building path from source device */
	TAILQ_FOREACH(cs_dev, &cs_devs, link) {
		if (cs_dev->dev_type == event->src &&
		    cs_dev->pdata->cpu == cpu) {
			TAILQ_INIT(&event->endplist);
			coresight_build_list(cs_dev, event);
			break;
		}
	}

	/* Ensure Coresight is initialized for the CPU */
	TAILQ_FOREACH(cs_dev, &cs_devs, link) {
		if (cs_dev->dev_type == CORESIGHT_CPU_DEBUG &&
		    cs_dev->pdata->cpu == cpu)
			CORESIGHT_INIT(cs_dev->dev);
	}

	/* Init all devices in the path */
	TAILQ_FOREACH(endp, &event->endplist, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_INIT(cs_dev->dev);
	}

	return (0);
}

void
coresight_deinit_event(int cpu, struct coresight_event *event)
{
	struct endpoint *endp;
	struct endpoint *endp1;

	TAILQ_FOREACH_SAFE(endp, &event->endplist, endplink, endp1) {
		free(endp, M_CORESIGHT);
	}
}

void
coresight_enable(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	TAILQ_FOREACH(endp, &event->endplist, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_ENABLE(cs_dev->dev, endp, event);
	}
}

void
coresight_disable(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	TAILQ_FOREACH_REVERSE(endp, &event->endplist, endpoint_list, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_DISABLE(cs_dev->dev, endp, event);
	}
}

void
coresight_read(int cpu, struct coresight_event *event)
{
	struct endpoint *endp;

	TAILQ_FOREACH(endp, &event->endplist, endplink)
		CORESIGHT_READ(endp->cs_dev->dev, endp, event);
}

void
coresight_allocate(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	TAILQ_FOREACH(endp, &event->endplist, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_ALLOCATE(cs_dev->dev, endp, event);
	}
}

void
coresight_release(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	TAILQ_FOREACH(endp, &event->endplist, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_RELEASE(cs_dev->dev, endp, event);
	}

	coresight_deinit_event(cpu, event);
}

void
coresight_info(int cpu, struct coresight_event *event)
{
	struct coresight_device *cs_dev;
	struct endpoint *endp;

	TAILQ_FOREACH(endp, &event->endplist, endplink) {
		cs_dev = endp->cs_dev;
		CORESIGHT_INFO(cs_dev->dev, endp, event);
	}
}
