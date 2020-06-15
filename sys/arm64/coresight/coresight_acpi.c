/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
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
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <arm64/coresight/coresight.h>

MALLOC_DEFINE(M_CORESIGHT_ACPI, "coresight_acpi", "ARM Coresight ACPI");
static struct mtx cs_mtx;

struct coresight_device_list cs_devs_acpi;

int
coresight_acpi_register(struct coresight_desc *desc)
{
	struct coresight_device *cs_dev;

	cs_dev = malloc(sizeof(struct coresight_device),
	    M_CORESIGHT_ACPI, M_WAITOK | M_ZERO);
	cs_dev->dev = desc->dev;
	//cs_dev->node = ofw_bus_get_node(desc->dev);
	cs_dev->pdata = desc->pdata;
	cs_dev->dev_type = desc->dev_type;

	mtx_lock(&cs_mtx);
	TAILQ_INSERT_TAIL(&cs_devs_acpi, cs_dev, link);
	mtx_unlock(&cs_mtx);

	return (0);
}

static int
coresight_get_cpu(device_t dev, struct coresight_platform_data *pdata)
{
	ACPI_HANDLE handle, parent;
	ACPI_STATUS status;
	int cpuid;

	handle = acpi_get_handle(dev);

	status = AcpiGetParent(handle, &parent);
	if (!ACPI_SUCCESS(status))
		return (ENXIO);

	if (!acpi_MatchHid(parent, "ACPI0007"))
		return (ENXIO);

	status = acpi_GetInteger(parent, "_UID", &cpuid);
	if (ACPI_SUCCESS(status)) {
		pdata->cpu = cpuid;
		printf("cpuid %d\n", cpuid);
		return (0);
	}

	return (ENXIO);
}

static int
coresight_get_ports(device_t dev,
    struct coresight_platform_data *pdata)
{

	/* TODO */

	return (0);
}

struct coresight_platform_data *
coresight_acpi_get_platform_data(device_t dev)
{
	struct coresight_platform_data *pdata;

	pdata = malloc(sizeof(struct coresight_platform_data),
	    M_CORESIGHT_ACPI, M_WAITOK | M_ZERO);
	mtx_init(&pdata->mtx_lock, "Coresight Platform Data", NULL, MTX_DEF);
	TAILQ_INIT(&pdata->endpoints);

	coresight_get_cpu(dev, pdata);
	coresight_get_ports(dev, pdata);

	if (bootverbose)
		printf("Total ports: in %d out %d\n",
		    pdata->in_ports, pdata->out_ports);

	return (pdata);
}

static void
coresight_acpi_init(void)
{

	mtx_init(&cs_mtx, "ARM Coresight ACPI", NULL, MTX_DEF);
	TAILQ_INIT(&cs_devs_acpi);
}

SYSINIT(coresight_acpi, SI_SUB_DRIVERS, SI_ORDER_FIRST,
    coresight_acpi_init, NULL);
