/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Ruslan Bukin <br@bsdpad.com>
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

#include "opt_acpi.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/intr.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "smmu_reg.h"
#include "smmu_var.h"

#define	MEMORY_RESOURCE_SIZE	0x40000

struct smmu_acpi_devinfo {
	//struct smmu_devinfo	di_smmu_dinfo;
	struct resource_list	di_rl;
};

static device_identify_t smmu_acpi_identify;
static device_probe_t smmu_acpi_probe;
static device_attach_t smmu_acpi_attach;
static bus_alloc_resource_t smmu_acpi_bus_alloc_res;

static void smmu_acpi_bus_attach(device_t);

static device_method_t smmu_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,		smmu_acpi_identify),
	DEVMETHOD(device_probe,			smmu_acpi_probe),
	DEVMETHOD(device_attach,		smmu_acpi_attach),

	/* Bus interface */
	DEVMETHOD(bus_alloc_resource,		smmu_acpi_bus_alloc_res),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),

	/* End */
	DEVMETHOD_END
};

DEFINE_CLASS_1(smmu, smmu_acpi_driver, smmu_acpi_methods,
    sizeof(struct smmu_softc), smmu_driver);

static devclass_t smmu_acpi_devclass;

EARLY_DRIVER_MODULE(smmu, acpi, smmu_acpi_driver, smmu_acpi_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

#define	MAX_SMMU	8

struct iort_table_data {
	device_t parent;
	device_t dev;
	ACPI_IORT_SMMU_V3 *smmu[MAX_SMMU];
	int count;
};

static void
iort_handler(ACPI_SUBTABLE_HEADER *entry, void *arg)
{
	struct iort_table_data *iort_data;
	ACPI_IORT_NODE *node;
	int i;

	iort_data = (struct iort_table_data *)arg;
	i = iort_data->count;

	switch(entry->Type) {
	case ACPI_IORT_NODE_SMMU_V3:
		if (iort_data->smmu[i] != NULL) {
			if (bootverbose)
				device_printf(iort_data->parent,
				    "smmu: Already have an SMMU table");
			break;
		}
		node = (ACPI_IORT_NODE *)entry;
		iort_data->smmu[i] = (ACPI_IORT_SMMU_V3 *)node->NodeData;
		iort_data->count++;
		break;
	default:
		break;
	}
}

#if 0
static void
rdist_map(ACPI_SUBTABLE_HEADER *entry, void *arg)
{
	ACPI_MADT_GENERIC_REDISTRIBUTOR *redist;
	struct iort_table_data *iort_data;

	iort_data = (struct iort_table_data *)arg;

	switch(entry->Type) {
	case ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR:
		redist = (ACPI_MADT_GENERIC_REDISTRIBUTOR *)entry;

		iort_data->count++;
		BUS_SET_RESOURCE(iort_data->parent, iort_data->dev,
		    SYS_RES_MEMORY, iort_data->count, redist->BaseAddress,
		    redist->Length);
		break;

	default:
		break;
	}
}
#endif

static void
smmu_acpi_identify(driver_t *driver, device_t parent)
{
	struct iort_table_data iort_data;
	ACPI_TABLE_IORT *iort;
	vm_paddr_t physaddr;
	device_t dev;
	int i;

	printf("%s\n", __func__);

	physaddr = acpi_find_table(ACPI_SIG_IORT);
	if (physaddr == 0)
		return;

	iort = acpi_map_table(physaddr, ACPI_SIG_IORT);
	if (iort == NULL) {
		device_printf(parent, "smmu: Unable to map the IORT\n");
		return;
	}

	iort_data.parent = parent;
	for (i = 0; i < MAX_SMMU; i++)
		iort_data.smmu[i] = NULL;
	iort_data.count = 0;

	acpi_walk_subtables(iort + 1, (char *)iort + iort->Header.Length,
	    iort_handler, &iort_data);
	if (iort_data.count == 0) {
		device_printf(parent, "No SMMU found.\n");
		goto out;
	}
#if 0
	/* This is for the wrong GIC version */
	if (iort_data.smmu->Version != ACPI_MADT_GIC_VERSION_V3)
		goto out;
#endif

	for (i = 0; i < iort_data.count; i++) {
		dev = BUS_ADD_CHILD(parent,
		    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE, "smmu", -1);
		if (dev == NULL) {
			device_printf(parent, "add smmu child failed\n");
			goto out;
		}

		printf("intr ids %d %d %d, prio %d\n",
			iort_data.smmu[i]->EventGsiv,
			iort_data.smmu[i]->SyncGsiv,
			iort_data.smmu[i]->GerrGsiv,
			iort_data.smmu[i]->PriGsiv);

		/* Add the IORT data */
		BUS_SET_RESOURCE(parent, dev, SYS_RES_IRQ, 0,
		    iort_data.smmu[i]->EventGsiv, 1);
		BUS_SET_RESOURCE(parent, dev, SYS_RES_IRQ, 1,
		    iort_data.smmu[i]->SyncGsiv, 1);
		BUS_SET_RESOURCE(parent, dev, SYS_RES_IRQ, 2,
		    iort_data.smmu[i]->GerrGsiv, 1);
		BUS_SET_RESOURCE(parent, dev, SYS_RES_MEMORY, 0,
		    iort_data.smmu[i]->BaseAddress, MEMORY_RESOURCE_SIZE);

		acpi_set_private(dev,
		    (void *)(uintptr_t)iort_data.smmu[i]->Model);
	}

	iort_data.dev = dev;

#if 0
	acpi_walk_subtables(iort + 1, (char *)iort + iort->Header.Length,
	    rdist_map, &iort_data);
#endif

out:
	acpi_unmap_table(iort);
}

static int
smmu_acpi_probe(device_t dev)
{

	switch((uintptr_t)acpi_get_private(dev)) {
	case ACPI_IORT_SMMU_V3_GENERIC:
		/* Generic SMMUv3 */
		break;
	default:
		return (ENXIO);
	}

	device_set_desc(dev, SMMU_DEVSTR);

	return (BUS_PROBE_NOWILDCARD);
}

static void
iort_count_redistrib(ACPI_SUBTABLE_HEADER *entry, void *arg)
{
#if 0
	struct smmu_softc *sc = arg;

	if (entry->Type == ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR)
		sc->smmu_redists.nregions++;
#endif
}

static int
smmu_acpi_count_regions(device_t dev)
{
#if 0
	struct smmu_softc *sc;
	ACPI_TABLE_MADT *iort;
	vm_paddr_t physaddr;

	sc = device_get_softc(dev);

	physaddr = acpi_find_table(ACPI_SIG_MADT);
	if (physaddr == 0)
		return (ENXIO);

	iort = acpi_map_table(physaddr, ACPI_SIG_MADT);
	if (iort == NULL) {
		device_printf(dev, "Unable to map the MADT\n");
		return (ENXIO);
	}

	acpi_walk_subtables(iort + 1, (char *)iort + iort->Header.Length,
	    iort_count_redistrib, sc);
	acpi_unmap_table(iort);

	return (sc->smmu_redists.nregions > 0 ? 0 : ENXIO);
#endif
	return (0);
}

static int
smmu_acpi_attach(device_t dev)
{
	struct smmu_softc *sc;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;
	//sc->smmu_bus = GIC_BUS_ACPI;

	err = smmu_acpi_count_regions(dev);
	if (err != 0)
		goto error;

	err = smmu_attach(dev);
	if (err != 0)
		goto error;

#if 0
	sc->smmu_pic = intr_pic_register(dev, ACPI_INTR_XREF);
	if (sc->smmu_pic == NULL) {
		device_printf(dev, "could not register PIC\n");
		err = ENXIO;
		goto error;
	}

	if (intr_pic_claim_root(dev, ACPI_INTR_XREF, arm_smmu_intr, sc,
	    GIC_LAST_SGI - GIC_FIRST_SGI + 1) != 0) {
		err = ENXIO;
		goto error;
	}
#endif

	if (1 == 0)
		smmu_acpi_bus_attach(dev);

	//if (device_get_children(dev, &sc->smmu_children,
	//    &sc->smmu_nchildren) !=0)
	//	sc->smmu_nchildren = 0;

	return (0);

error:
	if (bootverbose) {
		device_printf(dev,
		    "Failed to attach. Error %d\n", err);
	}
	/* Failure so free resources */
	smmu_detach(dev);

	return (err);
}

static void
smmu_add_children(ACPI_SUBTABLE_HEADER *entry, void *arg)
{
#if 0
	ACPI_MADT_GENERIC_TRANSLATOR *smmut;
	struct smmu_acpi_devinfo *di;
	struct smmu_softc *sc;
	device_t child, dev;
	u_int xref;
	int err, pxm;

	if (entry->Type == ACPI_MADT_TYPE_GENERIC_TRANSLATOR) {
		/* We have an ITS, add it as a child */
		smmut = (ACPI_MADT_GENERIC_TRANSLATOR *)entry;
		dev = arg;
		sc = device_get_softc(dev);

		child = device_add_child(dev, "its", -1);
		if (child == NULL)
			return;

		di = malloc(sizeof(*di), M_GIC_V3, M_WAITOK | M_ZERO);
		resource_list_init(&di->di_rl);
		resource_list_add(&di->di_rl, SYS_RES_MEMORY, 0,
		    smmut->BaseAddress, smmut->BaseAddress + 256 * 1024 - 1,
		    256 * 1024);
		err = acpi_iort_its_lookup(smmut->TranslationId, &xref, &pxm);
		if (err == 0) {
			di->di_smmu_dinfo.smmu_domain = pxm;
			di->di_smmu_dinfo.msi_xref = xref;
		} else {
			di->di_smmu_dinfo.smmu_domain = -1;
			di->di_smmu_dinfo.msi_xref = ACPI_MSI_XREF;
		}
		sc->smmu_nchildren++;
		device_set_ivars(child, di);
	}
#endif
}

static void
smmu_acpi_bus_attach(device_t dev)
{
	ACPI_TABLE_MADT *iort;
	vm_paddr_t physaddr;

	physaddr = acpi_find_table(ACPI_SIG_MADT);
	if (physaddr == 0)
		return;

	iort = acpi_map_table(physaddr, ACPI_SIG_MADT);
	if (iort == NULL) {
		device_printf(dev, "Unable to map the MADT to add children\n");
		return;
	}

	acpi_walk_subtables(iort + 1, (char *)iort + iort->Header.Length,
	    smmu_add_children, dev);

	acpi_unmap_table(iort);

	bus_generic_attach(dev);
}

static struct resource *
smmu_acpi_bus_alloc_res(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct smmu_acpi_devinfo *di;
	struct resource_list_entry *rle;

	/* We only allocate memory */
	if (type != SYS_RES_MEMORY)
		return (NULL);

	if (RMAN_IS_DEFAULT_RANGE(start, end)) {
		if ((di = device_get_ivars(child)) == NULL)
			return (NULL);

		/* Find defaults for this rid */
		rle = resource_list_find(&di->di_rl, type, *rid);
		if (rle == NULL)
			return (NULL);

		start = rle->start;
		end = rle->end;
		count = rle->count;
	}

	return (bus_generic_alloc_resource(bus, child, type, rid, start, end,
	    count, flags));
}
