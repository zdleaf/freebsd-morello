/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Andrew Turner
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpi_pcibvar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>
#include <dev/pci/pci_host_generic.h>
#include <dev/pci/pci_host_generic_acpi.h>

#include "pcib_if.h"

/* Assembling ECAM Configuration Address */
#define	PCIE_BUS_SHIFT		20
#define	PCIE_SLOT_SHIFT		15
#define	PCIE_FUNC_SHIFT		12
#define	PCIE_BUS_MASK		0xFF
#define	PCIE_SLOT_MASK		0x1F
#define	PCIE_FUNC_MASK		0x07
#define	PCIE_REG_MASK		0xFFF

#define	PCIE_ADDR_OFFSET(bus, slot, func, reg)			\
	((((bus) & PCIE_BUS_MASK) << PCIE_BUS_SHIFT)	|	\
	(((slot) & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT)	|	\
	(((func) & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT)	|	\
	((reg) & PCIE_REG_MASK))

#define	PCIE_BDF(bus, slot, func)				\
	((((bus) & PCIE_BUS_MASK) << PCIE_BUS_SHIFT)	|	\
	(((slot) & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT)	|	\
	(((func) & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT))

#define	AP_NS_SHARED_MEM_BASE	0x06000000
#define	MAX_SEGMENTS		2 /* Two PCIe root complex devices. */
#define	BDF_TABLE_SIZE		(16 * 1024)
#define	PCI_CFG_SPACE		0x1000

extern struct bus_space memmap_bus;
bus_space_handle_t rc_remapped_addr[MAX_SEGMENTS];

struct pcie_discovery_data {
	uint32_t rc_base_addr;
	uint32_t nr_bdfs;
	uint32_t valid_bdfs[0];
} *pcie_discovery_data[MAX_SEGMENTS];

static void
n1sdp_init(struct generic_pcie_core_softc *sc)
{
	bus_addr_t paddr;
	bus_addr_t paddr_rc;
	bus_size_t psize;
	bus_size_t psize_rc;
	bus_space_handle_t vaddr;
	bus_space_handle_t vaddr_rc;
	int err;
	int bdfs_size;
	struct pcie_discovery_data *shared_data;

	paddr = AP_NS_SHARED_MEM_BASE + sc->segment * BDF_TABLE_SIZE;
	psize = BDF_TABLE_SIZE;
	err = bus_space_map(&memmap_bus, paddr, psize, 0, &vaddr);

	shared_data = (struct pcie_discovery_data *)vaddr;
	bdfs_size = sizeof(struct pcie_discovery_data) +
		sizeof(uint32_t) * shared_data->nr_bdfs;
	pcie_discovery_data[sc->segment] =
	    malloc(bdfs_size, M_DEVBUF, M_WAITOK | M_ZERO);
	memcpy(pcie_discovery_data[sc->segment], shared_data, bdfs_size);

	paddr_rc = shared_data->rc_base_addr;
	psize_rc = PCI_CFG_SPACE;
	err = bus_space_map(&memmap_bus, paddr_rc, psize_rc, 0, &vaddr_rc);

	rc_remapped_addr[sc->segment] = vaddr_rc;

	int table_count;
	int i;

	if (bootverbose) {
		table_count = pcie_discovery_data[sc->segment]->nr_bdfs;
		for (i = 0; i < table_count; i++)
			device_printf(sc->dev, "valid bdf %x\n",
			    pcie_discovery_data[sc->segment]->valid_bdfs[i]);
	}

	bus_space_unmap(&memmap_bus, vaddr, psize);
}

static int
n1sdp_check_bdf(struct generic_pcie_core_softc *sc,
    u_int bus, u_int slot, u_int func)
{
	int table_count;
	int bdf;
	int i;

	bdf = PCIE_BDF(bus, slot, func);
	if (bdf == 0)
		return (1);

	table_count = pcie_discovery_data[sc->segment]->nr_bdfs;

	for (i = 0; i < table_count; i++)
		if (bdf == pcie_discovery_data[sc->segment]->valid_bdfs[i])
			return (1);

	return (0);
}

static int
n1sdp_pcie_acpi_probe(device_t dev)
{
	ACPI_DEVICE_INFO *devinfo;
	ACPI_TABLE_HEADER *hdr;
	ACPI_STATUS status;
	ACPI_HANDLE h;
	int root;

	if (acpi_disabled("pcib") || (h = acpi_get_handle(dev)) == NULL ||
	    ACPI_FAILURE(AcpiGetObjectInfo(h, &devinfo)))
		return (ENXIO);
	root = (devinfo->Flags & ACPI_PCI_ROOT_BRIDGE) != 0;
	AcpiOsFree(devinfo);
	if (!root)
		return (ENXIO);

	/* TODO: Move this to an ACPI quirk? */
	status = AcpiGetTable(ACPI_SIG_MCFG, 1, &hdr);
	if (ACPI_FAILURE(status))
		return (ENXIO);

	if (memcmp(hdr->OemId, "ARMLTD", ACPI_OEM_ID_SIZE) != 0 ||
	    memcmp(hdr->OemTableId, "ARMN1SDP", ACPI_OEM_TABLE_ID_SIZE) != 0 ||
	    hdr->OemRevision != 0x20181101)
		return (ENXIO);

	device_set_desc(dev, "ARM N1SDP PCI host controller");
	return (BUS_PROBE_DEFAULT);
}

static int
n1sdp_pcie_acpi_attach(device_t dev)
{
	struct generic_pcie_core_softc *sc;
	ACPI_HANDLE handle;
	ACPI_STATUS status;
	int err;

	sc = device_get_softc(dev);

	handle = acpi_get_handle(dev);

	/* Get PCI Segment (domain) needed for IOMMU space remap. */
	status = acpi_GetInteger(handle, "_SEG", &sc->segment);
	if (ACPI_FAILURE(status)) {
		device_printf(dev, "No _SEG for PCI Bus\n");
		return (ENXIO);
	}

	if (sc->segment == MAX_SEGMENTS) {
		device_printf(dev, "Unknown PCI Bus segment (domain)\n");
		return (ENXIO);
	}

	n1sdp_init(sc);

	err = pci_host_generic_acpi_attach(dev);

	return (err);
}

static uint32_t
n1sdp_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	struct generic_pcie_core_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t	t;
	uint64_t offset;
	uint32_t data;

	sc = device_get_softc(dev);
	if ((bus < sc->bus_start) || (bus > sc->bus_end))
		return (~0U);
	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) ||
	    (reg > PCIE_REGMAX))
		return (~0U);

	if (n1sdp_check_bdf(sc, bus, slot, func) == 0)
		return (~0U);

	offset = PCIE_ADDR_OFFSET(bus - sc->bus_start, slot, func, reg);
	t = sc->bst;
	h = sc->bsh;

	if (bus == 0 && slot == 0 && func == 0) {
		t = &memmap_bus;
		h = rc_remapped_addr[sc->segment];
	}

	data = bus_space_read_4(t, h, offset & ~3);

	switch (bytes) {
	case 1:
		data >>= (offset & 3) * 8;
		data &= 0xff;
		break;
	case 2:
		data >>= (offset & 3) * 8;
		data = le16toh(data);
		data &= 0xffff;
		break;
	case 4:
		data = le32toh(data);
		break;
	default:
		return (~0U);
	}

	return (data);
}

static void
n1sdp_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct generic_pcie_core_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t t;
	uint64_t offset;
	uint32_t data;

	sc = device_get_softc(dev);
	if ((bus < sc->bus_start) || (bus > sc->bus_end))
		return;
	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) ||
	    (reg > PCIE_REGMAX))
		return;

	if (n1sdp_check_bdf(sc, bus, slot, func) == 0)
		return;

	offset = PCIE_ADDR_OFFSET(bus - sc->bus_start, slot, func, reg);

	t = sc->bst;
	h = sc->bsh;

	if (bus == 0 && slot == 0 && func == 0) {
		t = &memmap_bus;
		h = rc_remapped_addr[sc->segment];
	}

	/*
	 * TODO: This is probably wrong on big-endian, however as arm64 is
	 * little endian it should be fine.
	 */
	switch (bytes) {
	case 1:
		data = bus_space_read_4(t, h, offset & ~3);
		data &= ~(0xff << ((offset & 3) * 8));
		data |= (val & 0xff) << ((offset & 3) * 8);
		bus_space_write_4(t, h, offset & ~3, htole32(data));
		break;
	case 2:
		data = bus_space_read_4(t, h, offset & ~3);
		data &= ~(0xffff << ((offset & 3) * 8));
		data |= (val & 0xffff) << ((offset & 3) * 8);
		bus_space_write_4(t, h, offset & ~3, htole32(data));
		break;
	case 4:
		bus_space_write_4(t, h, offset, htole32(val));
		break;
	default:
		return;
	}
}

static device_method_t n1sdp_pcie_acpi_methods[] = {
	DEVMETHOD(device_probe,		n1sdp_pcie_acpi_probe),
	DEVMETHOD(device_attach,	n1sdp_pcie_acpi_attach),

	/* pcib interface */
	DEVMETHOD(pcib_read_config,	n1sdp_pcie_read_config),
	DEVMETHOD(pcib_write_config,	n1sdp_pcie_write_config),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, n1sdp_pcie_acpi_driver, n1sdp_pcie_acpi_methods,
    sizeof(struct generic_pcie_acpi_softc), generic_pcie_acpi_driver);

static devclass_t n1sdp_pcie_acpi_devclass;

DRIVER_MODULE(n1sdp_pcib, acpi, n1sdp_pcie_acpi_driver,
    n1sdp_pcie_acpi_devclass, 0, 0);
