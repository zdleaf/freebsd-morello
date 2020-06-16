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
#include <sys/uuid.h>
#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <arm64/coresight/coresight.h>

MALLOC_DEFINE(M_CORESIGHT_ACPI, "coresight_acpi", "ARM Coresight ACPI");
static struct mtx cs_mtx;

#define	ACPI_CORESIGHT_LINK_MASTER	1
#define	ACPI_CORESIGHT_LINK_SLAVE	0

struct coresight_device_list cs_devs_acpi;

struct coresight_connection {
	int outport;
	int child_port;
	//struct fwnode_handle *child_fwnode;
	//struct coresight_device *child_dev;
};

#if 0
struct uuid {
        uint32_t        time_low;
        uint16_t        time_mid;
        uint16_t        time_hi_and_version;
        uint8_t         clock_seq_hi_and_reserved;
        uint8_t         clock_seq_low;
        uint8_t         node[_UUID_NODE_LEN];
};

static const struct uuid acpi_graph_uuid = {
	.time_low = 0xab02a46b,
	.time_mid = 0x74c7,
	.time_hi_and_version = 0x45a2,
	.clock_seq_hi_and_reserved = 0xbd,
	.clock_seq_low = 0x68,
	.node = { 0xf7, 0xd3, 0x44, 0xef, 0x21, 0x53 },
};
#endif

static const struct uuid acpi_graph_uuid = {
	0xab02a46b, 0x74c7, 0x45a2, 0xbd, 0x68,
	{ 0xf7, 0xd3, 0x44, 0xef, 0x21, 0x53 },
};

static const struct uuid coresight_graph_uuid = {
	0x3ecbc8b6, 0x1d0e, 0x4fb3, 0x81, 0x07,
	{ 0xe6, 0x27, 0xf8, 0x05, 0xc6, 0xcd },
};

static inline bool
acpi_validate_dsd_graph(const union acpi_object *graph)
{
	const union acpi_object *rev, *nr_graphs;
	const union acpi_object *obj;
	int i, n;

	if (graph->Package.Count < 2)
		return (false);

	rev = &graph->Package.Elements[0];
	nr_graphs = &graph->Package.Elements[1];

	if (rev->Type != ACPI_TYPE_INTEGER ||
	    nr_graphs->Type != ACPI_TYPE_INTEGER)
		return (false);

	/* Revision 0 supported only. */
	if (rev->Integer.Value != 0)
		return (false);

	/* We are looking for a single graph. */
	n = nr_graphs->Integer.Value;
	if (n != 1)
		return (false);

	/* Check number of elements. */
	if (graph->Package.Count != (n + 2))
		return (false);

	for (i = 2; i < n + 2; i++) {
		obj = &graph->Package.Elements[i];
		if (obj->Type != ACPI_TYPE_PACKAGE || obj->Package.Count < 3)
			return (false);
	}

	return (true);
}

static inline bool
is_acpi_guid(const union acpi_object *obj)
{

	return (obj->Type == ACPI_TYPE_BUFFER) && (obj->Buffer.Length == 16);
}

static inline bool
guid_equal(const struct uuid *u1, const struct uuid *u2)
{

	if (memcmp(u1, u2, 16) == 0)
		return (true);

	return (false);
}

static inline bool
acpi_guid_matches(const union acpi_object *obj, const struct uuid *guid)
{

	if (is_acpi_guid(obj) &&
	    guid_equal((struct uuid *)obj->Buffer.Pointer, guid))
		return (true);

	return (false);
}

static inline bool
is_acpi_dsd_graph_guid(const union acpi_object *obj)
{

	return (acpi_guid_matches(obj, &acpi_graph_uuid));
}

static inline bool
is_acpi_coresight_graph_guid(const union acpi_object *obj)
{

	return (acpi_guid_matches(obj, &coresight_graph_uuid));
}

static inline bool
is_acpi_coresight_graph(const union acpi_object *obj)
{
	const union acpi_object *graphid, *guid, *links;

	if (obj->Type != ACPI_TYPE_PACKAGE ||
	    obj->Package.Count < 3)
		return (false);

	graphid = &obj->Package.Elements[0];
	guid = &obj->Package.Elements[1];
	links = &obj->Package.Elements[2];

	if (graphid->Type != ACPI_TYPE_INTEGER ||
	    links->Type != ACPI_TYPE_INTEGER)
		return (false);

	if (is_acpi_coresight_graph_guid(guid))
		return (true);

	return (false);
}


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

static const union acpi_object *
coresight_get_dsd_graph(device_t dev)
{
	union acpi_object *dsd;
	ACPI_STATUS status;
	ACPI_BUFFER buf;
	device_t bus;
	const union acpi_object *guid, *package;
	int i;

	buf.Length = 4096;
	buf.Pointer = malloc(buf.Length, M_TEMP, M_NOWAIT | M_ZERO);
	if (buf.Pointer == NULL) {
		printf("failed to allocate memory\n");
		return (NULL);
	}

	bus = device_get_parent(dev);
	status = ACPI_EVALUATE_OBJECT(bus, dev, "_DSD", NULL, &buf);
	if (ACPI_FAILURE(status)) {
		printf("failed to evaluate object\n");
		return (NULL);
	}

	printf("buf.Length %ld\n", buf.Length);

	dsd = buf.Pointer;
	printf("dsd pc %d\n", dsd->Package.Count);

	for (i = 0; i + 1 < dsd->Package.Count; i += 2) {
		guid = &dsd->Package.Elements[i];
		package = &dsd->Package.Elements[i + 1];

		if (!is_acpi_guid(guid) || package->Type != ACPI_TYPE_PACKAGE)
			break;

		if (!is_acpi_dsd_graph_guid(guid))
			continue;

		printf("graph uuid found\n");

		if (acpi_validate_dsd_graph(package)) {
			printf("dsd graph validated\n");
			return (package);
		}
	}

	return (NULL);
}

static inline bool
acpi_validate_coresight_graph(const union acpi_object *cs_graph)
{
	int nlinks;

	nlinks = cs_graph->Package.Elements[2].Integer.Value;
	if (cs_graph->Package.Count != (nlinks + 3))
		return (false);

	return (true);
}

static const union acpi_object *
coresight_get_coresight_graph(device_t dev)
{
	const union acpi_object *graph_list, *graph;
	int i, nr_graphs;

	graph_list = coresight_get_dsd_graph(dev);
	if (!graph_list) {
		printf("failed to get graph list\n");
		return (NULL);
	}

	nr_graphs = graph_list->Package.Elements[1].Integer.Value;
	for (i = 2; i < nr_graphs + 2; i++) {
		graph = &graph_list->Package.Elements[i];
		if (!is_acpi_coresight_graph(graph))
			continue;
		if (acpi_validate_coresight_graph(graph))
			return (graph);
		break;
	}

	return (NULL);
}

static int
acpi_coresight_parse_link(device_t dev,
    const union acpi_object *link,
    struct coresight_connection *conn)
{
	const union acpi_object *fields;
	int dir;

	if (link->Type != ACPI_TYPE_PACKAGE ||
	    link->Package.Count != 4)
		return (ENXIO);

	fields = link->Package.Elements;
	if (fields[0].Type != ACPI_TYPE_INTEGER ||
	    fields[1].Type != ACPI_TYPE_INTEGER ||
	    fields[2].Type != ACPI_TYPE_LOCAL_REFERENCE ||
	    fields[3].Type != ACPI_TYPE_INTEGER)
		return (ENXIO);

	dir = fields[3].Integer.Value;
	if (dir == ACPI_CORESIGHT_LINK_MASTER) {
		printf("direction output\n");
	} else
		printf("direction input\n");

	return (0);
}

static int
coresight_get_ports(device_t dev,
    struct coresight_platform_data *pdata)
{
	const union acpi_object *graph;
	int nlinks;
	int i;

	graph = coresight_get_coresight_graph(dev);
	if (graph)
		printf("coresight graph found\n");

	nlinks = graph->Package.Elements[2].Integer.Value;
	if (!nlinks)
		return (0);

	printf("nlinks %d\n", nlinks);

	const union acpi_object *link;
	struct coresight_connection conn;
	struct coresight_connection *ptr;

	int dir;

	ptr = &conn;

	for (i = 0; i < nlinks; i++) {
		link = &graph->Package.Elements[3 + i];

		dir = acpi_coresight_parse_link(dev, link, ptr);
		if (dir < 0)
			return (dir);

		if (dir == ACPI_CORESIGHT_LINK_MASTER) {
			pdata->out_ports++;
			ptr++;
		} else {
			pdata->in_ports++;
		}
	}

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
