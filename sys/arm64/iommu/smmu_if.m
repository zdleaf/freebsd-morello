#-
# SPDX-License-Identifier: BSD-2-Clause
#:
# Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
#
# This software was developed by SRI International and the University of
# Cambridge Computer Laboratory (Department of Computer Science and
# Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
# DARPA SSITH research programme.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# $FreeBSD$
#

#include <vm/vm.h>

#include <sys/types.h>
#include <sys/taskqueue.h>
#include <sys/bus.h>
#include <sys/sysctl.h>
#include <sys/tree.h>
#include <sys/mutex.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/iommu/iommu.h>

#include <arm64/iommu/iommu.h>

INTERFACE smmu;

#
# Map a virtual address VA to a physical address PA.
#
METHOD int map {
	device_t		dev;
	struct smmu_domain	*domain;
	vm_offset_t		va;
	vm_page_t		*ma;
	bus_size_t		size;
	vm_prot_t		prot;
};

#
# Map a virtual address VA to a physical address PA.
#
METHOD int map_page {
	device_t		dev;
	struct smmu_domain	*domain;
	vm_offset_t		va;
	vm_paddr_t		pa;
	vm_prot_t		prot;
};

#
# Unmap a virtual address VA.
#
METHOD int unmap {
	device_t		dev;
	struct smmu_domain	*domain;
	vm_offset_t		va;
	bus_size_t		size;
};

#
# Allocate an IOMMU domain.
#
METHOD struct smmu_domain * domain_alloc {
	device_t		dev;
};

#
# Release all the resources held by IOMMU domain.
#
METHOD int domain_free {
	device_t		dev;
	struct smmu_domain	*domain;
};

#
# Attach a consumer device to a IOMMU domain.
#
METHOD int ctx_attach {
	device_t		dev;
	struct smmu_domain	*domain;
	struct smmu_ctx		*ctx;
};

#
# Detach a consumer device from IOMMU domain.
#
METHOD int ctx_detach {
	device_t		dev;
	struct smmu_ctx		*ctx;
};
