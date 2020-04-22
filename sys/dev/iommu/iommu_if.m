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

#include <dev/iommu/iommu.h>

INTERFACE iommu;

METHOD int map {
	device_t		dev;
	struct iommu_domain	*domain;
	vm_offset_t		va;
	vm_paddr_t		pa;
	vm_size_t		size;
	vm_prot_t		prot;
};

METHOD int unmap {
	device_t		dev;
	struct iommu_domain	*domain;
	vm_offset_t		va;
	vm_size_t		size;
};

METHOD struct iommu_domain * domain_alloc {
	device_t		dev;
};

METHOD void domain_free {
	device_t		dev;
	struct iommu_domain	*domain;
};

METHOD int add_device {
	device_t		smmu_dev;
	struct iommu_domain	*domain;
	struct iommu_device	*device;
};

METHOD int capable {
	device_t		smmu_dev;
	device_t		dev;
};
