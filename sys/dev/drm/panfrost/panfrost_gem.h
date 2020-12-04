/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
 *
 * This work was supported by Innovate UK project 105694, "Digital Security
 * by Design (DSbD) Technology Platform Prototype".
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
 *
 * $FreeBSD$
 */

#ifndef	_DEV_DRM_PANFROST_PANFROST_GEM_H_
#define	_DEV_DRM_PANFROST_PANFROST_GEM_H_

struct panfrost_gem_object *panfrost_gem_create_object(struct drm_file *file,
    struct drm_device *dev, size_t size, uint32_t flags, uint32_t *handle);
struct panfrost_gem_mapping *
    panfrost_gem_mapping_get(struct panfrost_gem_object *bo,
    struct panfrost_file *priv);

struct panfrost_gem_object {
	struct drm_gem_object base;	/* Must go first */
	vm_page_t pages;
	bool noexec;
	bool is_heap;
	TAILQ_HEAD(, panfrost_gem_mapping)	mappings;
};

struct panfrost_gem_mapping {
	struct panfrost_gem_object *obj;
	struct drm_mm_node mmnode;
	struct panfrost_mmu *mmu;
	TAILQ_ENTRY(panfrost_gem_mapping)	next;
	bool active;
};

#endif /* !_DEV_DRM_PANFROST_PANFROST_GEM_H_ */
