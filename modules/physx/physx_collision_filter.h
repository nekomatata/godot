/*************************************************************************/
/*  physx_collision_filter.h                                             */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2020 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2020 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef PHYSX_COLLISION_FILTER_H
#define PHYSX_COLLISION_FILTER_H

#include "core/int_types.h"

#include "PxFiltering.h"

// Collision filter data
extern void set_filter_data(physx::PxFilterData &p_filter_data, uint32_t p_collision_layer, uint32_t p_collision_mask);
extern void set_query_filter_data(physx::PxFilterData &p_filter_data, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas);
extern uint32_t get_collision_layer(const physx::PxFilterData &p_filter_data);
extern uint32_t get_collision_mask(const physx::PxFilterData &p_filter_data);

// Collision filter shader
extern physx::PxFilterFlags filter_shader(physx::PxFilterObjectAttributes attributes_0, physx::PxFilterData filter_data_0, physx::PxFilterObjectAttributes attributes_1, physx::PxFilterData filter_data_1, physx::PxPairFlags &pair_flags, const void *constant_block, physx::PxU32 constant_block_size);

#endif
