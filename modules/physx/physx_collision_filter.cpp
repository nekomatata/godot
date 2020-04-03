/*************************************************************************/
/*  physx_collision_filter.cpp                                           */
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

#include "physx_collision_filter.h"

using namespace physx;

enum QueryFlag {
	QUERY_FLAG_COLLIDE_WITH_BODIES = (1 << 0),
	QUERY_FLAG_COLLIDE_WITH_AREAS = (1 << 1),
};

// Collision filter data
void set_filter_data(PxFilterData &p_filter_data, uint32_t p_collision_layer, uint32_t p_collision_mask) {
	p_filter_data.word0 = p_collision_layer;
	p_filter_data.word1 = p_collision_mask;
}

void set_query_filter_data(PxFilterData &p_filter_data, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	p_filter_data.word0 = 0;
	p_filter_data.word1 = p_collision_mask;

	if (p_collide_with_bodies) {
		p_filter_data.word2 |= QUERY_FLAG_COLLIDE_WITH_BODIES;
	}

	if (p_collide_with_areas) {
		p_filter_data.word2 |= QUERY_FLAG_COLLIDE_WITH_AREAS;
	}
}

uint32_t get_collision_layer(const PxFilterData &p_filter_data) {
	return p_filter_data.word0;
}

uint32_t get_collision_mask(const PxFilterData &p_filter_data) {
	return p_filter_data.word1;
}

// Collision filter shader
extern PxFilterFlags filter_shader(PxFilterObjectAttributes attributes_0, PxFilterData filter_data_0, PxFilterObjectAttributes attributes_1, PxFilterData filter_data_1, PxPairFlags &pair_flags, const void *constant_block, PxU32 constant_block_size) {
	// Handle trigger detection or contacts.
	if (PxFilterObjectIsTrigger(attributes_0) || PxFilterObjectIsTrigger(attributes_1)) {
		pair_flags = PxPairFlag::eTRIGGER_DEFAULT;
	} else {
		pair_flags = PxPairFlag::eCONTACT_DEFAULT;
	}

	// Filter collision detection
	uint32_t collision_layer_0 = get_collision_layer(filter_data_0);
	uint32_t collision_layer_1 = get_collision_layer(filter_data_1);
	uint32_t collision_mask_0 = get_collision_mask(filter_data_0);
	uint32_t collision_mask_1 = get_collision_mask(filter_data_1);
	if ((collision_layer_0 & collision_mask_1) || (collision_layer_1 & collision_mask_0)) {
		return PxFilterFlag::eDEFAULT;
	} else {
		return PxFilterFlag::eSUPPRESS;
	}
}
