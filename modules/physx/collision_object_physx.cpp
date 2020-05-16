/*************************************************************************/
/*  collision_object_physx.cpp                                           */
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

#include "collision_object_physx.h"

#include "area_physx.h"
#include "physx_collision_filter.h"
#include "physx_types_converter.h"
#include "shape_physx.h"
#include "space_physx.h"

#include "PxMaterial.h"
#include "PxPhysics.h"
#include "PxRigidActor.h"
#include "PxShape.h"
#include "foundation/PxTransform.h"

using namespace physx;

CollisionObjectPhysX::CollisionObjectPhysX(Type p_type) :
		type(p_type) {
}

CollisionObjectPhysX::~CollisionObjectPhysX() {
	destroy_actor();
}

void CollisionObjectPhysX::set_body_scale(const Vector3 &p_scale) {
	if (!p_scale.is_equal_approx(body_scale)) {
		body_scale = p_scale;
		body_scale_changed();
	}
}

void CollisionObjectPhysX::body_scale_changed() {
	if (px_actor) {
		force_shape_reset = true;
	}
}

void CollisionObjectPhysX::setup_actor(PxRigidActor *p_actor) {
	px_actor = p_actor;
	p_actor->userData = this;
}

void CollisionObjectPhysX::destroy_actor() {
	if (!px_actor) {
		return;
	}

	px_actor->release();
	px_actor = nullptr;
}

void CollisionObjectPhysX::add_collision_exception(const CollisionObjectPhysX *p_other_object) {
	exceptions.insert(p_other_object->get_self());
	ERR_FAIL_MSG("collision exception is not implemented in PhysX.");
}

void CollisionObjectPhysX::remove_collision_exception(const CollisionObjectPhysX *p_other_object) {
	exceptions.erase(p_other_object->get_self());
	ERR_FAIL_MSG("collision exception is not implemented in PhysX.");
}

bool CollisionObjectPhysX::has_collision_exception(const CollisionObjectPhysX *p_other_object) const {
	return exceptions.has(p_other_object->get_self());
}

void CollisionObjectPhysX::set_transform(const Transform &p_global_transform) {
	set_body_scale(p_global_transform.basis.get_scale_abs());
	G_TO_PX(p_global_transform, body_transform);
	set_px_transform(body_transform);
}

Transform CollisionObjectPhysX::get_transform() const {
	Transform godot_transform;
	PX_TO_G(body_transform, godot_transform);
	godot_transform.basis.scale(body_scale);
	return godot_transform;
}

void CollisionObjectPhysX::set_px_transform(const PxTransform &p_global_transform) {
	if (!px_actor) {
		return;
	}

	px_actor->setGlobalPose(p_global_transform);
}

RigidCollisionObjectPhysX::RigidCollisionObjectPhysX(Type p_type) :
		CollisionObjectPhysX(p_type),
		px_material(nullptr),
		friction(0.0),
		restitution(0.0) {
}

RigidCollisionObjectPhysX::~RigidCollisionObjectPhysX() {
	remove_all_shapes(false);

	if (px_material) {
		px_material->release();
		px_material = nullptr;
	}
}

void RigidCollisionObjectPhysX::add_shape(ShapePhysX *p_shape, const Transform &p_transform, bool p_disabled) {
	ShapeWrapper shape_wrapper(p_shape, !p_disabled);
	G_TO_PX(p_transform, shape_wrapper.shape_instance.transform);
	G_TO_PX(p_transform.basis.get_scale_abs(), shape_wrapper.shape_instance.scale);
	shapes.push_back(shape_wrapper);
	p_shape->add_owner(this);
	reload_shapes();
}

void RigidCollisionObjectPhysX::set_shape(int p_index, ShapePhysX *p_shape) {
	ShapeWrapper &shp = shapes.write[p_index];
	shp.shape->remove_owner(this);
	p_shape->add_owner(this);
	shp.shape = p_shape;
	reload_shapes();
}

int RigidCollisionObjectPhysX::get_shape_count() const {
	return shapes.size();
}

ShapePhysX *RigidCollisionObjectPhysX::get_shape(int p_index) const {
	return shapes[p_index].shape;
}

PxShape *RigidCollisionObjectPhysX::get_px_shape(int p_index) const {
	return shapes[p_index].px_shape;
}

int RigidCollisionObjectPhysX::find_shape(ShapePhysX *p_shape) const {
	const int size = shapes.size();
	for (int i = 0; i < size; ++i) {
		if (shapes[i].shape == p_shape) {
			return i;
		}
	}
	return -1;
}

void RigidCollisionObjectPhysX::remove_shape(ShapePhysX *p_shape) {
	// Remove the shape, all the times it appears
	// Reverse order required for delete.
	for (int i = shapes.size() - 1; 0 <= i; --i) {
		if (p_shape == shapes[i].shape) {
			remove_shape_internal(i);
			shapes.remove(i);
		}
	}
	reload_shapes();
}

void RigidCollisionObjectPhysX::remove_shape(int p_index) {
	ERR_FAIL_INDEX(p_index, get_shape_count());
	remove_shape_internal(p_index);
	shapes.remove(p_index);
	reload_shapes();
}

void RigidCollisionObjectPhysX::remove_all_shapes(bool p_reload_shapes) {
	// Reverse order required for delete.
	for (int i = shapes.size() - 1; 0 <= i; --i) {
		remove_shape_internal(i);
	}
	shapes.clear();
	if (p_reload_shapes) {
		reload_shapes();
	}
}

void RigidCollisionObjectPhysX::set_shape_transform(int p_index, const Transform &p_transform) {
	ERR_FAIL_INDEX(p_index, get_shape_count());

	ShapeWrapper &shape_wrapper = shapes.write[p_index];
	G_TO_PX(p_transform, shape_wrapper.shape_instance.transform);

	Vector3 prev_scale;
	PX_TO_G(shape_wrapper.shape_instance.scale, prev_scale);

	Vector3 shape_scale = p_transform.basis.get_scale_abs();

	if (!shape_scale.is_equal_approx(prev_scale)) {
		G_TO_PX(shape_scale, shape_wrapper.shape_instance.scale);
		shape_changed(p_index);
	} else if (px_actor) {
		// No need to recreate, just update the local transform
		PxVec3 px_body_scale;
		G_TO_PX(body_scale, px_body_scale);
		if (shape_wrapper.attached) {
			detach_shape_internal(shape_wrapper, false);
			attach_shape_internal(shape_wrapper, px_body_scale);
		} else {
			shape_wrapper.shape->set_transform(shape_wrapper.px_shape, shape_wrapper.shape_instance.transform, px_body_scale);
		}
	}
}

Transform RigidCollisionObjectPhysX::get_shape_transform(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, get_shape_count(), Transform());

	Transform godot_transform;
	PX_TO_G(shapes[p_index].shape_instance.transform, godot_transform);
	return godot_transform;
}

void RigidCollisionObjectPhysX::set_shape_disabled(int p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, get_shape_count());

	if (shapes[p_index].active != p_disabled) {
		return;
	}
	shapes.write[p_index].active = !p_disabled;
	shape_changed(p_index);
}

bool RigidCollisionObjectPhysX::is_shape_disabled(int p_index) {
	ERR_FAIL_INDEX_V(p_index, get_shape_count(), false);

	return !shapes[p_index].active;
}

void RigidCollisionObjectPhysX::shape_changed(int p_shape_index) {
	if (px_actor) {
		ShapeWrapper &shape_wrapper = shapes.write[p_shape_index];
		detach_shape_internal(shape_wrapper);
		reload_shapes();
	}
}

void RigidCollisionObjectPhysX::on_collision_filter_change() {
	if (!px_actor) {
		return;
	}

	const int shape_count = shapes.size();
	for (int shape_index = 0; shape_index < shape_count; ++shape_index) {
		ShapeWrapper &shape_wrapper = shapes.write[shape_index];
		if (shape_wrapper.px_shape) {
			update_shape_collision_filter(shape_wrapper.px_shape);
		}
	}
}

void RigidCollisionObjectPhysX::update_shape_collision_filter(PxShape *p_shape) {
	PxFilterData filter_data;
	set_filter_data(filter_data, collision_layer, collision_mask);

	p_shape->setSimulationFilterData(filter_data);
	p_shape->setQueryFilterData(filter_data);
}

void RigidCollisionObjectPhysX::setup_shape(PxShape *p_shape) {
	update_shape_collision_filter(p_shape);
}

void RigidCollisionObjectPhysX::reload_shapes() {
	if (!px_actor) {
		return;
	}

	PxVec3 px_body_scale;
	G_TO_PX(body_scale, px_body_scale);

	const int shape_count = shapes.size();
	for (int shape_index = 0; shape_index < shape_count; ++shape_index) {
		ShapeWrapper &shape_wrapper = shapes.write[shape_index];
		if (force_shape_reset) {
			detach_shape_internal(shape_wrapper);
		}

		if (!shape_wrapper.active) {
			detach_shape_internal(shape_wrapper);
			continue;
		}

		if (!shape_wrapper.px_shape) {
			if (!px_material) {
				px_material = PxGetPhysics().createMaterial(friction, friction, restitution);
				ERR_FAIL_COND_MSG(!px_material, "Failed to create shape material.");
			}

			shape_wrapper.shape_instance.material = px_material;
			shape_wrapper.px_shape = shape_wrapper.shape->create(shape_wrapper.shape_instance, px_body_scale);
			setup_shape(shape_wrapper.px_shape);
		}

		if (!shape_wrapper.attached) {
			attach_shape_internal(shape_wrapper, px_body_scale);
		}
	}

	force_shape_reset = false;
}

void RigidCollisionObjectPhysX::release_shapes() {
	if (!px_actor) {
		return;
	}

	const int shape_count = shapes.size();
	for (int shape_index = 0; shape_index < shape_count; ++shape_index) {
		ShapeWrapper &shape_wrapper = shapes.write[shape_index];
		detach_shape_internal(shape_wrapper);
	}

	force_shape_reset = false;
}

void RigidCollisionObjectPhysX::set_friction(float p_friction) {
	friction = p_friction;
	if (px_material) {
		px_material->setDynamicFriction(p_friction);
		px_material->setStaticFriction(p_friction);
	}
}

void RigidCollisionObjectPhysX::set_restitution(float p_restitution) {
	restitution = p_restitution;
	if (px_material) {
		px_material->setRestitution(p_restitution);
	}
}

void RigidCollisionObjectPhysX::destroy_actor() {
	if (!px_actor) {
		return;
	}

	const int shape_count = shapes.size();
	for (int shape_index = 0; shape_index < shape_count; ++shape_index) {
		ShapeWrapper &shape_wrapper = shapes.write[shape_index];
		detach_shape_internal(shape_wrapper, false);
	}

	CollisionObjectPhysX::destroy_actor();
}

void RigidCollisionObjectPhysX::body_scale_changed() {
	CollisionObjectPhysX::body_scale_changed();
	reload_shapes();
}

void RigidCollisionObjectPhysX::attach_shape_internal(ShapeWrapper &p_shape_wrapper, const PxVec3 &p_global_scale) {
	p_shape_wrapper.shape->set_transform(p_shape_wrapper.px_shape, p_shape_wrapper.shape_instance.transform, p_global_scale);
	px_actor->attachShape(*p_shape_wrapper.px_shape);
	p_shape_wrapper.attached = true;
}

void RigidCollisionObjectPhysX::detach_shape_internal(ShapeWrapper &p_shape_wrapper, bool p_release) {
	if (p_shape_wrapper.attached) {
		px_actor->detachShape(*p_shape_wrapper.px_shape);
		p_shape_wrapper.attached = false;
	}
	if (p_release && p_shape_wrapper.px_shape) {
		p_shape_wrapper.px_shape->release();
		p_shape_wrapper.px_shape = nullptr;
	}
}

void RigidCollisionObjectPhysX::remove_shape_internal(int p_index) {
	ShapeWrapper &shape_wrapper = shapes.write[p_index];
	shape_wrapper.shape->remove_owner(this);
	detach_shape_internal(shape_wrapper);
}
