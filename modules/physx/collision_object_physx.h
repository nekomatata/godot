/*************************************************************************/
/*  collision_object_physx.h                                             */
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

#ifndef COLLISION_OBJECT_PHYSX_H
#define COLLISION_OBJECT_PHYSX_H

#include "core/int_types.h"
#include "core/math/transform.h"
#include "core/math/vector3.h"
#include "core/object_id.h"
#include "core/typedefs.h"
#include "core/vector.h"
#include "core/vset.h"
#include "rid_physx.h"
#include "shape_instance_physx.h"
#include "shape_owner_physx.h"

#include "foundation/PxTransform.h"

class AreaPhysX;
class ShapePhysX;
class SpacePhysX;

namespace physx {
class PxRigidActor;
class PxShape;
} // namespace physx

class CollisionObjectPhysX : public RIDPhysX {
public:
	enum Type {
		TYPE_AREA = 0,
		TYPE_RIGID_BODY,
		TYPE_SOFT_BODY,
		TYPE_KINEMATIC_GHOST_BODY
	};

	struct ShapeWrapper {
		ShapePhysX *shape = nullptr;
		ShapeInstancePhysX shape_instance;
		physx::PxShape *px_shape = nullptr;
		bool active = true;
		bool attached = false;

		ShapeWrapper() {
		}

		ShapeWrapper(ShapePhysX *p_shape, bool p_active) :
				shape(p_shape),
				active(p_active) {
		}

		ShapeWrapper(const ShapeWrapper &otherShape) {
			operator=(otherShape);
		}

		void operator=(const ShapeWrapper &otherShape) {
			shape = otherShape.shape;
			shape_instance = otherShape.shape_instance;
			px_shape = otherShape.px_shape;
			active = otherShape.active;
			attached = otherShape.attached;
		}
	};

protected:
	Type type;
	ObjectID instance_id;
	uint32_t collision_layer = 0;
	uint32_t collision_mask = 0;
	bool ray_pickable = false;
	physx::PxRigidActor *px_actor = nullptr;
	Vector3 body_scale = Vector3(1.0, 1.0, 1.0);
	physx::PxTransform body_transform;
	bool force_shape_reset = false;
	SpacePhysX *space = nullptr;

	VSet<RID> exceptions;

	bool transform_changed;

public:
	CollisionObjectPhysX(Type p_type);
	virtual ~CollisionObjectPhysX();

	Type getType() { return type; }

	_FORCE_INLINE_ physx::PxRigidActor *get_px_actor() const { return px_actor; }

	_FORCE_INLINE_ void set_instance_id(const ObjectID &p_instance_id) { instance_id = p_instance_id; }
	_FORCE_INLINE_ ObjectID get_instance_id() const { return instance_id; }

	_FORCE_INLINE_ void set_ray_pickable(bool p_enable) { ray_pickable = p_enable; }
	_FORCE_INLINE_ bool is_ray_pickable() const { return ray_pickable; }

	void set_body_scale(const Vector3 &p_scale);
	const Vector3 &get_body_scale() const { return body_scale; }
	virtual void body_scale_changed();

	void add_collision_exception(const CollisionObjectPhysX *p_other_object);
	void remove_collision_exception(const CollisionObjectPhysX *p_other_object);
	bool has_collision_exception(const CollisionObjectPhysX *p_other_object) const;
	_FORCE_INLINE_ const VSet<RID> &get_exceptions() const { return exceptions; }

	_FORCE_INLINE_ void set_collision_layer(uint32_t p_layer) {
		if (collision_layer != p_layer) {
			collision_layer = p_layer;
			on_collision_filter_change();
		}
	}
	_FORCE_INLINE_ uint32_t get_collision_layer() const { return collision_layer; }

	_FORCE_INLINE_ void set_collision_mask(uint32_t p_mask) {
		if (collision_mask != p_mask) {
			collision_mask = p_mask;
			on_collision_filter_change();
		}
	}
	_FORCE_INLINE_ uint32_t get_collision_mask() const { return collision_mask; }

	virtual void set_space(SpacePhysX *p_space) = 0;
	_FORCE_INLINE_ SpacePhysX *get_space() const { return space; }

	virtual void on_pre_simulation() {}

	virtual void on_enter_area(AreaPhysX *p_area) {}
	virtual void on_exit_area(AreaPhysX *p_area) {}

	void set_transform(const Transform &p_global_transform);
	Transform get_transform() const;

protected:
	void setup_actor(physx::PxRigidActor *p_actor);
	virtual void destroy_actor();

	virtual void on_collision_filter_change() = 0;

	virtual void set_px_transform(const physx::PxTransform &p_global_transform);
};

class RigidCollisionObjectPhysX : public CollisionObjectPhysX, public ShapeOwnerPhysX {
protected:
	Vector<ShapeWrapper> shapes;

	physx::PxMaterial *px_material;
	float friction;
	float restitution;

	virtual void destroy_actor();

	virtual void on_collision_filter_change();

	virtual void setup_shape(physx::PxShape *p_shape);
	virtual void reload_shapes();

	void release_shapes();

	void set_friction(float p_friction);
	void set_restitution(float p_restitution);

	float get_friction() const { return friction; }
	float get_restitution() const { return restitution; }

public:
	RigidCollisionObjectPhysX(Type p_type);
	~RigidCollisionObjectPhysX();

	_FORCE_INLINE_ const Vector<ShapeWrapper> &get_shapes_wrappers() const { return shapes; }

	void add_shape(ShapePhysX *p_shape, const Transform &p_transform = Transform(), bool p_disabled = false);
	void set_shape(int p_index, ShapePhysX *p_shape);

	int get_shape_count() const;
	ShapePhysX *get_shape(int p_index) const;
	physx::PxShape *get_px_shape(int p_index) const;

	// ShapeOwnerPhysX interface
	virtual int find_shape(ShapePhysX *p_shape) const;
	virtual void remove_shape(ShapePhysX *p_shape);
	virtual void shape_changed(int p_shape_index);

	void remove_shape(int p_index);
	void remove_all_shapes(bool p_reload_shapes = true);

	void set_shape_transform(int p_index, const Transform &p_transform);
	Transform get_shape_transform(int p_index) const;

	void set_shape_disabled(int p_index, bool p_disabled);
	bool is_shape_disabled(int p_index);

	virtual void body_scale_changed();

private:
	void attach_shape_internal(ShapeWrapper &p_shape_wrapper, const physx::PxVec3 &p_global_scale);
	void detach_shape_internal(ShapeWrapper &p_shape_wrapper, bool p_release = true);
	void remove_shape_internal(int p_index);

	void update_shape_collision_filter(physx::PxShape *p_shape);
};

#endif
