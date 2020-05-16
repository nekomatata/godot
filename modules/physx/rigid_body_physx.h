/*************************************************************************/
/*  rigid_body_physx.h                                                   */
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

#ifndef RIGID_BODY_PHYSX_H
#define RIGID_BODY_PHYSX_H

#include "collision_object_physx.h"
#include "core/int_types.h"
#include "core/math/math_defs.h"
#include "core/math/vector3.h"
#include "core/object_id.h"
#include "core/string_name.h"
#include "core/variant.h"
#include "servers/physics_server_3d.h"

#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

namespace physx {
class PxRigidDynamic;
} // namespace physx

class RigidBodyPhysX : public RigidCollisionObjectPhysX {
	friend class PhysXPhysicsDirectBodyState3D;

public:
	struct ForceIntegrationCallback {
		ObjectID id;
		StringName method;
		Variant udata;
	};

private:
	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;
	physx::PxRigidDynamic *px_rigid_dynamic = nullptr;
	uint16_t locked_axis = 0;
	real_t mass = 1.0;
	real_t gravity_scale = 1.0;
	physx::PxVec3 px_total_gravity = physx::PxVec3(0.0);
	real_t linear_damping = 0.0;
	real_t angular_damping = 0.0;
	bool can_sleep = true;
	bool omit_force_integration = false;

	int spOv_gravity_point_count = 0;
	bool spOv_need_update = false;

	Vector3 applied_force;
	Vector3 applied_torque;

	ForceIntegrationCallback *force_integration_callback = nullptr;

	Vector<AreaPhysX *> areas_overlapped;

public:
	RigidBodyPhysX();
	~RigidBodyPhysX();

	_FORCE_INLINE_ physx::PxRigidDynamic *get_px_rigid_dynamic() { return px_rigid_dynamic; }

	virtual void set_space(SpacePhysX *p_space);

	virtual void on_pre_simulation();
	void set_force_integration_callback(ObjectID p_id, const StringName &p_method, const Variant &p_udata = Variant());

	void set_sleeping(bool p_sleeping);
	bool is_sleeping() const;

	void set_omit_force_integration(bool p_omit);
	_FORCE_INLINE_ bool get_omit_force_integration() const { return omit_force_integration; }

	void set_param(PhysicsServer3D::BodyParameter p_param, real_t);
	real_t get_param(PhysicsServer3D::BodyParameter p_param) const;

	void set_mode(PhysicsServer3D::BodyMode p_mode);
	PhysicsServer3D::BodyMode get_mode() const;

	void set_state(PhysicsServer3D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer3D::BodyState p_state) const;

	void apply_impulse(const Vector3 &p_pos, const Vector3 &p_impulse);
	void apply_central_impulse(const Vector3 &p_impulse);
	void apply_torque_impulse(const Vector3 &p_impulse);

	void apply_central_force(const Vector3 &p_force);
	void apply_force(const Vector3 &p_force, const Vector3 &p_pos);
	void apply_torque(const Vector3 &p_torque);

	void set_applied_force(const Vector3 &p_force);
	const Vector3 &get_applied_force() const { return applied_force; }
	void set_applied_torque(const Vector3 &p_torque);
	const Vector3 &get_applied_torque() const { return applied_torque; }

	void set_axis_lock(PhysicsServer3D::BodyAxis p_axis, bool lock);
	bool is_axis_locked(PhysicsServer3D::BodyAxis p_axis) const;

	void set_continuous_collision_detection(bool p_enable);
	bool is_continuous_collision_detection_enabled() const;

	void set_linear_velocity(const Vector3 &p_velocity);
	Vector3 get_linear_velocity() const;

	void set_angular_velocity(const Vector3 &p_velocity);
	Vector3 get_angular_velocity() const;

	virtual void on_enter_area(AreaPhysX *p_area);
	virtual void on_exit_area(AreaPhysX *p_area);

	void spOv_mark_for_update() { spOv_need_update = true; }

protected:
	virtual void reload_shapes();

	virtual void set_px_transform(const physx::PxTransform &p_global_transform);

private:
	void spOv_update();

	void create_rigid_body_internal();
	void destroy_rigid_body_internal();

	void update_transform_internal();

	void update_mass_internal();
	void update_axis_lock_internal();
	void update_can_sleep_internal();

	void apply_gravity_internal();
};
#endif
