/*************************************************************************/
/*  area_physx.h                                                         */
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

#ifndef AREA_PHYSX_H
#define AREA_PHYSX_H

#include "collision_object_physx.h"
#include "core/math/vector3.h"
#include "core/object_id.h"
#include "core/string_name.h"
#include "core/variant.h"
#include "servers/physics_server_3d.h"

class SpacePhysX;

class AreaPhysX : public RigidCollisionObjectPhysX {
public:
	struct InOutEventCallback {
		ObjectID event_callback_id;
		StringName event_callback_method;

		InOutEventCallback() {}
	};

private:
	bool monitorable;
	bool monitoring;

	PhysicsServer3D::AreaSpaceOverrideMode spOv_mode;
	bool spOv_gravity_point;
	real_t spOv_gravity_point_distance_scale;
	real_t spOv_gravity_point_attenuation;
	Vector3 spOv_gravity_vector;
	real_t spOv_gravity_magnitude;
	real_t spOv_linear_damping;
	real_t spOv_angular_damping;
	int spOv_priority;

	static const int EVENT_CALLBACK_COUNT = 2;
	InOutEventCallback event_callbacks[EVENT_CALLBACK_COUNT];

public:
	AreaPhysX();
	~AreaPhysX();

	void set_monitorable(bool p_monitorable);
	_FORCE_INLINE_ bool is_monitorable() const { return monitorable; }
	_FORCE_INLINE_ bool is_monitoring() const { return monitoring; }

	_FORCE_INLINE_ void set_spOv_mode(PhysicsServer3D::AreaSpaceOverrideMode p_mode) { spOv_mode = p_mode; }
	_FORCE_INLINE_ PhysicsServer3D::AreaSpaceOverrideMode get_spOv_mode() const { return spOv_mode; }

	_FORCE_INLINE_ void set_spOv_gravity_point(bool p_isGP) { spOv_gravity_point = p_isGP; }
	_FORCE_INLINE_ bool is_spOv_gravity_point() const { return spOv_gravity_point; }

	_FORCE_INLINE_ void set_spOv_gravity_point_distance_scale(real_t p_GPDS) { spOv_gravity_point_distance_scale = p_GPDS; }
	_FORCE_INLINE_ real_t get_spOv_gravity_point_distance_scale() const { return spOv_gravity_point_distance_scale; }

	_FORCE_INLINE_ void set_spOv_gravity_point_attenuation(real_t p_GPA) { spOv_gravity_point_attenuation = p_GPA; }
	_FORCE_INLINE_ real_t get_spOv_gravity_point_attenuation() const { return spOv_gravity_point_attenuation; }

	_FORCE_INLINE_ void set_spOv_gravity_vector(Vector3 p_vec) { spOv_gravity_vector = p_vec; }
	_FORCE_INLINE_ const Vector3 &get_spOv_gravity_vector() const { return spOv_gravity_vector; }

	_FORCE_INLINE_ void set_spOv_gravity_magnitude(real_t p_gravityMag) { spOv_gravity_magnitude = p_gravityMag; }
	_FORCE_INLINE_ real_t get_spOv_gravity_magnitude() const { return spOv_gravity_magnitude; }

	_FORCE_INLINE_ void set_spOv_linear_damping(real_t p_linearDump) { spOv_linear_damping = p_linearDump; }
	_FORCE_INLINE_ real_t get_spOv_linear_damping() const { return spOv_linear_damping; }

	_FORCE_INLINE_ void set_spOv_angular_damping(real_t p_angularDump) { spOv_angular_damping = p_angularDump; }
	_FORCE_INLINE_ real_t get_spOv_angular_damping() const { return spOv_angular_damping; }

	_FORCE_INLINE_ void set_spOv_priority(int p_priority) { spOv_priority = p_priority; }
	_FORCE_INLINE_ int get_spOv_priority() const { return spOv_priority; }

	virtual void set_space(SpacePhysX *p_space);

	void call_event(CollisionObjectPhysX *p_otherObject, PhysicsServer3D::AreaBodyStatus p_status);

	void on_collision_entered(CollisionObjectPhysX *p_otherObject);
	void on_collision_exited(CollisionObjectPhysX *p_otherObject);

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	void set_event_callback(Type p_callbackObjectType, ObjectID p_id, const StringName &p_method);
	bool has_event_callback(Type p_callbackObjectType);

protected:
	virtual void setup_shape(physx::PxShape *p_shape);

private:
	void create_rigid_body_internal();
	void destroy_rigid_body_internal();

	void set_monitoring(bool p_monitoring);
};

#endif
