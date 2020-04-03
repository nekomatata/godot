/*************************************************************************/
/*  direct_body_state_physx.h                                            */
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

#ifndef DIRECT_BODY_STATE_PHYSX_H
#define DIRECT_BODY_STATE_PHYSX_H

#include "servers/physics_server_3d.h"

class RigidBodyPhysX;

class PhysXPhysicsDirectBodyState3D : public PhysicsDirectBodyState3D {
	GDCLASS(PhysXPhysicsDirectBodyState3D, PhysicsDirectBodyState3D);

	static PhysXPhysicsDirectBodyState3D *singleton;

public:
	static void init_singleton() {
		if (!singleton) {
			singleton = memnew(PhysXPhysicsDirectBodyState3D);
		}
	}

	static void destroy_singleton() {
		memdelete(singleton);
		singleton = nullptr;
	}

	static void singleton_set_delta_time(real_t p_delta_time) {
		singleton->delta_time = p_delta_time;
	}

	static PhysXPhysicsDirectBodyState3D *get_singleton(RigidBodyPhysX *p_body) {
		singleton->body = p_body;
		return singleton;
	}

public:
	RigidBodyPhysX *body;
	real_t delta_time;

private:
	PhysXPhysicsDirectBodyState3D() {
	}

public:
	virtual Vector3 get_total_gravity() const;
	virtual float get_total_angular_damp() const;
	virtual float get_total_linear_damp() const;

	virtual Vector3 get_center_of_mass() const;
	virtual Basis get_principal_inertia_axes() const;
	virtual float get_inverse_mass() const;
	virtual Vector3 get_inverse_inertia() const;
	virtual Basis get_inverse_inertia_tensor() const;

	virtual void set_linear_velocity(const Vector3 &p_velocity);
	virtual Vector3 get_linear_velocity() const;

	virtual void set_angular_velocity(const Vector3 &p_velocity);
	virtual Vector3 get_angular_velocity() const;

	virtual void set_transform(const Transform &p_transform);
	virtual Transform get_transform() const;

	virtual void add_central_force(const Vector3 &p_force);
	virtual void add_force(const Vector3 &p_force, const Vector3 &p_pos);
	virtual void add_torque(const Vector3 &p_torque);
	virtual void apply_central_impulse(const Vector3 &p_impulse);
	virtual void apply_impulse(const Vector3 &p_pos, const Vector3 &p_impulse);
	virtual void apply_torque_impulse(const Vector3 &p_impulse);

	virtual void set_sleep_state(bool p_sleep);
	virtual bool is_sleeping() const;

	virtual int get_contact_count() const;

	virtual Vector3 get_contact_local_position(int p_contact_idx) const;
	virtual Vector3 get_contact_local_normal(int p_contact_idx) const;
	virtual float get_contact_impulse(int p_contact_idx) const;
	virtual int get_contact_local_shape(int p_contact_idx) const;

	virtual RID get_contact_collider(int p_contact_idx) const;
	virtual Vector3 get_contact_collider_position(int p_contact_idx) const;
	virtual ObjectID get_contact_collider_id(int p_contact_idx) const;
	virtual int get_contact_collider_shape(int p_contact_idx) const;
	virtual Vector3 get_contact_collider_velocity_at_position(int p_contact_idx) const;

	virtual real_t get_step() const { return delta_time; }
	virtual void integrate_forces() {
		// Skip the execution of this function
	}

	virtual PhysicsDirectSpaceState3D *get_space_state();
};
#endif
