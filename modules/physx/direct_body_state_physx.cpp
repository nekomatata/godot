/*************************************************************************/
/*  direct_body_state_physx.cpp                                          */
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

#include "direct_body_state_physx.h"

#include "direct_space_state_physx.h"
#include "physx_types_converter.h"
#include "rigid_body_physx.h"
#include "space_physx.h"

#include "PxRigidDynamic.h"

using namespace physx;

PhysXPhysicsDirectBodyState3D *PhysXPhysicsDirectBodyState3D::singleton = nullptr;

Vector3 PhysXPhysicsDirectBodyState3D::get_total_gravity() const {
	Vector3 total_gravity;
	PX_TO_G(body->px_total_gravity, total_gravity);
	return total_gravity;
}

float PhysXPhysicsDirectBodyState3D::get_total_angular_damp() const {
	if (!body->px_rigid_dynamic) {
		return 0.0;
	}

	return body->px_rigid_dynamic->getAngularDamping();
}

float PhysXPhysicsDirectBodyState3D::get_total_linear_damp() const {
	if (!body->px_rigid_dynamic) {
		return 0.0;
	}

	return body->px_rigid_dynamic->getLinearDamping();
}

Vector3 PhysXPhysicsDirectBodyState3D::get_center_of_mass() const {
	if (!body->px_rigid_dynamic) {
		return Vector3();
	}

	Vector3 center_of_mass;
	PX_TO_G(body->px_rigid_dynamic->getCMassLocalPose().p, center_of_mass);
	return center_of_mass;
}

Basis PhysXPhysicsDirectBodyState3D::get_principal_inertia_axes() const {
	if (!body->px_rigid_dynamic) {
		return Basis();
	}

	Basis body_basis = body->get_transform().basis;

	Vector3 local_inertia;
	PX_TO_G(body->px_rigid_dynamic->getMassSpaceInertiaTensor(), local_inertia);

	Basis principal_inertia_axes = local_inertia.to_diagonal_matrix().transposed();
	principal_inertia_axes = body_basis * principal_inertia_axes;
	return principal_inertia_axes;
}

float PhysXPhysicsDirectBodyState3D::get_inverse_mass() const {
	if (!body->px_rigid_dynamic) {
		return 0.0;
	}

	return body->px_rigid_dynamic->getInvMass();
}

Vector3 PhysXPhysicsDirectBodyState3D::get_inverse_inertia() const {
	if (!body->px_rigid_dynamic) {
		return Vector3();
	}

	Vector3 inverse_inertia;
	PX_TO_G(body->px_rigid_dynamic->getMassSpaceInvInertiaTensor(), inverse_inertia);
	return inverse_inertia;
}

Basis PhysXPhysicsDirectBodyState3D::get_inverse_inertia_tensor() const {
	if (!body->px_rigid_dynamic) {
		return Basis();
	}

	Vector3 local_inertia;
	PX_TO_G(body->px_rigid_dynamic->getMassSpaceInertiaTensor(), local_inertia);

	if (local_inertia.is_equal_approx(Vector3())) {
		return Basis();
	}

	Basis body_basis = body->get_transform().basis;

	Basis inverse_inertia_tensor = local_inertia.to_diagonal_matrix();
	inverse_inertia_tensor = body_basis * inverse_inertia_tensor * body_basis.transposed();
	inverse_inertia_tensor = inverse_inertia_tensor.inverse();

	return inverse_inertia_tensor;
}

void PhysXPhysicsDirectBodyState3D::set_linear_velocity(const Vector3 &p_velocity) {
	body->set_linear_velocity(p_velocity);
}

Vector3 PhysXPhysicsDirectBodyState3D::get_linear_velocity() const {
	return body->get_linear_velocity();
}

void PhysXPhysicsDirectBodyState3D::set_angular_velocity(const Vector3 &p_velocity) {
	body->set_angular_velocity(p_velocity);
}

Vector3 PhysXPhysicsDirectBodyState3D::get_angular_velocity() const {
	return body->get_angular_velocity();
}

void PhysXPhysicsDirectBodyState3D::set_transform(const Transform &p_transform) {
	body->set_transform(p_transform);
}

Transform PhysXPhysicsDirectBodyState3D::get_transform() const {
	return body->get_transform();
}

void PhysXPhysicsDirectBodyState3D::add_central_force(const Vector3 &p_force) {
	body->apply_central_force(p_force);
}

void PhysXPhysicsDirectBodyState3D::add_force(const Vector3 &p_force, const Vector3 &p_pos) {
	body->apply_force(p_force, p_pos);
}

void PhysXPhysicsDirectBodyState3D::add_torque(const Vector3 &p_torque) {
	body->apply_torque(p_torque);
}

void PhysXPhysicsDirectBodyState3D::apply_central_impulse(const Vector3 &p_impulse) {
	body->apply_central_impulse(p_impulse);
}

void PhysXPhysicsDirectBodyState3D::apply_impulse(const Vector3 &p_pos, const Vector3 &p_impulse) {
	body->apply_impulse(p_pos, p_impulse);
}

void PhysXPhysicsDirectBodyState3D::apply_torque_impulse(const Vector3 &p_impulse) {
	body->apply_torque_impulse(p_impulse);
}

void PhysXPhysicsDirectBodyState3D::set_sleep_state(bool p_sleep) {
	body->set_sleeping(p_sleep);
}

bool PhysXPhysicsDirectBodyState3D::is_sleeping() const {
	return body->is_sleeping();
}

int PhysXPhysicsDirectBodyState3D::get_contact_count() const {
	ERR_FAIL_V_MSG(0, "Rigid body contact information is not implemented in PhysX.");
}

Vector3 PhysXPhysicsDirectBodyState3D::get_contact_local_position(int p_contact_idx) const {
	ERR_FAIL_V_MSG(Vector3(), "Rigid body contact information is not implemented in PhysX.");
}

Vector3 PhysXPhysicsDirectBodyState3D::get_contact_local_normal(int p_contact_idx) const {
	ERR_FAIL_V_MSG(Vector3(), "Rigid body contact information is not implemented in PhysX.");
}

float PhysXPhysicsDirectBodyState3D::get_contact_impulse(int p_contact_idx) const {
	ERR_FAIL_V_MSG(0.0, "Rigid body contact information is not implemented in PhysX.");
}

int PhysXPhysicsDirectBodyState3D::get_contact_local_shape(int p_contact_idx) const {
	ERR_FAIL_V_MSG(0, "Rigid body contact information is not implemented in PhysX.");
}

RID PhysXPhysicsDirectBodyState3D::get_contact_collider(int p_contact_idx) const {
	ERR_FAIL_V_MSG(RID(), "Rigid body contact information is not implemented in PhysX.");
}

Vector3 PhysXPhysicsDirectBodyState3D::get_contact_collider_position(int p_contact_idx) const {
	ERR_FAIL_V_MSG(Vector3(), "Rigid body contact information is not implemented in PhysX.");
}

ObjectID PhysXPhysicsDirectBodyState3D::get_contact_collider_id(int p_contact_idx) const {
	ERR_FAIL_V_MSG(ObjectID(), "Rigid body contact information is not implemented in PhysX.");
}

int PhysXPhysicsDirectBodyState3D::get_contact_collider_shape(int p_contact_idx) const {
	ERR_FAIL_V_MSG(0, "Rigid body contact information is not implemented in PhysX.");
}

Vector3 PhysXPhysicsDirectBodyState3D::get_contact_collider_velocity_at_position(int p_contact_idx) const {
	ERR_FAIL_V_MSG(Vector3(), "Rigid body contact information is not implemented in PhysX.");
}

PhysicsDirectSpaceState3D *PhysXPhysicsDirectBodyState3D::get_space_state() {
	ERR_FAIL_COND_V(!body->space, nullptr);
	return body->space->get_direct_state();
}
