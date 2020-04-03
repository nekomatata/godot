/*************************************************************************/
/*  rigid_body_physx.cpp                                                 */
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

#include "rigid_body_physx.h"

#include "area_physx.h"
#include "direct_body_state_physx.h"
#include "physx_types_converter.h"
#include "space_physx.h"

#include "PxForceMode.h"
#include "PxPhysics.h"
#include "PxRigidBody.h"
#include "PxRigidDynamic.h"
#include "PxRigidStatic.h"
#include "common/PxTolerancesScale.h"
#include "extensions/PxRigidBodyExt.h"

using namespace physx;

RigidBodyPhysX::RigidBodyPhysX() :
		RigidCollisionObjectPhysX(CollisionObjectPhysX::TYPE_RIGID_BODY),
		mode(PhysicsServer3D::BODY_MODE_RIGID),
		px_rigid_dynamic(nullptr),
		locked_axis(0),
		mass(1.0),
		gravity_scale(1.0),
		px_total_gravity(0.0),
		linear_damping(0.0),
		angular_damping(0.0),
		can_sleep(true),
		omit_force_integration(false),
		spOv_gravity_point_count(0),
		spOv_need_update(false),
		force_integration_callback(nullptr) {
}

RigidBodyPhysX::~RigidBodyPhysX() {
	if (force_integration_callback) {
		memdelete(force_integration_callback);
	}

	ERR_FAIL_COND(space);
}

void RigidBodyPhysX::set_space(SpacePhysX *p_space) {
	if (space == p_space) {
		return;
	}

	// Clear the old space if there is one
	if (space) {
		// Remove this object form the physics world
		space->remove_rigid_body(this);
	}

	space = p_space;

	if (space) {
		if (!px_actor) {
			create_rigid_body_internal();
		}

		space->add_rigid_body(this);
	} else {
		destroy_rigid_body_internal();
		release_shapes();
	}
}

void RigidBodyPhysX::on_pre_simulation() {
	if (is_sleeping()) {
		return;
	}

	update_transform_internal();

	if (force_integration_callback) {
		if (omit_force_integration) {
			ERR_PRINT("Omit Force Integration is not implemented in PhysX.");
		}

		Object *obj = ObjectDB::get_instance(force_integration_callback->id);
		if (!obj) {
			// Remove integration callback
			set_force_integration_callback(ObjectID(), StringName());
		} else {
			PhysXPhysicsDirectBodyState3D *direct_body_state = PhysXPhysicsDirectBodyState3D::get_singleton(this);
			Variant variant_direct_body_state = direct_body_state;
			const Variant *vp[2] = { &variant_direct_body_state, &force_integration_callback->udata };

			Callable::CallError call_error;
			int argc = (force_integration_callback->udata.get_type() == Variant::NIL) ? 1 : 2;
			obj->call(force_integration_callback->method, vp, argc, call_error);
		}
	}

	if (spOv_need_update || (0 < spOv_gravity_point_count)) {
		spOv_need_update = false;
		spOv_update();
	}

	apply_gravity_internal();

	applied_force.zero();
	applied_torque.zero();
}

void RigidBodyPhysX::apply_gravity_internal() {
	if (!px_rigid_dynamic) {
		return;
	}

	if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
		return;
	}

	px_rigid_dynamic->addForce(px_total_gravity * mass, PxForceMode::eACCELERATION, false);
}

void RigidBodyPhysX::set_force_integration_callback(ObjectID p_id, const StringName &p_method, const Variant &p_udata) {
	if (force_integration_callback) {
		memdelete(force_integration_callback);
		force_integration_callback = nullptr;
	}

	if (p_id.is_valid()) {
		force_integration_callback = memnew(ForceIntegrationCallback);
		force_integration_callback->id = p_id;
		force_integration_callback->method = p_method;
		force_integration_callback->udata = p_udata;
	}
}

void RigidBodyPhysX::set_sleeping(bool p_sleeping) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
		return;
	}

	if (!p_sleeping) {
		px_rigid_dynamic->wakeUp();
	} else if (can_sleep) {
		px_rigid_dynamic->putToSleep();
	}
}

bool RigidBodyPhysX::is_sleeping() const {
	if (!px_rigid_dynamic) {
		return false;
	}

	return px_rigid_dynamic->isSleeping();
}

void RigidBodyPhysX::set_omit_force_integration(bool p_omit) {
	omit_force_integration = p_omit;
}

void RigidBodyPhysX::set_param(PhysicsServer3D::BodyParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE: {
			set_restitution(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_FRICTION: {
			set_friction(p_value);
		} break;
		case PhysicsServer3D::BODY_PARAM_MASS: {
			ERR_FAIL_COND(p_value < 0);
			mass = p_value;
			update_mass_internal();
		} break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP: {
			linear_damping = p_value;
			// The final linear damping will be is set in spOv_update.
			spOv_mark_for_update();
		} break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP: {
			angular_damping = p_value;
			// The final linear damping will be is set in spOv_update.
			spOv_mark_for_update();
		} break;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE: {
			gravity_scale = p_value;
			// The final gravity will be is set in spOv_update.
			spOv_mark_for_update();
		} break;
		default:
			WARN_PRINT("Parameter " + itos(p_param) + " not supported by PhysX. Value: " + itos(p_value));
	}
}

real_t RigidBodyPhysX::get_param(PhysicsServer3D::BodyParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_BOUNCE:
			return get_restitution();
		case PhysicsServer3D::BODY_PARAM_FRICTION:
			return get_friction();
		case PhysicsServer3D::BODY_PARAM_MASS:
			return mass;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP:
			return linear_damping;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP:
			return angular_damping;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE:
			return gravity_scale;
		default:
			WARN_PRINT("Parameter " + itos(p_param) + " not supported by PhysX.");
			return 0;
	}
}

void RigidBodyPhysX::set_mode(PhysicsServer3D::BodyMode p_mode) {
	if (p_mode == mode) {
		return;
	}

	PhysicsServer3D::BodyMode prev_mode = mode;
	mode = p_mode;

	// Nothing else to do if the rigid body is not simulated
	if (!space) {
		return;
	}

	if ((prev_mode == PhysicsServer3D::BODY_MODE_STATIC) || (mode == PhysicsServer3D::BODY_MODE_STATIC)) {
		// Switch between static and dynamic, recreate the whole rigid body
		space->remove_rigid_body(this);
		destroy_rigid_body_internal();
		create_rigid_body_internal();
		space->add_rigid_body(this);
	} else {
		// Switch between dynamic modes, just update mass properties
		update_mass_internal();

		ERR_FAIL_COND(!px_rigid_dynamic);

		if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
			px_rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		} else {
			px_rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
		}

		px_rigid_dynamic->setAngularVelocity(PxVec3(0.0));
		px_rigid_dynamic->setLinearVelocity(PxVec3(0.0));
	}
}

PhysicsServer3D::BodyMode RigidBodyPhysX::get_mode() const {
	return mode;
}

void RigidBodyPhysX::set_state(PhysicsServer3D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM:
			set_transform(p_variant);
			break;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY:
			set_linear_velocity(p_variant);
			break;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY:
			set_angular_velocity(p_variant);
			break;
		case PhysicsServer3D::BODY_STATE_SLEEPING:
			set_sleeping(bool(p_variant));
			break;
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP:
			can_sleep = bool(p_variant);
			update_can_sleep_internal();
			if (!can_sleep) {
				set_sleeping(false);
			}
			break;
	}
}

Variant RigidBodyPhysX::get_state(PhysicsServer3D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM:
			return get_transform();
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY:
			return get_linear_velocity();
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY:
			return get_angular_velocity();
		case PhysicsServer3D::BODY_STATE_SLEEPING:
			return is_sleeping();
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP:
			return can_sleep;
		default:
			WARN_PRINT("This state " + itos(p_state) + " is not supported by PhysX.");
			return Variant();
	}
}

void RigidBodyPhysX::apply_central_impulse(const Vector3 &p_impulse) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_impulse.is_equal_approx(Vector3())) {
		return;
	}

	PxVec3 px_impulse;
	G_TO_PX(p_impulse, px_impulse);

	px_rigid_dynamic->addForce(px_impulse, PxForceMode::eIMPULSE);
}

void RigidBodyPhysX::apply_impulse(const Vector3 &p_pos, const Vector3 &p_impulse) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_impulse.is_equal_approx(Vector3())) {
		return;
	}

	PxVec3 px_impulse;
	G_TO_PX(p_impulse, px_impulse);

	px_rigid_dynamic->addForce(px_impulse, PxForceMode::eIMPULSE);

	Vector3 center_of_mass;
	PX_TO_G(px_rigid_dynamic->getCMassLocalPose().p, center_of_mass);

	Vector3 torque_impulse = (p_pos - center_of_mass).cross(p_impulse);

	PxVec3 px_torque_impulse;
	G_TO_PX(torque_impulse, px_torque_impulse);

	px_rigid_dynamic->addTorque(px_torque_impulse, PxForceMode::eIMPULSE);
}

void RigidBodyPhysX::apply_torque_impulse(const Vector3 &p_impulse) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_impulse.is_equal_approx(Vector3())) {
		return;
	}

	PxVec3 px_impulse;
	G_TO_PX(p_impulse, px_impulse);

	px_rigid_dynamic->addTorque(px_impulse, PxForceMode::eIMPULSE);
}

void RigidBodyPhysX::apply_central_force(const Vector3 &p_force) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_force.is_equal_approx(Vector3())) {
		return;
	}

	applied_force += p_force;

	PxVec3 px_force;
	G_TO_PX(p_force, px_force);

	px_rigid_dynamic->addForce(px_force, PxForceMode::eFORCE);
}

void RigidBodyPhysX::apply_force(const Vector3 &p_force, const Vector3 &p_pos) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_force.is_equal_approx(Vector3())) {
		return;
	}

	applied_force += p_force;

	PxVec3 px_force;
	G_TO_PX(p_force, px_force);

	px_rigid_dynamic->addForce(px_force, PxForceMode::eFORCE);

	Vector3 center_of_mass;
	PX_TO_G(px_rigid_dynamic->getCMassLocalPose().p, center_of_mass);

	Vector3 torque = (p_pos - center_of_mass).cross(p_force);
	applied_torque += torque;

	PxVec3 px_torque;
	G_TO_PX(torque, px_torque);

	px_rigid_dynamic->addTorque(px_torque, PxForceMode::eFORCE);
}

void RigidBodyPhysX::apply_torque(const Vector3 &p_torque) {
	if (!px_rigid_dynamic) {
		return;
	}

	if (p_torque.is_equal_approx(Vector3())) {
		return;
	}

	applied_torque += p_torque;

	PxVec3 px_torque;
	G_TO_PX(p_torque, px_torque);

	px_rigid_dynamic->addTorque(px_torque, PxForceMode::eFORCE);
}

void RigidBodyPhysX::set_applied_force(const Vector3 &p_force) {
	if (!px_rigid_dynamic) {
		return;
	}

	applied_force = p_force;

	px_rigid_dynamic->clearForce(PxForceMode::eFORCE);

	if (p_force.is_equal_approx(Vector3())) {
		return;
	}

	PxVec3 px_force;
	G_TO_PX(p_force, px_force);

	px_rigid_dynamic->addForce(px_force, PxForceMode::eFORCE);
}

void RigidBodyPhysX::set_applied_torque(const Vector3 &p_torque) {
	if (!px_rigid_dynamic) {
		return;
	}

	applied_torque = p_torque;

	px_rigid_dynamic->clearTorque(PxForceMode::eFORCE);

	if (p_torque.is_equal_approx(Vector3())) {
		return;
	}

	PxVec3 px_torque;
	G_TO_PX(p_torque, px_torque);

	px_rigid_dynamic->addTorque(px_torque, PxForceMode::eFORCE);
}

void RigidBodyPhysX::set_axis_lock(PhysicsServer3D::BodyAxis p_axis, bool lock) {
	if (lock) {
		locked_axis |= p_axis;
	} else {
		locked_axis &= ~p_axis;
	}

	update_axis_lock_internal();
}

bool RigidBodyPhysX::is_axis_locked(PhysicsServer3D::BodyAxis p_axis) const {
	return (locked_axis & p_axis);
}

void RigidBodyPhysX::set_continuous_collision_detection(bool p_enable) {
	ERR_FAIL_MSG("Continuous Collision Detection is not implemented in PhysX.");
}

bool RigidBodyPhysX::is_continuous_collision_detection_enabled() const {
	ERR_FAIL_V_MSG(false, "Continuous Collision Detection is not implemented in PhysX.");
}

void RigidBodyPhysX::set_linear_velocity(const Vector3 &p_velocity) {
	if (!px_rigid_dynamic) {
		return;
	}

	PxVec3 px_velocity;
	G_TO_PX(p_velocity, px_velocity);

	px_rigid_dynamic->setLinearVelocity(px_velocity);
}

Vector3 RigidBodyPhysX::get_linear_velocity() const {
	if (!px_rigid_dynamic) {
		return Vector3();
	}

	Vector3 godot_velocity;
	PX_TO_G(px_rigid_dynamic->getLinearVelocity(), godot_velocity);

	return godot_velocity;
}

void RigidBodyPhysX::set_angular_velocity(const Vector3 &p_velocity) {
	if (!px_rigid_dynamic) {
		return;
	}

	PxVec3 px_velocity;
	G_TO_PX(p_velocity, px_velocity);

	px_rigid_dynamic->setAngularVelocity(px_velocity);
}

Vector3 RigidBodyPhysX::get_angular_velocity() const {
	if (!px_rigid_dynamic) {
		return Vector3();
	}

	Vector3 godot_velocity;
	PX_TO_G(px_rigid_dynamic->getAngularVelocity(), godot_velocity);

	return godot_velocity;
}

void RigidBodyPhysX::set_px_transform(const PxTransform &p_global_transform) {
	if (px_rigid_dynamic && (mode == PhysicsServer3D::BODY_MODE_KINEMATIC)) {
		px_rigid_dynamic->setKinematicTarget(p_global_transform);
	} else {
		RigidCollisionObjectPhysX::set_px_transform(p_global_transform);
	}
}

void RigidBodyPhysX::update_transform_internal() {
	if (px_rigid_dynamic) {
		if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
			if (px_rigid_dynamic->getKinematicTarget(body_transform)) {
				return;
			}
		}

		body_transform = px_rigid_dynamic->getGlobalPose();
	}
}

struct AreaComparator {
	_FORCE_INLINE_ bool operator()(const AreaPhysX *a, const AreaPhysX *b) const {
		return a->get_spOv_priority() < b->get_spOv_priority();
	}
};

void RigidBodyPhysX::on_enter_area(AreaPhysX *p_area) {
	if (PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED == p_area->get_spOv_mode()) {
		// No need to track area
		return;
	}

	spOv_mark_for_update();

	areas_overlapped.push_back(p_area);
	areas_overlapped.sort_custom<AreaComparator>();

	if (p_area->is_spOv_gravity_point()) {
		++spOv_gravity_point_count;
		ERR_FAIL_COND(spOv_gravity_point_count <= 0);
	}
}

void RigidBodyPhysX::on_exit_area(AreaPhysX *p_area) {
	if (PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED == p_area->get_spOv_mode()) {
		// No need to track area
		return;
	}

	for (int area_index = areas_overlapped.size() - 1; area_index >= 0; --area_index) {
		if (areas_overlapped[area_index] == p_area) {
			areas_overlapped.remove(area_index);

			spOv_mark_for_update();

			if (p_area->is_spOv_gravity_point()) {
				--spOv_gravity_point_count;
				ERR_FAIL_COND(spOv_gravity_point_count < 0);
			}

			return;
		}
	}

	ERR_FAIL_MSG("Can't find overlapping area on rigid body exit event.");
}

void RigidBodyPhysX::spOv_update() {
	if (is_sleeping()) {
		return;
	}

	ERR_FAIL_COND(!px_rigid_dynamic);
	ERR_FAIL_COND(!space);

	Vector3 total_gravity(0.0, 0.0, 0.0);
	real_t total_linear_damping(linear_damping);
	real_t total_angular_damping(angular_damping);

	bool stopped = false;
	for (int area_index = areas_overlapped.size() - 1; (area_index >= 0) && !stopped; --area_index) {
		AreaPhysX *current_area = areas_overlapped[area_index];
		ERR_CONTINUE(!current_area);
		ERR_CONTINUE(PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED == current_area->get_spOv_mode());

		Vector3 support_gravity = Vector3(1.0, 1.0, 1.0);
		if (current_area->is_spOv_gravity_point()) {
			Vector3 support_gravity = current_area->get_transform().xform(current_area->get_spOv_gravity_vector()) - get_transform().get_origin();
			real_t distance_mag = support_gravity.length();
			if (distance_mag == 0) {
				support_gravity = Vector3();
			} else {
				support_gravity /= distance_mag;
			}

			if (current_area->get_spOv_gravity_point_distance_scale() > 0) {
				support_gravity *= current_area->get_spOv_gravity_magnitude() / Math::pow(distance_mag * current_area->get_spOv_gravity_point_distance_scale() + 1, 2);
			} else {
				support_gravity *= current_area->get_spOv_gravity_magnitude();
			}
		} else {
			support_gravity = current_area->get_spOv_gravity_vector() * current_area->get_spOv_gravity_magnitude();
		}

		switch (current_area->get_spOv_mode()) {
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE: {
				total_gravity += support_gravity;
				total_linear_damping += current_area->get_spOv_linear_damping();
				total_angular_damping += current_area->get_spOv_angular_damping();
			} break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
				total_gravity += support_gravity;
				total_linear_damping += current_area->get_spOv_linear_damping();
				total_angular_damping += current_area->get_spOv_angular_damping();
				stopped = true;
			} break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE: {
				total_gravity = support_gravity;
				total_linear_damping = current_area->get_spOv_linear_damping();
				total_angular_damping = current_area->get_spOv_angular_damping();
				stopped = true;
			} break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
				total_gravity = support_gravity;
				total_linear_damping = current_area->get_spOv_linear_damping();
				total_angular_damping = current_area->get_spOv_angular_damping();
			} break;
			case PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED:
				break;
		}
	}

	if (!stopped) {
		total_gravity += space->get_gravity_direction() * space->get_gravity_magnitude();
		total_linear_damping += space->get_linear_damping();
		total_angular_damping += space->get_angular_damping();
	}

	G_TO_PX(total_gravity * gravity_scale, px_total_gravity);

	px_rigid_dynamic->setLinearDamping(total_linear_damping);
	px_rigid_dynamic->setAngularDamping(total_angular_damping);
}

void RigidBodyPhysX::reload_shapes() {
	RigidCollisionObjectPhysX::reload_shapes();

	if (mode == PhysicsServer3D::BODY_MODE_RIGID) {
		update_mass_internal();
	}
}

void RigidBodyPhysX::create_rigid_body_internal() {
	if (mode == PhysicsServer3D::BODY_MODE_STATIC) {
		PxRigidStatic *px_rigid_static = PxGetPhysics().createRigidStatic(body_transform);
		ERR_FAIL_COND_MSG(!px_rigid_static, "Failed to create static rigid body.");

		setup_actor(px_rigid_static);

		reload_shapes();
	} else {
		px_rigid_dynamic = PxGetPhysics().createRigidDynamic(body_transform);
		ERR_FAIL_COND_MSG(!px_rigid_dynamic, "Failed to create dynamic rigid body.");

		setup_actor(px_rigid_dynamic);

		// Disable gravity to apply it ourselves
		px_rigid_dynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);

		if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
			px_rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		}

		// Set queries to use the kinematic target when possible
		px_rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES, true);

		reload_shapes();
		update_axis_lock_internal();
		update_can_sleep_internal();

		spOv_mark_for_update();
	}
}

void RigidBodyPhysX::destroy_rigid_body_internal() {
	destroy_actor();
	px_rigid_dynamic = nullptr;
}

void RigidBodyPhysX::update_mass_internal() {
	if (!px_rigid_dynamic) {
		return;
	}

	switch (mode) {
		case PhysicsServer3D::BODY_MODE_RIGID: {
			// calculate mass, inertia and center of mass
			PxRigidBodyExt::setMassAndUpdateInertia(*px_rigid_dynamic, mass);
		} break;
		case PhysicsServer3D::BODY_MODE_CHARACTER: {
			px_rigid_dynamic->setMass(mass);
			px_rigid_dynamic->setMassSpaceInertiaTensor(PxVec3(0.0)); // infinite inertia
		} break;
		case PhysicsServer3D::BODY_MODE_KINEMATIC: {
			px_rigid_dynamic->setMass(0.0); // infinite mass
			px_rigid_dynamic->setMassSpaceInertiaTensor(PxVec3(0.0)); // infinite inertia
		} break;
		case PhysicsServer3D::BODY_MODE_STATIC:
			break;
	}
}

void RigidBodyPhysX::update_axis_lock_internal() {
	if (!px_rigid_dynamic) {
		return;
	}

	PxRigidDynamicLockFlags lock_flags;
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_LINEAR_X)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_X;
	}
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_LINEAR_Y)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Y;
	}
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_LINEAR_Z)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Z;
	}
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_ANGULAR_X)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_X;
	}
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_ANGULAR_Y)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y;
	}
	if (is_axis_locked(PhysicsServer3D::BODY_AXIS_ANGULAR_Z)) {
		lock_flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z;
	}

	px_rigid_dynamic->setRigidDynamicLockFlags(lock_flags);
}

void RigidBodyPhysX::update_can_sleep_internal() {
	if (!px_rigid_dynamic) {
		return;
	}

	if (can_sleep) {
		// Default sleep threshold from the engine
		float scale_speed = PxGetPhysics().getTolerancesScale().speed;
		px_rigid_dynamic->setSleepThreshold(5e-5f * scale_speed * scale_speed);
	} else {
		px_rigid_dynamic->setSleepThreshold(0.0);
	}
}
