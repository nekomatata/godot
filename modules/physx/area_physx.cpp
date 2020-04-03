/*************************************************************************/
/*  area_physx.cpp                                                       */
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

#include "area_physx.h"

#include "space_physx.h"

#include "PxPhysics.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

using namespace physx;

AreaPhysX::AreaPhysX() :
		RigidCollisionObjectPhysX(CollisionObjectPhysX::TYPE_AREA),
		monitorable(true),
		monitoring(true),
		spOv_mode(PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED),
		spOv_gravity_point(false),
		spOv_gravity_point_distance_scale(0),
		spOv_gravity_point_attenuation(1),
		spOv_gravity_vector(0, -1, 0),
		spOv_gravity_magnitude(10),
		spOv_linear_damping(0.1),
		spOv_angular_damping(1),
		spOv_priority(0) {
}

AreaPhysX::~AreaPhysX() {
}

void AreaPhysX::call_event(CollisionObjectPhysX *p_otherObject, PhysicsServer3D::AreaBodyStatus p_status) {
	int type_index = static_cast<int>(p_otherObject->getType());
	ERR_FAIL_INDEX(type_index, EVENT_CALLBACK_COUNT);

	InOutEventCallback &event_callback = event_callbacks[type_index];
	Object *area_object = ObjectDB::get_instance(event_callback.event_callback_id);
	if (!area_object) {
		event_callback.event_callback_id = ObjectID();
		return;
	}

	Variant call_event_res[5];
	call_event_res[0] = p_status;
	call_event_res[1] = p_otherObject->get_self(); // Other body
	call_event_res[2] = p_otherObject->get_instance_id(); // instance ID
	call_event_res[3] = 0; // other_body_shape ID
	call_event_res[4] = 0; // self_shape ID

	Callable::CallError error;
	area_object->call(event_callback.event_callback_method, (const Variant **)&call_event_res, 5, error);
}

void AreaPhysX::set_space(SpacePhysX *p_space) {
	if (space == p_space) {
		return;
	}

	// Clear the old space if there is one
	if (space) {
		// Remove this object form the physics world
		space->remove_area(this);
	}

	space = p_space;

	if (space) {
		if (!px_actor) {
			create_rigid_body_internal();
		}

		space->add_area(this);
	} else {
		destroy_rigid_body_internal();
		release_shapes();
	}
}

void AreaPhysX::on_collision_entered(CollisionObjectPhysX *p_otherObject) {
	call_event(p_otherObject, PhysicsServer3D::AREA_BODY_ADDED);
	p_otherObject->on_enter_area(this);
}

void AreaPhysX::on_collision_exited(CollisionObjectPhysX *p_otherObject) {
	call_event(p_otherObject, PhysicsServer3D::AREA_BODY_REMOVED);
	p_otherObject->on_exit_area(this);
}

void AreaPhysX::set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			set_spOv_gravity_magnitude(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			set_spOv_gravity_vector(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP: {
			set_spOv_linear_damping(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			set_spOv_angular_damping(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_PRIORITY: {
			set_spOv_priority(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT: {
			set_spOv_gravity_point(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_DISTANCE_SCALE: {
			set_spOv_gravity_point_distance_scale(p_value);
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_ATTENUATION: {
			set_spOv_gravity_point_attenuation(p_value);
		} break;
		default:
			WARN_PRINT("Area doesn't support this parameter in the Bullet backend: " + itos(p_param));
	}
}

Variant AreaPhysX::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY:
			return spOv_gravity_magnitude;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
			return spOv_gravity_vector;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP:
			return spOv_linear_damping;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP:
			return spOv_angular_damping;
		case PhysicsServer3D::AREA_PARAM_PRIORITY:
			return spOv_priority;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT:
			return spOv_gravity_point;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
			return spOv_gravity_point_distance_scale;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
			return spOv_gravity_point_attenuation;
		default:
			WARN_PRINT("Area doesn't support this parameter in the Bullet backend: " + itos(p_param));
			return Variant();
	}
}

void AreaPhysX::set_event_callback(Type p_callbackObjectType, ObjectID p_id, const StringName &p_method) {
	int type_index = static_cast<int>(p_callbackObjectType);
	ERR_FAIL_INDEX(type_index, EVENT_CALLBACK_COUNT);

	InOutEventCallback &ev = event_callbacks[type_index];
	ev.event_callback_id = p_id;
	ev.event_callback_method = p_method;

	bool enable_monitoring = false;
	for (int callback_index = 0; callback_index < EVENT_CALLBACK_COUNT; ++callback_index) {
		if (event_callbacks[callback_index].event_callback_id.is_valid()) {
			enable_monitoring = true;
			break;
		}
	}
	set_monitoring(enable_monitoring);
}

bool AreaPhysX::has_event_callback(Type p_callbackObjectType) {
	int type_index = static_cast<int>(p_callbackObjectType);
	ERR_FAIL_INDEX_V(type_index, EVENT_CALLBACK_COUNT, false);

	return event_callbacks[type_index].event_callback_id.is_valid();
}

void AreaPhysX::set_monitoring(bool p_monitoring) {
	if (p_monitoring == monitoring) {
		return;
	}

	monitoring = p_monitoring;

	if (monitoring) {
		// TODO: enable trigger shapes
	} else {
		// TODO: disable trigger shapes
	}
}

void AreaPhysX::set_monitorable(bool p_monitorable) {
	if (p_monitorable == monitorable) {
		return;
	}

	monitorable = p_monitorable;

	if (monitoring) {
		// TODO: enable collision shapes
	} else {
		// TODO: disable collision shapes
	}
}

void AreaPhysX::setup_shape(PxShape *p_shape) {
	RigidCollisionObjectPhysX::setup_shape(p_shape);

	// setup trigger shape
	p_shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	p_shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);

	// TODO: setup collision shape
}

void AreaPhysX::create_rigid_body_internal() {
	PxRigidDynamic *px_rigid_dynamic = PxGetPhysics().createRigidDynamic(body_transform);
	ERR_FAIL_COND_MSG(!px_rigid_dynamic, "Failed to create area rigid body.");

	setup_actor(px_rigid_dynamic);

	px_rigid_dynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	px_rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	reload_shapes();

	// TODO: create trigger and collision shapes
}

void AreaPhysX::destroy_rigid_body_internal() {
	destroy_actor();
}
