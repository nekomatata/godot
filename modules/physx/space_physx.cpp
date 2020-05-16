/*************************************************************************/
/*  space_physx.cpp                                                      */
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

#include "space_physx.h"

#include "area_physx.h"
#include "collision_object_physx.h"
#include "direct_space_state_physx.h"
#include "physx_collision_filter.h"
#include "physx_types_converter.h"
#include "rigid_body_physx.h"

#include "PxActor.h"
#include "PxPhysics.h"
#include "PxRigidActor.h"
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "extensions/PxDefaultCpuDispatcher.h"

using namespace physx;

SpacePhysX::SpacePhysX() {
	create_scene_internal();
	direct_access = memnew(PhysXPhysicsDirectSpaceState3D(this));
}

SpacePhysX::~SpacePhysX() {
	memdelete(direct_access);

	destroy_scene_internal();
}

void SpacePhysX::step(float p_delta_time) {
	delta_time = p_delta_time;

	PxActorTypeFlags actor_flags = PxActorTypeFlag::eRIGID_DYNAMIC;

	int dynamic_actor_count = px_scene->getNbActors(actor_flags);
	if (actor_buffer.size() < dynamic_actor_count) {
		actor_buffer.resize(dynamic_actor_count);
	}

	px_scene->getActors(actor_flags, actor_buffer.ptrw(), dynamic_actor_count);
	for (int i = 0; i < dynamic_actor_count; ++i) {
		static_cast<CollisionObjectPhysX *>(actor_buffer[i]->userData)->on_pre_simulation();
	}

	px_scene->collide(p_delta_time);
	ERR_FAIL_COND_MSG(!px_scene->fetchCollision(true), "Failed to fetch collision from the simulation.");
	px_scene->advance();

	PxU32 error_code;
	ERR_FAIL_COND_MSG(!px_scene->fetchResults(true, &error_code), "Failed to fetch results from the simulation.");
	ERR_FAIL_COND_MSG(error_code, "Failed to fetch results from the simulation, error " + itos(error_code) + ".");
}

void SpacePhysX::set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY: {
			gravity_magnitude = p_value;
			update_gravity_internal();
		} break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR: {
			gravity_direction = p_value;
			update_gravity_internal();
		} break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP:
			linear_damping = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP: {
			angular_damping = p_value;
		} break;
		case PhysicsServer3D::AREA_PARAM_PRIORITY:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
		default: {
			WARN_PRINT("Space area parameter " + itos(p_param) + " is not implemented in PhysX.");
		} break;
	}
}

Variant SpacePhysX::get_param(PhysicsServer3D::AreaParameter p_param) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY:
			return gravity_magnitude;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
			return gravity_direction;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP:
			return linear_damping;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP:
			return angular_damping;
		case PhysicsServer3D::AREA_PARAM_PRIORITY:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
		default: {
			WARN_PRINT("Space area parameter " + itos(p_param) + " is not implemented in PhysX.");
			return Variant();
		}
	}
}

void SpacePhysX::set_param(PhysicsServer3D::SpaceParameter p_param, float p_value) {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer3D::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer3D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default: {
			WARN_PRINT("Space parameter " + itos(p_param) + " is not implemented in PhysX.");
		} break;
	}
}

float SpacePhysX::get_param(PhysicsServer3D::SpaceParameter p_param) {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer3D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer3D::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer3D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default: {
			WARN_PRINT("Space parameter " + itos(p_param) + " is not implemented in PhysX.");
			return 0.f;
		}
	}
}

void SpacePhysX::add_area(AreaPhysX *p_area) {
	PxActor *actor = p_area->get_px_actor();
	ERR_FAIL_COND_MSG(!actor, "Invalid Area actor.");
	px_scene->addActor(*actor);
}

void SpacePhysX::remove_area(AreaPhysX *p_area) {
	PxActor *actor = p_area->get_px_actor();
	ERR_FAIL_COND_MSG(!actor, "Invalid Area actor.");
	px_scene->removeActor(*actor);
}

void SpacePhysX::add_rigid_body(RigidBodyPhysX *p_body) {
	PxActor *actor = p_body->get_px_actor();
	ERR_FAIL_COND_MSG(!actor, "Invalid Rigid Body actor.");
	px_scene->addActor(*actor);
	p_body->spOv_mark_for_update();
}

void SpacePhysX::remove_rigid_body(RigidBodyPhysX *p_body) {
	PxActor *actor = p_body->get_px_actor();
	ERR_FAIL_COND_MSG(!actor, "Invalid Rigid Body actor.");
	px_scene->removeActor(*actor);
}

void SpacePhysX::add_soft_body(SoftBodyPhysX *p_body) {
	ERR_FAIL_MSG("Soft body is not implemented in PhysX.");
}

void SpacePhysX::remove_soft_body(SoftBodyPhysX *p_body) {
	ERR_FAIL_MSG("Soft body is not implemented in PhysX.");
}

void SpacePhysX::add_joint(JointPhysX *p_joint, bool p_disable_collision) {
	ERR_FAIL_MSG("Joint is not implemented in PhysX.");
}

void SpacePhysX::remove_joint(JointPhysX *p_joint) {
	ERR_FAIL_MSG("Joint is not implemented in PhysX.");
}

void SpacePhysX::create_scene_internal() {
	PxPhysics &physics = PxGetPhysics();

	PxDefaultCpuDispatcher *cpu_dispatcher = PxDefaultCpuDispatcherCreate(1);
	ERR_FAIL_COND_MSG(!cpu_dispatcher, "Failed to create default cpu dispatcher.");

	PxSceneDesc scene_desc(physics.getTolerancesScale());
	scene_desc.cpuDispatcher = cpu_dispatcher;
	scene_desc.filterShader = filter_shader;

	// TODO: setup collision filter
	// TODO: setup contact callback
	// TODO: setup trigger callback

	px_scene = physics.createScene(scene_desc);
	ERR_FAIL_COND_MSG(!px_scene, "Failed to create physics scene.");

	update_gravity_internal();
}

void SpacePhysX::destroy_scene_internal() {
	PxActorTypeFlags actor_flags = PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC;

	int actor_count = px_scene->getNbActors(actor_flags);
	if (actor_buffer.size() < actor_count) {
		actor_buffer.resize(actor_count);
	}
	px_scene->getActors(actor_flags, actor_buffer.ptrw(), actor_count);
	for (int i = 0; i < actor_count; ++i) {
		static_cast<CollisionObjectPhysX *>(actor_buffer[i]->userData)->set_space(nullptr);
	}

	PxDefaultCpuDispatcher *cpu_dispatcher = static_cast<PxDefaultCpuDispatcher *>(px_scene->getCpuDispatcher());
	px_scene->release();
	px_scene = nullptr;
	cpu_dispatcher->release();
}

void SpacePhysX::update_gravity_internal() {
	PxVec3 px_gravity;
	G_TO_PX(gravity_direction * gravity_magnitude, px_gravity);

	px_scene->setGravity(px_gravity);
}
