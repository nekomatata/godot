/*************************************************************************/
/*  physics_server_physx.cpp                                             */
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

#include "physics_server_physx.h"

#include "area_physx.h"
#include "direct_body_state_physx.h"
#include "direct_space_state_physx.h"
#include "joint_physx.h"
#include "rigid_body_physx.h"
#include "shape_physx.h"
#include "space_physx.h"

#include "PxFoundation.h"
#include "PxPhysics.h"
#include "PxPhysicsVersion.h"
#include "common/PxTolerancesScale.h"
#include "cooking/PxCooking.h"
#include "extensions/PxExtensionsAPI.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxErrorCallback.h"

#ifdef DEBUG_ENABLED
#include "pvd/PxPvd.h"
#include "pvd/PxPvdTransport.h"
#endif

using namespace physx;

template <class T>
RID PhysXPhysicsServer3D::create_rid(RID_PtrOwner<T> &p_owner, T *p_rid_object) {
	RID rid = p_owner.make_rid(p_rid_object);
	p_rid_object->set_self(rid);
	p_rid_object->set_physics_server(this);
	return rid;
}

class PhysXAllocatorCallback : public PxAllocatorCallback {
public:
	virtual ~PhysXAllocatorCallback() {}

	virtual void *allocate(size_t size, const char *typeName, const char *filename, int line) {
		return Memory::alloc_static(size, true);
	}

	virtual void deallocate(void *ptr) {
		Memory::free_static(ptr, true);
	}
};

class PhysXErrorCallback : public PxErrorCallback {
public:
	virtual ~PhysXErrorCallback() {}

	virtual void reportError(PxErrorCode::Enum code, const char *message, const char *file, int line) {
#ifdef DEBUG_ENABLED
		String formatted_message = " in " + String(file) + ", line " + itos(line) + ": " + String(message);
		switch (code) {
			case PxErrorCode::eDEBUG_INFO: {
				print_line("PhysX Info" + formatted_message);
			} break;
			case PxErrorCode::eDEBUG_WARNING: {
				WARN_PRINT("PhysX Warning" + formatted_message);
			} break;
			case PxErrorCode::ePERF_WARNING: {
				WARN_PRINT("PhysX Performance Warning" + formatted_message);
			} break;
			case PxErrorCode::eINVALID_PARAMETER: {
				ERR_PRINT("PhysX Error (Invalid Parameter)" + formatted_message);
			} break;
			case PxErrorCode::eINVALID_OPERATION: {
				ERR_PRINT("PhysX Error (Invalid Operation)" + formatted_message);
			} break;
			case PxErrorCode::eOUT_OF_MEMORY: {
				ERR_PRINT("PhysX Error (Out of Memory)" + formatted_message);
			} break;
			case PxErrorCode::eINTERNAL_ERROR: {
				ERR_PRINT("PhysX Error (Internal Error)" + formatted_message);
			} break;
			case PxErrorCode::eABORT: {
				ERR_PRINT("PhysX Error (Abort)" + formatted_message);
			} break;
			default: {
				ERR_PRINT("PhysX Error Code " + itos(code) + formatted_message);
			} break;
		}
#endif
	}
};

PhysXPhysicsServer3D::PhysXPhysicsServer3D() :
		allocator_callback(nullptr),
		error_callback(nullptr),
		px_foundation(nullptr),
		px_physics(nullptr),
		px_cooking(nullptr),
		px_debugger(nullptr),
		active(true) {
	// Create PhysX callback handlers
	allocator_callback = memnew(PhysXAllocatorCallback);
	error_callback = memnew(PhysXErrorCallback);

	// Create foundation
	px_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *allocator_callback, *error_callback);
	ERR_FAIL_COND_MSG(!px_foundation, "Failed to create PhysX Foundation.");

#ifdef DEBUG_ENABLED
	// Create debugger
	px_debugger = PxCreatePvd(*px_foundation);
	ERR_FAIL_COND_MSG(!px_debugger, "Failed to create PhysX Debugger.");

	PxPvdTransport *transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	if (!px_debugger->connect(*transport, PxPvdInstrumentationFlag::eALL)) {
		WARN_PRINT("Failed to connect to PhysX Debugger.");
	}
#endif

	// Create physics system
	PxTolerancesScale tolerance_scale;
	px_physics = PxCreateBasePhysics(PX_PHYSICS_VERSION, *px_foundation, tolerance_scale, true, px_debugger);
	ERR_FAIL_COND_MSG(!px_physics, "Failed to create PhysX System.");

	// Init components
	//PxRegisterArticulations(*px_physics);
	PxRegisterHeightFields(*px_physics);

	// Init extensions
	if (!PxInitExtensions(*px_physics, px_debugger)) {
		ERR_FAIL_MSG("Failed to initialize PhysX Extensions.");
	}

	// Create cooking
	PxCookingParams cooking_params(tolerance_scale);
	px_cooking = PxCreateCooking(PX_PHYSICS_VERSION, *px_foundation, cooking_params);
	ERR_FAIL_COND_MSG(!px_cooking, "Failed to create PhysX Cooking.");
}

PhysXPhysicsServer3D::~PhysXPhysicsServer3D() {
	if (px_cooking) {
		px_cooking->release();
		px_cooking = nullptr;
	}

	if (px_physics) {
		PxCloseExtensions();
		px_physics->release();
		px_physics = nullptr;
	}

#ifdef DEBUG_ENABLED
	if (px_debugger) {
		px_debugger->disconnect();
		PxPvdTransport *pvd_transport = px_debugger->getTransport();
		px_debugger->release();
		pvd_transport->release();
		px_debugger = nullptr;
	}
#endif

	if (px_foundation) {
		px_foundation->release();
		px_foundation = nullptr;
	}

	memdelete(allocator_callback);
	allocator_callback = nullptr;

	memdelete(error_callback);
	error_callback = nullptr;
}

RID PhysXPhysicsServer3D::shape_create(ShapeType p_shape) {
	ShapePhysX *shape = nullptr;

	switch (p_shape) {
		case SHAPE_PLANE: {
			shape = memnew(PlaneShapePhysX);
		} break;
		case SHAPE_SPHERE: {
			shape = memnew(SphereShapePhysX);
		} break;
		case SHAPE_BOX: {
			shape = memnew(BoxShapePhysX);
		} break;
		case SHAPE_CAPSULE: {
			shape = memnew(CapsuleShapePhysX);
		} break;
		case SHAPE_CYLINDER: {
			shape = memnew(CylinderShapePhysX);
		} break;
		case SHAPE_CONVEX_POLYGON: {
			shape = memnew(ConvexPolygonShapePhysX);
		} break;
		case SHAPE_CONCAVE_POLYGON: {
			shape = memnew(ConcavePolygonShapePhysX);
		} break;
		case SHAPE_HEIGHTMAP: {
			shape = memnew(HeightMapShapePhysX);
		} break;
		case SHAPE_RAY: {
			shape = memnew(RayShapePhysX);
		} break;
		case SHAPE_CUSTOM:
		default:
			ERR_FAIL_V(RID());
			break;
	}

	return create_rid(shape_owner, shape);
}

void PhysXPhysicsServer3D::shape_set_data(RID p_shape, const Variant &p_data) {
	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);
	shape->set_data(*px_cooking, p_data);
}

void PhysXPhysicsServer3D::shape_set_custom_solver_bias(RID p_shape, real_t p_bias) {
	//WARN_PRINT("Bias not supported by Bullet physics engine");
}

PhysicsServer3D::ShapeType PhysXPhysicsServer3D::shape_get_type(RID p_shape) const {
	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND_V(!shape, PhysicsServer3D::SHAPE_CUSTOM);
	return shape->get_type();
}

Variant PhysXPhysicsServer3D::shape_get_data(RID p_shape) const {
	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND_V(!shape, Variant());
	return shape->get_data();
}

void PhysXPhysicsServer3D::shape_set_margin(RID p_shape, real_t p_margin) {
	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);
	shape->set_margin(p_margin);
}

real_t PhysXPhysicsServer3D::shape_get_margin(RID p_shape) const {
	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND_V(!shape, 0.0);
	return shape->get_margin();
}

real_t PhysXPhysicsServer3D::shape_get_custom_solver_bias(RID p_shape) const {
	//WARN_PRINT("Bias not supported by Bullet physics engine");
	return 0.;
}

RID PhysXPhysicsServer3D::space_create() {
	SpacePhysX *space = memnew(SpacePhysX);

	return create_rid(space_owner, space);
}

void PhysXPhysicsServer3D::space_set_active(RID p_space, bool p_active) {

	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND(!space);

	if (space_is_active(p_space) == p_active) {
		return;
	}

	if (p_active) {
		active_spaces.push_back(space);
	} else {
		active_spaces.erase(space);
	}
}

bool PhysXPhysicsServer3D::space_is_active(RID p_space) const {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND_V(!space, false);

	return -1 != active_spaces.find(space);
}

void PhysXPhysicsServer3D::space_set_param(RID p_space, SpaceParameter p_param, real_t p_value) {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND(!space);
	space->set_param(p_param, p_value);
}

real_t PhysXPhysicsServer3D::space_get_param(RID p_space, SpaceParameter p_param) const {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND_V(!space, 0);
	return space->get_param(p_param);
}

PhysicsDirectSpaceState3D *PhysXPhysicsServer3D::space_get_direct_state(RID p_space) {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND_V(!space, nullptr);

	return space->get_direct_state();
}

void PhysXPhysicsServer3D::space_set_debug_contacts(RID p_space, int p_max_contacts) {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND(!space);

	ERR_PRINT_ONCE("Debug Contacts are not implemented in PhysX.");
	//space->set_debug_contacts(p_max_contacts);
}

Vector<Vector3> PhysXPhysicsServer3D::space_get_contacts(RID p_space) const {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND_V(!space, Vector<Vector3>());

	ERR_PRINT_ONCE("Debug Contacts are not implemented in PhysX.");
	return Vector<Vector3>();
	//return space->get_debug_contacts();
}

int PhysXPhysicsServer3D::space_get_contact_count(RID p_space) const {
	SpacePhysX *space = space_owner.getornull(p_space);
	ERR_FAIL_COND_V(!space, 0);

	ERR_PRINT_ONCE("Debug Contacts are not implemented in PhysX.");
	return 0;
	//return space->get_debug_contact_count();
}

RID PhysXPhysicsServer3D::area_create() {
	AreaPhysX *area = memnew(AreaPhysX);
	area->set_collision_layer(1);
	area->set_collision_mask(1);

	return create_rid(area_owner, area);
}

void PhysXPhysicsServer3D::area_set_space(RID p_area, RID p_space) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	SpacePhysX *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.getornull(p_space);
		ERR_FAIL_COND(!space);
	}
	area->set_space(space);
}

RID PhysXPhysicsServer3D::area_get_space(RID p_area) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	return area->get_space()->get_self();
}

void PhysXPhysicsServer3D::area_set_space_override_mode(RID p_area, AreaSpaceOverrideMode p_mode) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_spOv_mode(p_mode);
}

PhysicsServer3D::AreaSpaceOverrideMode PhysXPhysicsServer3D::area_get_space_override_mode(RID p_area) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED);

	return area->get_spOv_mode();
}

void PhysXPhysicsServer3D::area_add_shape(RID p_area, RID p_shape, const Transform &p_transform, bool p_disabled) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);

	area->add_shape(shape, p_transform, p_disabled);
}

void PhysXPhysicsServer3D::area_set_shape(RID p_area, int p_shape_idx, RID p_shape) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);

	area->set_shape(p_shape_idx, shape);
}

void PhysXPhysicsServer3D::area_set_shape_transform(RID p_area, int p_shape_idx, const Transform &p_transform) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_shape_transform(p_shape_idx, p_transform);
}

int PhysXPhysicsServer3D::area_get_shape_count(RID p_area) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, 0);

	return area->get_shape_count();
}

RID PhysXPhysicsServer3D::area_get_shape(RID p_area, int p_shape_idx) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, RID());

	return area->get_shape(p_shape_idx)->get_self();
}

Transform PhysXPhysicsServer3D::area_get_shape_transform(RID p_area, int p_shape_idx) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, Transform());

	return area->get_shape_transform(p_shape_idx);
}

void PhysXPhysicsServer3D::area_remove_shape(RID p_area, int p_shape_idx) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	return area->remove_shape(p_shape_idx);
}

void PhysXPhysicsServer3D::area_clear_shapes(RID p_area) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	for (int i = area->get_shape_count(); 0 < i; --i)
		area->remove_shape(0);
}

void PhysXPhysicsServer3D::area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_shape_disabled(p_shape_idx, p_disabled);
}

void PhysXPhysicsServer3D::area_attach_object_instance_id(RID p_area, ObjectID p_id) {
	if (space_owner.owns(p_area)) {
		return;
	}
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	area->set_instance_id(p_id);
}

ObjectID PhysXPhysicsServer3D::area_get_object_instance_id(RID p_area) const {
	if (space_owner.owns(p_area)) {
		return ObjectID();
	}
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, ObjectID());
	return area->get_instance_id();
}

void PhysXPhysicsServer3D::area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value) {
	if (space_owner.owns(p_area)) {
		SpacePhysX *space = space_owner.getornull(p_area);
		if (space) {
			space->set_param(p_param, p_value);
		}
	} else {

		AreaPhysX *area = area_owner.getornull(p_area);
		ERR_FAIL_COND(!area);

		area->set_param(p_param, p_value);
	}
}

Variant PhysXPhysicsServer3D::area_get_param(RID p_area, AreaParameter p_param) const {
	if (space_owner.owns(p_area)) {
		SpacePhysX *space = space_owner.getornull(p_area);
		return space->get_param(p_param);
	} else {
		AreaPhysX *area = area_owner.getornull(p_area);
		ERR_FAIL_COND_V(!area, Variant());

		return area->get_param(p_param);
	}
}

void PhysXPhysicsServer3D::area_set_transform(RID p_area, const Transform &p_transform) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	area->set_transform(p_transform);
}

Transform PhysXPhysicsServer3D::area_get_transform(RID p_area) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, Transform());
	return area->get_transform();
}

void PhysXPhysicsServer3D::area_set_collision_mask(RID p_area, uint32_t p_mask) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	area->set_collision_mask(p_mask);
}

void PhysXPhysicsServer3D::area_set_collision_layer(RID p_area, uint32_t p_layer) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	area->set_collision_layer(p_layer);
}

void PhysXPhysicsServer3D::area_set_monitorable(RID p_area, bool p_monitorable) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_monitorable(p_monitorable);
}

void PhysXPhysicsServer3D::area_set_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_event_callback(CollisionObjectPhysX::TYPE_RIGID_BODY, p_receiver ? p_receiver->get_instance_id() : ObjectID(), p_method);
}

void PhysXPhysicsServer3D::area_set_area_monitor_callback(RID p_area, Object *p_receiver, const StringName &p_method) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);

	area->set_event_callback(CollisionObjectPhysX::TYPE_AREA, p_receiver ? p_receiver->get_instance_id() : ObjectID(), p_method);
}

void PhysXPhysicsServer3D::area_set_ray_pickable(RID p_area, bool p_enable) {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND(!area);
	area->set_ray_pickable(p_enable);
}

bool PhysXPhysicsServer3D::area_is_ray_pickable(RID p_area) const {
	AreaPhysX *area = area_owner.getornull(p_area);
	ERR_FAIL_COND_V(!area, false);
	return area->is_ray_pickable();
}

RID PhysXPhysicsServer3D::body_create(BodyMode p_mode, bool p_init_sleeping) {
	RigidBodyPhysX *body = memnew(RigidBodyPhysX);
	body->set_mode(p_mode);
	body->set_collision_layer(1);
	body->set_collision_mask(1);
	if (p_init_sleeping) {
		body->set_state(BODY_STATE_SLEEPING, p_init_sleeping);
	}
	return create_rid(rigid_body_owner, body);
}

void PhysXPhysicsServer3D::body_set_space(RID p_body, RID p_space) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	SpacePhysX *space = nullptr;

	if (p_space.is_valid()) {
		space = space_owner.getornull(p_space);
		ERR_FAIL_COND(!space);
	}

	if (body->get_space() == space)
		return; //pointles

	body->set_space(space);
}

RID PhysXPhysicsServer3D::body_get_space(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, RID());

	SpacePhysX *space = body->get_space();
	if (!space)
		return RID();
	return space->get_self();
}

void PhysXPhysicsServer3D::body_set_mode(RID p_body, PhysicsServer3D::BodyMode p_mode) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_mode(p_mode);
}

PhysicsServer3D::BodyMode PhysXPhysicsServer3D::body_get_mode(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, BODY_MODE_STATIC);
	return body->get_mode();
}

void PhysXPhysicsServer3D::body_add_shape(RID p_body, RID p_shape, const Transform &p_transform, bool p_disabled) {

	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void PhysXPhysicsServer3D::body_set_shape(RID p_body, int p_shape_idx, RID p_shape) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	ShapePhysX *shape = shape_owner.getornull(p_shape);
	ERR_FAIL_COND(!shape);

	body->set_shape(p_shape_idx, shape);
}

void PhysXPhysicsServer3D::body_set_shape_transform(RID p_body, int p_shape_idx, const Transform &p_transform) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int PhysXPhysicsServer3D::body_get_shape_count(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_shape_count();
}

RID PhysXPhysicsServer3D::body_get_shape(RID p_body, int p_shape_idx) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, RID());

	ShapePhysX *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_self();
}

Transform PhysXPhysicsServer3D::body_get_shape_transform(RID p_body, int p_shape_idx) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Transform());
	return body->get_shape_transform(p_shape_idx);
}

void PhysXPhysicsServer3D::body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void PhysXPhysicsServer3D::body_remove_shape(RID p_body, int p_shape_idx) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->remove_shape(p_shape_idx);
}

void PhysXPhysicsServer3D::body_clear_shapes(RID p_body) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->remove_all_shapes();
}

void PhysXPhysicsServer3D::body_attach_object_instance_id(RID p_body, ObjectID p_id) {
	CollisionObjectPhysX *body = get_collision_object(p_body);
	ERR_FAIL_COND(!body);

	body->set_instance_id(p_id);
}

ObjectID PhysXPhysicsServer3D::body_get_object_instance_id(RID p_body) const {
	CollisionObjectPhysX *body = get_collision_object(p_body);
	ERR_FAIL_COND_V(!body, ObjectID());

	return body->get_instance_id();
}

void PhysXPhysicsServer3D::body_set_enable_continuous_collision_detection(RID p_body, bool p_enable) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_continuous_collision_detection(p_enable);
}

bool PhysXPhysicsServer3D::body_is_continuous_collision_detection_enabled(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, false);

	return body->is_continuous_collision_detection_enabled();
}

void PhysXPhysicsServer3D::body_set_collision_layer(RID p_body, uint32_t p_layer) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_collision_layer(p_layer);
}

uint32_t PhysXPhysicsServer3D::body_get_collision_layer(RID p_body) const {
	const RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_layer();
}

void PhysXPhysicsServer3D::body_set_collision_mask(RID p_body, uint32_t p_mask) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_collision_mask(p_mask);
}

uint32_t PhysXPhysicsServer3D::body_get_collision_mask(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_mask();
}

void PhysXPhysicsServer3D::body_set_user_flags(RID p_body, uint32_t p_flags) {
	// This function si not currently supported
}

uint32_t PhysXPhysicsServer3D::body_get_user_flags(RID p_body) const {
	// This function si not currently supported
	return 0;
}

void PhysXPhysicsServer3D::body_set_param(RID p_body, BodyParameter p_param, float p_value) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_param(p_param, p_value);
}

float PhysXPhysicsServer3D::body_get_param(RID p_body, BodyParameter p_param) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_param(p_param);
}

void PhysXPhysicsServer3D::body_set_kinematic_safe_margin(RID p_body, real_t p_margin) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	ERR_FAIL_MSG("Kinematic Safe Margin is not implemented in PhysX.");
}

real_t PhysXPhysicsServer3D::body_get_kinematic_safe_margin(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	ERR_FAIL_V_MSG(0, "Kinematic Safe Margin is not implemented in PhysX.");
}

void PhysXPhysicsServer3D::body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_state(p_state, p_variant);
}

Variant PhysXPhysicsServer3D::body_get_state(RID p_body, BodyState p_state) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Variant());

	return body->get_state(p_state);
}

void PhysXPhysicsServer3D::body_set_applied_force(RID p_body, const Vector3 &p_force) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_applied_force(p_force);
}

Vector3 PhysXPhysicsServer3D::body_get_applied_force(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Vector3());
	return body->get_applied_force();
}

void PhysXPhysicsServer3D::body_set_applied_torque(RID p_body, const Vector3 &p_torque) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_applied_torque(p_torque);
}

Vector3 PhysXPhysicsServer3D::body_get_applied_torque(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Vector3());

	return body->get_applied_torque();
}

void PhysXPhysicsServer3D::body_add_central_force(RID p_body, const Vector3 &p_force) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_central_force(p_force);
}

void PhysXPhysicsServer3D::body_add_force(RID p_body, const Vector3 &p_force, const Vector3 &p_pos) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_force(p_force, p_pos);
}

void PhysXPhysicsServer3D::body_add_torque(RID p_body, const Vector3 &p_torque) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_torque(p_torque);
}

void PhysXPhysicsServer3D::body_apply_central_impulse(RID p_body, const Vector3 &p_impulse) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_central_impulse(p_impulse);
}

void PhysXPhysicsServer3D::body_apply_impulse(RID p_body, const Vector3 &p_pos, const Vector3 &p_impulse) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_impulse(p_pos, p_impulse);
}

void PhysXPhysicsServer3D::body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->apply_torque_impulse(p_impulse);
}

void PhysXPhysicsServer3D::body_set_axis_velocity(RID p_body, const Vector3 &p_axis_velocity) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	Vector3 v = body->get_linear_velocity();
	Vector3 axis = p_axis_velocity.normalized();
	v -= axis * axis.dot(v);
	v += p_axis_velocity;
	body->set_linear_velocity(v);
}

void PhysXPhysicsServer3D::body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_axis_lock(p_axis, p_lock);
}

bool PhysXPhysicsServer3D::body_is_axis_locked(RID p_body, BodyAxis p_axis) const {
	const RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->is_axis_locked(p_axis);
}

void PhysXPhysicsServer3D::body_add_collision_exception(RID p_body, RID p_body_b) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	RigidBodyPhysX *other_body = rigid_body_owner.getornull(p_body_b);
	ERR_FAIL_COND(!other_body);

	body->add_collision_exception(other_body);
}

void PhysXPhysicsServer3D::body_remove_collision_exception(RID p_body, RID p_body_b) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	RigidBodyPhysX *other_body = rigid_body_owner.getornull(p_body_b);
	ERR_FAIL_COND(!other_body);

	body->remove_collision_exception(other_body);
}

void PhysXPhysicsServer3D::body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	for (int i = 0; i < body->get_exceptions().size(); i++) {
		p_exceptions->push_back(body->get_exceptions()[i]);
	}
}

void PhysXPhysicsServer3D::body_set_max_contacts_reported(RID p_body, int p_contacts) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	ERR_FAIL_MSG("Contact reports not implemented in PhysX.");
	//body->set_max_collisions_detection(p_contacts);
}

int PhysXPhysicsServer3D::body_get_max_contacts_reported(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	ERR_FAIL_V_MSG(0, "Contact reports not implemented in PhysX.");
	//return body->get_max_collisions_detection();
}

void PhysXPhysicsServer3D::body_set_contacts_reported_depth_threshold(RID p_body, float p_threshold) {
	ERR_FAIL_MSG("Contact reports not implemented in PhysX.");
}

float PhysXPhysicsServer3D::body_get_contacts_reported_depth_threshold(RID p_body) const {
	ERR_FAIL_V_MSG(0.0, "Contact reports not implemented in PhysX.");
}

void PhysXPhysicsServer3D::body_set_omit_force_integration(RID p_body, bool p_omit) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_omit_force_integration(p_omit);
}

bool PhysXPhysicsServer3D::body_is_omitting_force_integration(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, false);

	return body->get_omit_force_integration();
}

void PhysXPhysicsServer3D::body_set_force_integration_callback(RID p_body, Object *p_receiver, const StringName &p_method, const Variant &p_udata) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_force_integration_callback(p_receiver ? p_receiver->get_instance_id() : ObjectID(), p_method, p_udata);
}

void PhysXPhysicsServer3D::body_set_ray_pickable(RID p_body, bool p_enable) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_ray_pickable(p_enable);
}

bool PhysXPhysicsServer3D::body_is_ray_pickable(RID p_body) const {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, false);

	return body->is_ray_pickable();
}

PhysicsDirectBodyState3D *PhysXPhysicsServer3D::body_get_direct_state(RID p_body) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, nullptr);

	return PhysXPhysicsDirectBodyState3D::get_singleton(body);
}

bool PhysXPhysicsServer3D::body_test_motion(RID p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, MotionResult *r_result, bool p_exclude_raycast_shapes) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, false);
	ERR_FAIL_COND_V(!body->get_space(), false);

	ERR_FAIL_V_MSG(false, "Test Motion is not implemented in PhysX.");
	//return body->get_space()->test_body_motion(body, p_from, p_motion, p_infinite_inertia, r_result, p_exclude_raycast_shapes);
}

int PhysXPhysicsServer3D::body_test_ray_separation(RID p_body, const Transform &p_transform, bool p_infinite_inertia, Vector3 &r_recover_motion, SeparationResult *r_results, int p_result_max, float p_margin) {
	RigidBodyPhysX *body = rigid_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);
	ERR_FAIL_COND_V(!body->get_space(), 0);

	ERR_FAIL_V_MSG(0, "Test Ray Separation is not implemented in PhysX.");
	//return body->get_space()->test_ray_separation(body, p_transform, p_infinite_inertia, r_recover_motion, r_results, p_result_max, p_margin);
}

RID PhysXPhysicsServer3D::soft_body_create(bool p_init_sleeping) {
	ERR_FAIL_V_MSG(RID(), "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = memnew(SoftBodyPhysX);
	body->set_collision_layer(1);
	body->set_collision_mask(1);
	if (p_init_sleeping) {
		body->set_activation_state(false);
	}
	return create_rid(soft_body_owner, body);*/
}

void PhysXPhysicsServer3D::soft_body_update_rendering_server(RID p_body, class SoftBodyRenderingServerHandler *p_rendering_server_handler) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->update_rendering_server(p_rendering_server_handler);*/
}

void PhysXPhysicsServer3D::soft_body_set_space(RID p_body, RID p_space) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	SpacePhysX *space = nullptr;

	if (p_space.is_valid()) {
		space = space_owner.getornull(p_space);
		ERR_FAIL_COND(!space);
	}

	if (body->get_space() == space)
		return; //pointles

	body->set_space(space);*/
}

RID PhysXPhysicsServer3D::soft_body_get_space(RID p_body) const {
	ERR_FAIL_V_MSG(RID(), "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, RID());

	SpacePhysX *space = body->get_space();
	if (!space)
		return RID();
	return space->get_self();*/
}

void PhysXPhysicsServer3D::soft_body_set_mesh(RID p_body, const REF &p_mesh) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_soft_mesh(p_mesh);*/
}

void PhysXPhysicsServer3D::soft_body_set_collision_layer(RID p_body, uint32_t p_layer) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_collision_layer(p_layer);*/
}

uint32_t PhysXPhysicsServer3D::soft_body_get_collision_layer(RID p_body) const {
	ERR_FAIL_V_MSG(0, "Soft Body is not implemented in PhysX.");

	/*const SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_layer();*/
}

void PhysXPhysicsServer3D::soft_body_set_collision_mask(RID p_body, uint32_t p_mask) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_collision_mask(p_mask);*/
}

uint32_t PhysXPhysicsServer3D::soft_body_get_collision_mask(RID p_body) const {
	ERR_FAIL_V_MSG(0, "Soft Body is not implemented in PhysX.");

	/*const SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_mask();*/
}

void PhysXPhysicsServer3D::soft_body_add_collision_exception(RID p_body, RID p_body_b) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	CollisionObjectPhysX *other_body = rigid_body_owner.getornull(p_body_b);
	if (!other_body) {
		other_body = soft_body_owner.getornull(p_body_b);
	}
	ERR_FAIL_COND(!other_body);

	body->add_collision_exception(other_body);*/
}

void PhysXPhysicsServer3D::soft_body_remove_collision_exception(RID p_body, RID p_body_b) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	CollisionObjectPhysX *other_body = rigid_body_owner.getornull(p_body_b);
	if (!other_body) {
		other_body = soft_body_owner.getornull(p_body_b);
	}
	ERR_FAIL_COND(!other_body);

	body->remove_collision_exception(other_body);*/
}

void PhysXPhysicsServer3D::soft_body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	for (int i = 0; i < body->get_exceptions().size(); i++) {
		p_exceptions->push_back(body->get_exceptions()[i]);
	}*/
}

void PhysXPhysicsServer3D::soft_body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");
}

Variant PhysXPhysicsServer3D::soft_body_get_state(RID p_body, BodyState p_state) const {
	ERR_FAIL_V_MSG(Variant(), "Soft Body is not implemented in PhysX.");
}

void PhysXPhysicsServer3D::soft_body_set_transform(RID p_body, const Transform &p_transform) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);

	body->set_soft_transform(p_transform);*/
}

Vector3 PhysXPhysicsServer3D::soft_body_get_vertex_position(RID p_body, int vertex_index) const {
	ERR_FAIL_V_MSG(Vector3(), "Soft Body is not implemented in PhysX.");

	/*const SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	Vector3 pos;
	ERR_FAIL_COND_V(!body, pos);

	body->get_node_position(vertex_index, pos);
	return pos;*/
}

void PhysXPhysicsServer3D::soft_body_set_ray_pickable(RID p_body, bool p_enable) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_ray_pickable(p_enable);*/
}

bool PhysXPhysicsServer3D::soft_body_is_ray_pickable(RID p_body) const {
	ERR_FAIL_V_MSG(false, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, false);
	return body->is_ray_pickable();*/
}

void PhysXPhysicsServer3D::soft_body_set_simulation_precision(RID p_body, int p_simulation_precision) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_simulation_precision(p_simulation_precision);*/
}

int PhysXPhysicsServer3D::soft_body_get_simulation_precision(RID p_body) {
	ERR_FAIL_V_MSG(0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_simulation_precision();*/
}

void PhysXPhysicsServer3D::soft_body_set_total_mass(RID p_body, real_t p_total_mass) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_total_mass(p_total_mass);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_total_mass(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_total_mass();*/
}

void PhysXPhysicsServer3D::soft_body_set_linear_stiffness(RID p_body, real_t p_stiffness) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_linear_stiffness(p_stiffness);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_linear_stiffness(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_linear_stiffness();*/
}

void PhysXPhysicsServer3D::soft_body_set_areaAngular_stiffness(RID p_body, real_t p_stiffness) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_areaAngular_stiffness(p_stiffness);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_areaAngular_stiffness(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_areaAngular_stiffness();*/
}

void PhysXPhysicsServer3D::soft_body_set_volume_stiffness(RID p_body, real_t p_stiffness) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_volume_stiffness(p_stiffness);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_volume_stiffness(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_volume_stiffness();*/
}

void PhysXPhysicsServer3D::soft_body_set_pressure_coefficient(RID p_body, real_t p_pressure_coefficient) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_pressure_coefficient(p_pressure_coefficient);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_pressure_coefficient(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_pressure_coefficient();*/
}

void PhysXPhysicsServer3D::soft_body_set_pose_matching_coefficient(RID p_body, real_t p_pose_matching_coefficient) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	return body->set_pose_matching_coefficient(p_pose_matching_coefficient);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_pose_matching_coefficient(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_pose_matching_coefficient();*/
}

void PhysXPhysicsServer3D::soft_body_set_damping_coefficient(RID p_body, real_t p_damping_coefficient) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_damping_coefficient(p_damping_coefficient);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_damping_coefficient(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_damping_coefficient();*/
}

void PhysXPhysicsServer3D::soft_body_set_drag_coefficient(RID p_body, real_t p_drag_coefficient) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_drag_coefficient(p_drag_coefficient);*/
}

real_t PhysXPhysicsServer3D::soft_body_get_drag_coefficient(RID p_body) {
	ERR_FAIL_V_MSG(0.0, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_drag_coefficient();*/
}

void PhysXPhysicsServer3D::soft_body_move_point(RID p_body, int p_point_index, const Vector3 &p_global_position) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_node_position(p_point_index, p_global_position);*/
}

Vector3 PhysXPhysicsServer3D::soft_body_get_point_global_position(RID p_body, int p_point_index) {
	ERR_FAIL_V_MSG(Vector3(), "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Vector3(0., 0., 0.));
	Vector3 pos;
	body->get_node_position(p_point_index, pos);
	return pos;*/
}

Vector3 PhysXPhysicsServer3D::soft_body_get_point_offset(RID p_body, int p_point_index) const {
	ERR_FAIL_V_MSG(Vector3(), "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, Vector3());
	Vector3 res;
	body->get_node_offset(p_point_index, res);
	return res;*/
}

void PhysXPhysicsServer3D::soft_body_remove_all_pinned_points(RID p_body) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->reset_all_node_mass();*/
}

void PhysXPhysicsServer3D::soft_body_pin_point(RID p_body, int p_point_index, bool p_pin) {
	ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND(!body);
	body->set_node_mass(p_point_index, p_pin ? 0 : 1);*/
}

bool PhysXPhysicsServer3D::soft_body_is_point_pinned(RID p_body, int p_point_index) {
	ERR_FAIL_V_MSG(false, "Soft Body is not implemented in PhysX.");

	/*SoftBodyPhysX *body = soft_body_owner.getornull(p_body);
	ERR_FAIL_COND_V(!body, 0.f);
	return body->get_node_mass(p_point_index);*/
}

PhysicsServer3D::JointType PhysXPhysicsServer3D::joint_get_type(RID p_joint) const {
	JointPhysX *joint = joint_owner.getornull(p_joint);
	ERR_FAIL_COND_V(!joint, JOINT_PIN);
	return joint->get_type();
}

void PhysXPhysicsServer3D::joint_set_solver_priority(RID p_joint, int p_priority) {
	ERR_FAIL_MSG("Joint solver priority is not implemented in PhysX.");
}

int PhysXPhysicsServer3D::joint_get_solver_priority(RID p_joint) const {
	ERR_FAIL_V_MSG(0, "Joint solver priority is not implemented in PhysX.");
}

void PhysXPhysicsServer3D::joint_disable_collisions_between_bodies(RID p_joint, const bool p_disable) {
	JointPhysX *joint = joint_owner.getornull(p_joint);
	ERR_FAIL_COND(!joint);

	joint->disable_collision(p_disable);
}

bool PhysXPhysicsServer3D::joint_is_disabled_collisions_between_bodies(RID p_joint) const {
	JointPhysX *joint(joint_owner.getornull(p_joint));
	ERR_FAIL_COND_V(!joint, false);

	return joint->is_collision_disabled();
}

bool PhysXPhysicsServer3D::init_joint_rigid_bodies(RID p_rid_A, RID p_rid_B, RigidBodyPhysX *&p_rigid_body_a, RigidBodyPhysX *&p_rigid_body_b) {
	ERR_FAIL_COND_V_MSG(p_rid_A.is_null(), false, "Failed to create joint, body A can't be null.");

	p_rigid_body_a = rigid_body_owner.getornull(p_rid_A);
	ERR_FAIL_COND_V_MSG(!p_rigid_body_a, false, "Failed to create joint, invalid body A.");
	ERR_FAIL_COND_V_MSG(!p_rigid_body_a->get_space(), false, "Failed to create joint, body A is not added to a space.");

	if (p_rid_B.is_valid()) {
		p_rigid_body_b = rigid_body_owner.getornull(p_rid_B);
		ERR_FAIL_COND_V_MSG(!p_rigid_body_b, false, "Failed to create joint, invalid body B.");
		ERR_FAIL_COND_V_MSG(!p_rigid_body_b->get_space(), false, "Failed to create joint, body B is not added to a space.");

		ERR_FAIL_COND_V_MSG(p_rigid_body_a == p_rigid_body_b, false, "Failed to create joint, body A and B are the same object.");
		ERR_FAIL_COND_V_MSG(p_rigid_body_a->get_space() != p_rigid_body_b->get_space(), false, "Failed to create joint, body A and B are not in the same space.");
	} else {
		p_rigid_body_b = nullptr;
	}

	return true;
}

RID PhysXPhysicsServer3D::joint_create_pin(RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(PinJointPhysX(rigid_body_A, rigid_body_B, p_local_A, p_local_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

void PhysXPhysicsServer3D::pin_joint_set_param(RID p_joint, PinJointParam p_param, float p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

float PhysXPhysicsServer3D::pin_joint_get_param(RID p_joint, PinJointParam p_param) const {
	ERR_FAIL_V_MSG(0.0, "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::pin_joint_set_local_a(RID p_joint, const Vector3 &p_A) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

Vector3 PhysXPhysicsServer3D::pin_joint_get_local_a(RID p_joint) const {
	ERR_FAIL_V_MSG(Vector3(), "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::pin_joint_set_local_b(RID p_joint, const Vector3 &p_B) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

Vector3 PhysXPhysicsServer3D::pin_joint_get_local_b(RID p_joint) const {
	ERR_FAIL_V_MSG(Vector3(), "Joints not implemented in PhysX.");
}

RID PhysXPhysicsServer3D::joint_create_hinge(RID p_body_A, const Transform &p_hinge_A, RID p_body_B, const Transform &p_hinge_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(HingeJointPhysX(rigid_body_A, rigid_body_B, p_hinge_A, p_hinge_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

RID PhysXPhysicsServer3D::joint_create_hinge_simple(RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(HingeJointPhysX(rigid_body_A, rigid_body_B, p_pivot_A, p_pivot_B, p_axis_A, p_axis_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

void PhysXPhysicsServer3D::hinge_joint_set_param(RID p_joint, HingeJointParam p_param, float p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

float PhysXPhysicsServer3D::hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const {
	ERR_FAIL_V_MSG(0.0, "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

bool PhysXPhysicsServer3D::hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const {
	ERR_FAIL_V_MSG(0, "Joints not implemented in PhysX.");
}

RID PhysXPhysicsServer3D::joint_create_slider(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(SliderJointPhysX(rigid_body_A, rigid_body_B, p_local_frame_A, p_local_frame_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

void PhysXPhysicsServer3D::slider_joint_set_param(RID p_joint, SliderJointParam p_param, float p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

float PhysXPhysicsServer3D::slider_joint_get_param(RID p_joint, SliderJointParam p_param) const {
	ERR_FAIL_V_MSG(0.0, "Joints not implemented in PhysX.");
}

RID PhysXPhysicsServer3D::joint_create_cone_twist(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(ConeTwistJointPhysX(rigid_body_A, rigid_body_B, p_local_frame_A, p_local_frame_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

void PhysXPhysicsServer3D::cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, float p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

float PhysXPhysicsServer3D::cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const {
	ERR_FAIL_V_MSG(0.0, "Joints not implemented in PhysX.");
}

RID PhysXPhysicsServer3D::joint_create_generic_6dof(RID p_body_A, const Transform &p_local_frame_A, RID p_body_B, const Transform &p_local_frame_B) {
	RigidBodyPhysX *rigid_body_A = nullptr;
	RigidBodyPhysX *rigid_body_B = nullptr;
	if (!init_joint_rigid_bodies(p_body_A, p_body_B, rigid_body_A, rigid_body_B)) {
		return RID();
	}

	JointPhysX *joint = memnew(Generic6DOFJointPhysX(rigid_body_A, rigid_body_B, p_local_frame_A, p_local_frame_B));
	joint->set_space(rigid_body_A->get_space());

	return create_rid(joint_owner, joint);
}

void PhysXPhysicsServer3D::generic_6dof_joint_set_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param, float p_value) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

float PhysXPhysicsServer3D::generic_6dof_joint_get_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param) {
	ERR_FAIL_V_MSG(0.0, "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag, bool p_enable) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

bool PhysXPhysicsServer3D::generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag) {
	ERR_FAIL_V_MSG(false, "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::generic_6dof_joint_set_precision(RID p_joint, int p_precision) {
	ERR_FAIL_MSG("Joints not implemented in PhysX.");
}

int PhysXPhysicsServer3D::generic_6dof_joint_get_precision(RID p_joint) {
	ERR_FAIL_V_MSG(0, "Joints not implemented in PhysX.");
}

void PhysXPhysicsServer3D::free(RID p_rid) {
	if (shape_owner.owns(p_rid)) {
		ShapePhysX *shape = shape_owner.getornull(p_rid);

		// Remove shape from all owners
		for (Map<ShapeOwnerPhysX *, int>::Element *element = shape->get_owners().front(); element; element = element->next()) {
			static_cast<ShapeOwnerPhysX *>(element->key())->remove_shape(shape);
		}

		shape_owner.free(p_rid);
		memdelete(shape);
	} else if (rigid_body_owner.owns(p_rid)) {
		RigidBodyPhysX *body = rigid_body_owner.getornull(p_rid);

		body->set_space(nullptr);
		body->remove_all_shapes(false);

		rigid_body_owner.free(p_rid);
		memdelete(body);
	} else if (soft_body_owner.owns(p_rid)) {
		ERR_FAIL_MSG("Soft Body is not implemented in PhysX.");
		/*SoftBodyPhysX *body = soft_body_owner.getornull(p_rid);

		body->set_space(nullptr);

		soft_body_owner.free(p_rid);
		memdelete(body);*/
	} else if (area_owner.owns(p_rid)) {
		AreaPhysX *area = area_owner.getornull(p_rid);

		area->set_space(nullptr);

		area->remove_all_shapes(false);

		area_owner.free(p_rid);
		memdelete(area);
	} else if (joint_owner.owns(p_rid)) {
		JointPhysX *joint = joint_owner.getornull(p_rid);

		joint->set_space(nullptr);

		joint_owner.free(p_rid);
		memdelete(joint);
	} else if (space_owner.owns(p_rid)) {
		SpacePhysX *space = space_owner.getornull(p_rid);

		space_set_active(p_rid, false);

		space_owner.free(p_rid);
		memdelete(space);
	} else {
		ERR_PRINT("Invalid ID.");
	}
}

void PhysXPhysicsServer3D::init() {
	PhysXPhysicsDirectBodyState3D::init_singleton();
}

void PhysXPhysicsServer3D::step(float p_deltaTime) {
	if (!active) {
		return;
	}

	PhysXPhysicsDirectBodyState3D::singleton_set_delta_time(p_deltaTime);

	int active_space_count = active_spaces.size();
	for (int i = 0; i < active_space_count; ++i) {
		active_spaces[i]->step(p_deltaTime);
	}
}

void PhysXPhysicsServer3D::sync() {
}

void PhysXPhysicsServer3D::flush_queries() {
}

void PhysXPhysicsServer3D::finish() {
	PhysXPhysicsDirectBodyState3D::destroy_singleton();
}

int PhysXPhysicsServer3D::get_process_info(ProcessInfo p_info) {
	return 0;
}

CollisionObjectPhysX *PhysXPhysicsServer3D::get_collision_object(RID p_object) const {
	if (rigid_body_owner.owns(p_object)) {
		return rigid_body_owner.getornull(p_object);
	}
	if (area_owner.owns(p_object)) {
		return area_owner.getornull(p_object);
	}
	if (soft_body_owner.owns(p_object)) {
		ERR_FAIL_V_MSG(nullptr, "Soft Body is not implemented in PhysX.");
		//return soft_body_owner.getornull(p_object);
	}
	return nullptr;
}

RigidCollisionObjectPhysX *PhysXPhysicsServer3D::get_rigid_collision_object(RID p_object) const {
	if (rigid_body_owner.owns(p_object)) {
		return rigid_body_owner.getornull(p_object);
	}
	if (area_owner.owns(p_object)) {
		return area_owner.getornull(p_object);
	}
	return nullptr;
}
