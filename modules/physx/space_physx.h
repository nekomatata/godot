/*************************************************************************/
/*  space_physx.h                                                        */
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

#ifndef SPACE_PHYSX_H
#define SPACE_PHYSX_H

#include "core/math/vector3.h"
#include "core/variant.h"
#include "core/vector.h"
#include "rid_physx.h"
#include "servers/physics_server_3d.h"

class AreaPhysX;
class JointPhysX;
class PhysXPhysicsDirectSpaceState3D;
class RigidBodyPhysX;
class SoftBodyPhysX;
class Transform;

namespace physx {
class PxActor;
class PxScene;
} // namespace physx

class SpacePhysX : public RIDPhysX {
	physx::PxScene *px_scene = nullptr;
	PhysXPhysicsDirectSpaceState3D *direct_access = nullptr;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	float gravity_magnitude = 9.81;
	float linear_damping = 0.0;
	float angular_damping = 0.0;
	float delta_time = 0.0;
	Vector<physx::PxActor *> actor_buffer;

public:
	SpacePhysX();
	virtual ~SpacePhysX();

	_FORCE_INLINE_ physx::PxScene *get_px_scene() { return px_scene; }

	_FORCE_INLINE_ PhysXPhysicsDirectSpaceState3D *get_direct_state() { return direct_access; }

	_FORCE_INLINE_ const Vector3 &get_gravity_direction() const { return gravity_direction; }
	_FORCE_INLINE_ float get_gravity_magnitude() const { return gravity_magnitude; }

	_FORCE_INLINE_ float get_linear_damping() const { return linear_damping; }
	_FORCE_INLINE_ float get_angular_damping() const { return angular_damping; }

	_FORCE_INLINE_ float get_delta_time() { return delta_time; }
	void step(float p_delta_time);

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer3D::AreaParameter p_param);

	void set_param(PhysicsServer3D::SpaceParameter p_param, float p_value);
	float get_param(PhysicsServer3D::SpaceParameter p_param);

	void add_area(AreaPhysX *p_area);
	void remove_area(AreaPhysX *p_area);

	void add_rigid_body(RigidBodyPhysX *p_body);
	void remove_rigid_body(RigidBodyPhysX *p_body);

	void add_soft_body(SoftBodyPhysX *p_body);
	void remove_soft_body(SoftBodyPhysX *p_body);

	void add_joint(JointPhysX *p_joint, bool p_disable_collision = true);
	void remove_joint(JointPhysX *p_joint);

private:
	void create_scene_internal();
	void destroy_scene_internal();

	void update_gravity_internal();
};
#endif
