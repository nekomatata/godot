/*************************************************************************/
/*  joint_physx.h                                                        */
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

#ifndef JOINT_PHYSX_H
#define JOINT_PHYSX_H

#include "rid_physx.h"
#include "servers/physics_server_3d.h"

class RigidBodyPhysX;
class SpacePhysX;
class Transform;
struct Vector3;

namespace physx {
class PxJoint;
} // namespace physx

class JointPhysX : public RIDPhysX {
protected:
	SpacePhysX *space = nullptr;
	physx::PxJoint *joint = nullptr;
	bool disabled_collision = true;

public:
	JointPhysX();
	virtual ~JointPhysX();

	virtual PhysicsServer3D::JointType get_type() const = 0;

	virtual void setup(physx::PxJoint *p_joint);
	virtual void set_space(SpacePhysX *p_space);

	void disable_collision(bool p_disable_collision);
	_FORCE_INLINE_ bool is_collision_disabled() const { return disabled_collision; }

public:
	_FORCE_INLINE_ physx::PxJoint *get_px_joint() { return joint; }
};

// TODO: implement joints

class PinJointPhysX : public JointPhysX {
	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_PIN; }

public:
	PinJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Vector3 &p_pos_a, const Vector3 &p_pos_b) {
	}
	virtual ~PinJointPhysX() {}
};

class HingeJointPhysX : public JointPhysX {
	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_HINGE; }

public:
	HingeJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Transform &p_frame_a, const Transform &p_frame_b) {
	}
	HingeJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Vector3 &p_pivot_a, const Vector3 &p_pivot_b, const Vector3 &p_axis_a, const Vector3 &p_axis_b) {
	}
	virtual ~HingeJointPhysX() {}
};

class SliderJointPhysX : public JointPhysX {
	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_SLIDER; }

public:
	SliderJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Transform &p_frame_a, const Transform &p_frame_b) {
	}
	virtual ~SliderJointPhysX() {}
};

class ConeTwistJointPhysX : public JointPhysX {
	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_CONE_TWIST; }

public:
	ConeTwistJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Transform &p_frame_a, const Transform &p_frame_b) {
	}
	virtual ~ConeTwistJointPhysX() {}
};

class Generic6DOFJointPhysX : public JointPhysX {
	virtual PhysicsServer3D::JointType get_type() const { return PhysicsServer3D::JOINT_6DOF; }

public:
	Generic6DOFJointPhysX(RigidBodyPhysX *p_body_a, RigidBodyPhysX *p_body_b, const Transform &p_frame_a, const Transform &p_frame_b) {
	}
	virtual ~Generic6DOFJointPhysX() {}
};

#endif
