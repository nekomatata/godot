/*************************************************************************/
/*  physx_types_converter.cpp                                            */
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

#include "physx_types_converter.h"

#include "core/math/basis.h"
#include "core/math/plane.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vector3.h"

#include "foundation/PxMat33.h"
#include "foundation/PxPlane.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

using namespace physx;

// PhysX to Godot
void PX_TO_G(const PxVec3 &p_in, Vector3 &p_out) {
	p_out.x = p_in.x;
	p_out.y = p_in.y;
	p_out.z = p_in.z;
}

void INVERT_PX_TO_G(const PxVec3 &p_in, Vector3 &p_out) {
	p_out.x = (p_in.x != 0.0) ? 1.0 / p_in.x : 0.0;
	p_out.y = (p_in.y != 0.0) ? 1.0 / p_in.y : 0.0;
	p_out.z = (p_in.z != 0.0) ? 1.0 / p_in.z : 0.0;
}

void PX_TO_G(const PxQuat &p_in, Quat &p_out) {
	p_out.x = p_in.x;
	p_out.y = p_in.y;
	p_out.z = p_in.z;
	p_out.w = p_in.w;
}

void PX_TO_G(const PxPlane &p_in, Plane &p_out) {
	PX_TO_G(p_in.n, p_out.normal);
	p_out.d = p_in.d;
}

void PX_TO_G(const PxMat33 &p_in, Basis &p_out) {
	PX_TO_G(p_in.column0, p_out[0]);
	PX_TO_G(p_in.column1, p_out[1]);
	PX_TO_G(p_in.column2, p_out[2]);
}

void INVERT_PX_TO_G(const PxMat33 &p_in, Basis &p_out) {
	INVERT_PX_TO_G(p_in.column0, p_out[0]);
	INVERT_PX_TO_G(p_in.column1, p_out[1]);
	INVERT_PX_TO_G(p_in.column2, p_out[2]);
}

void PX_TO_G(const PxTransform &p_in, Transform &p_out) {
	Quat godot_quat;
	PX_TO_G(p_in.q, godot_quat);
	p_out.basis.set_quat(godot_quat);
	PX_TO_G(p_in.p, p_out.origin);
}

// Godot to PhysX
void G_TO_PX(const Vector3 &p_in, PxVec3 &p_out) {
	p_out.x = p_in.x;
	p_out.y = p_in.y;
	p_out.z = p_in.z;
}

void INVERT_G_TO_PX(const Vector3 &p_in, PxVec3 &p_out) {
	p_out.x = (p_in.x != 0.0) ? 1.0 / p_in.x : 0.0;
	p_out.y = (p_in.y != 0.0) ? 1.0 / p_in.y : 0.0;
	p_out.z = (p_in.z != 0.0) ? 1.0 / p_in.z : 0.0;
}

void G_TO_PX(const Quat &p_in, PxQuat &p_out) {
	p_out.x = p_in.x;
	p_out.y = p_in.y;
	p_out.z = p_in.z;
	p_out.w = p_in.w;
}

void G_TO_PX(const Plane &p_in, PxPlane &p_out) {
	G_TO_PX(p_in.normal, p_out.n);
	p_out.d = p_in.d;
}

void G_TO_PX(const Basis &p_in, PxMat33 &p_out) {
	G_TO_PX(p_in[0], p_out.column0);
	G_TO_PX(p_in[1], p_out.column1);
	G_TO_PX(p_in[2], p_out.column2);
}

void INVERT_G_TO_PX(const Basis &p_in, PxMat33 &p_out) {
	INVERT_G_TO_PX(p_in[0], p_out.column0);
	INVERT_G_TO_PX(p_in[1], p_out.column1);
	INVERT_G_TO_PX(p_in[2], p_out.column2);
}

void G_TO_PX(const Transform &p_in, PxTransform &p_out) {
	G_TO_PX(p_in.basis, p_out.q);
	G_TO_PX(p_in.origin, p_out.p);
}
