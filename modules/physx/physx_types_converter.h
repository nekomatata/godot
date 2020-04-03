/*************************************************************************/
/*  physx_types_converter.h                                              */
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

#ifndef PHYSX_TYPES_CONVERTER_H
#define PHYSX_TYPES_CONVERTER_H

class Basis;
class Plane;
class Quat;
class Transform;
struct Vector3;

namespace physx {
class PxMat33;
class PxPlane;
class PxQuat;
class PxTransform;
class PxVec3;
} // namespace physx

// PhysX to Godot
extern void PX_TO_G(const physx::PxVec3 &p_in, Vector3 &p_out);
extern void INVERT_PX_TO_G(const physx::PxVec3 &p_in, Vector3 &p_out);
extern void PX_TO_G(const physx::PxQuat &p_in, Quat &p_out);
extern void PX_TO_G(const physx::PxPlane &p_in, Plane &p_out);
extern void PX_TO_G(const physx::PxMat33 &p_in, Basis &p_out);
extern void INVERT_PX_TO_G(const physx::PxMat33 &p_in, Basis &p_out);
extern void PX_TO_G(const physx::PxTransform &p_in, Transform &p_out);

// Godot to PhysX
extern void G_TO_PX(const Vector3 &p_in, physx::PxVec3 &p_out);
extern void INVERT_G_TO_PX(const Vector3 &p_in, physx::PxVec3 &p_out);
extern void G_TO_PX(const Quat &p_in, physx::PxQuat &p_out);
extern void G_TO_PX(const Plane &p_in, physx::PxPlane &p_out);
extern void G_TO_PX(const Basis &p_in, physx::PxMat33 &p_out);
extern void INVERT_G_TO_PX(const Basis &p_in, physx::PxMat33 &p_out);
extern void G_TO_PX(const Transform &p_in, physx::PxTransform &p_out);
#endif
