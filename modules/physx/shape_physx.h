/*************************************************************************/
/*  shape_physx.h                                                        */
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

#ifndef SHAPE_PHYSX_H
#define SHAPE_PHYSX_H

#include "core/int_types.h"
#include "core/map.h"
#include "core/math/plane.h"
#include "core/math/vector3.h"
#include "core/typedefs.h"
#include "core/variant.h"
#include "core/vector.h"
#include "rid_physx.h"
#include "servers/physics_server_3d.h"

#include "foundation/PxPlane.h"
#include "foundation/PxQuat.h"
#include "foundation/PxVec3.h"
#include "geometry/PxHeightFieldSample.h"

class ShapeOwnerPhysX;
struct ShapeInstancePhysX;

namespace physx {
class PxConvexMesh;
class PxCooking;
class PxGeometry;
class PxGeometryHolder;
class PxHeightField;
class PxTriangleMesh;
class PxShape;
} // namespace physx

class ShapePhysX : public RIDPhysX {
	Map<ShapeOwnerPhysX *, int> owners;
	float margin = 0.04;

protected:
	void notify_shape_changed();

public:
	ShapePhysX();
	virtual ~ShapePhysX();

	void set_margin(float p_margin);
	float get_margin() const;

	virtual physx::PxShape *create(const ShapeInstancePhysX &p_instance, const physx::PxVec3 &p_global_scale);
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const = 0;
	virtual void adjust_transform(physx::PxTransform &p_local_transform) const {}
	void set_transform(physx::PxShape *p_shape, const physx::PxTransform &p_local_transform, const physx::PxVec3 &p_global_scale) const;

	void add_owner(ShapeOwnerPhysX *p_owner);
	void remove_owner(ShapeOwnerPhysX *p_owner);
	bool is_owner(ShapeOwnerPhysX *p_owner) const;
	const Map<ShapeOwnerPhysX *, int> &get_owners() const;

	/// Setup the shape
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data) = 0;
	virtual Variant get_data() const = 0;

	virtual PhysicsServer3D::ShapeType get_type() const = 0;
};

class PlaneShapePhysX : public ShapePhysX {
	physx::PxPlane plane = physx::PxPlane(1.0, 0.0, 0.0, 0.0);

public:
	PlaneShapePhysX();

	_FORCE_INLINE_ const physx::PxPlane &get_plane() const { return plane; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;
	virtual void adjust_transform(physx::PxTransform &p_local_transform) const;

private:
	void setup(const Plane &p_plane);
};

class SphereShapePhysX : public ShapePhysX {
	float radius = 0.0;

public:
	SphereShapePhysX();

	_FORCE_INLINE_ float get_radius() const { return radius; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(float p_radius);
};

class BoxShapePhysX : public ShapePhysX {
	physx::PxVec3 half_extents = physx::PxVec3(0.0);

public:
	BoxShapePhysX();

	_FORCE_INLINE_ const physx::PxVec3 &get_half_extents() const { return half_extents; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(const Vector3 &p_half_extents);
};

class CapsuleShapePhysX : public ShapePhysX {
	float radius = 0.0;
	float height = 0.0;

	static const physx::PxQuat CAPSULE_SETUP_ROTATION;

public:
	CapsuleShapePhysX();

	_FORCE_INLINE_ float get_radius() const { return radius; }
	_FORCE_INLINE_ float get_height() const { return height; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;
	virtual void adjust_transform(physx::PxTransform &p_local_transform) const;

private:
	void setup(float p_height, float p_radius);
};

class CylinderShapePhysX : public ShapePhysX {
	float radius = 0.0;
	float height = 0.0;

public:
	CylinderShapePhysX();

	_FORCE_INLINE_ float get_radius() const { return radius; }
	_FORCE_INLINE_ float get_height() const { return height; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(float p_height, float p_radius);
};

class ConvexPolygonShapePhysX : public ShapePhysX {
	physx::PxConvexMesh *convex_mesh = nullptr;
	Vector<physx::PxVec3> points;

public:
	ConvexPolygonShapePhysX();
	virtual ~ConvexPolygonShapePhysX();

	void get_vertices(Vector<Vector3> &p_out_vertices) const;
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(physx::PxCooking &p_cooking, const Vector<Vector3> &p_vertices);
	void release();
};

class ConcavePolygonShapePhysX : public ShapePhysX {
	physx::PxTriangleMesh *triangle_mesh = nullptr;
	Vector<physx::PxVec3> points;
	Vector<uint32_t> triangles;

public:
	ConcavePolygonShapePhysX();
	virtual ~ConcavePolygonShapePhysX();

	void get_faces(Vector<Vector3> &p_out_faces) const;
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual physx::PxShape *create(const ShapeInstancePhysX &p_instance, const physx::PxVec3 &p_global_scale);
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(physx::PxCooking &p_cooking, const Vector<Vector3> &p_faces);
	void release();
};

class HeightMapShapePhysX : public ShapePhysX {
	physx::PxHeightField *height_field = nullptr;
	Vector<physx::PxHeightFieldSample> samples;
	int width = 0;
	int depth = 0;
	float min_height = 0.0;
	float max_height = 0.0;

public:
	HeightMapShapePhysX();
	~HeightMapShapePhysX();

	void get_heights(Vector<float> &p_out_heights) const;
	_FORCE_INLINE_ int get_width() const { return width; }
	_FORCE_INLINE_ int get_depth() const { return depth; }
	_FORCE_INLINE_ float get_min_height() const { return min_height; }
	_FORCE_INLINE_ float get_max_height() const { return max_height; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(physx::PxCooking &p_cooking, const Vector<float> &p_heights, int p_width, int p_depth, float p_min_height, float p_max_height);
	void release();
};

class RayShapePhysX : public ShapePhysX {
	float length = 1.0;
	bool slips_on_slope = false;

public:
	RayShapePhysX();

	_FORCE_INLINE_ float get_length() const { return length; }
	_FORCE_INLINE_ bool is_slip_on_slope_enabled() const { return slips_on_slope; }
	virtual void set_data(physx::PxCooking &p_cooking, const Variant &p_data);
	virtual Variant get_data() const;
	virtual PhysicsServer3D::ShapeType get_type() const;
	virtual bool create_geometry(physx::PxGeometryHolder &p_geometry_holder, const physx::PxVec3 &p_scale, float p_extra_edge) const;

private:
	void setup(float p_length, bool p_slips_on_slope);
};
#endif
