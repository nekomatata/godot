/*************************************************************************/
/*  shape_physx.cpp                                                      */
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

#include "shape_physx.h"

#include "core/image.h"
#include "physx_types_converter.h"
#include "shape_instance_physx.h"
#include "shape_owner_physx.h"

#include "PxPhysics.h"
#include "PxShape.h"
#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxCooking.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "extensions/PxDefaultStreams.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"

using namespace physx;

ShapePhysX::ShapePhysX() {
}

ShapePhysX::~ShapePhysX() {
}

void ShapePhysX::set_margin(float p_margin) {
	margin = p_margin;
	notify_shape_changed();
}

float ShapePhysX::get_margin() const {
	return margin;
}

PxShape *ShapePhysX::create(const ShapeInstancePhysX &p_instance, const PxVec3 &p_global_scale) {
	PxGeometryHolder geometry_holder;
	if (!create_geometry(geometry_holder, p_instance.scale.multiply(p_global_scale), p_instance.extra_edge)) {
		ERR_FAIL_V_MSG(nullptr, "Failed to create shape geometry.");
	}

	PxShape *shape = PxGetPhysics().createShape(geometry_holder.any(), *p_instance.material);
	ERR_FAIL_COND_V_MSG(!shape, nullptr, "Failed to create shape.");

	shape->setContactOffset(margin);
	shape->userData = this;

	return shape;
}

void ShapePhysX::set_transform(PxShape *p_shape, const PxTransform &p_local_transform, const PxVec3 &p_global_scale) const {
	ERR_FAIL_COND(!p_shape);

	PxTransform local_transform(p_local_transform);
	local_transform.p = p_local_transform.p.multiply(p_global_scale);

	adjust_transform(local_transform);

	p_shape->setLocalPose(local_transform);
}

void ShapePhysX::notify_shape_changed() {
	for (Map<ShapeOwnerPhysX *, int>::Element *E = owners.front(); E; E = E->next()) {
		ShapeOwnerPhysX *owner = static_cast<ShapeOwnerPhysX *>(E->key());
		owner->shape_changed(owner->find_shape(this));
	}
}

void ShapePhysX::add_owner(ShapeOwnerPhysX *p_owner) {
	Map<ShapeOwnerPhysX *, int>::Element *E = owners.find(p_owner);
	if (E) {
		E->get()++;
	} else {
		owners[p_owner] = 1; // add new owner
	}
}

void ShapePhysX::remove_owner(ShapeOwnerPhysX *p_owner) {
	Map<ShapeOwnerPhysX *, int>::Element *E = owners.find(p_owner);
	if (!E) {
		return;
	}
	E->get()--;
	if (0 >= E->get()) {
		owners.erase(E);
	}
}

bool ShapePhysX::is_owner(ShapeOwnerPhysX *p_owner) const {
	return owners.has(p_owner);
}

const Map<ShapeOwnerPhysX *, int> &ShapePhysX::get_owners() const {
	return owners;
}

/* PLANE */

PlaneShapePhysX::PlaneShapePhysX() {
}

void PlaneShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	setup(p_data);
}

Variant PlaneShapePhysX::get_data() const {
	Plane godot_plane;
	PX_TO_G(plane, godot_plane);
	return godot_plane;
}

PhysicsServer3D::ShapeType PlaneShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_PLANE;
}

void PlaneShapePhysX::setup(const Plane &p_plane) {
	G_TO_PX(p_plane, plane);
	notify_shape_changed();
}

bool PlaneShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	p_geometry_holder.storeAny(PxPlaneGeometry());

	return true;
}

void PlaneShapePhysX::adjust_transform(PxTransform &p_local_transform) const {
	// Make transformation from plane parameters
	p_local_transform *= PxTransformFromPlaneEquation(plane);
}

/* Sphere */

SphereShapePhysX::SphereShapePhysX() {
}

void SphereShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	setup(p_data);
}

Variant SphereShapePhysX::get_data() const {
	return radius;
}

PhysicsServer3D::ShapeType SphereShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_SPHERE;
}

void SphereShapePhysX::setup(float p_radius) {
	radius = p_radius;
	notify_shape_changed();
}

bool SphereShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	p_geometry_holder.storeAny(PxSphereGeometry(radius * p_scale.x + p_extra_edge));

	return true;
}

/* Box */
BoxShapePhysX::BoxShapePhysX() {
}

void BoxShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	setup(p_data);
}

Variant BoxShapePhysX::get_data() const {
	Vector3 godot_half_extents;
	PX_TO_G(half_extents, godot_half_extents);
	return godot_half_extents;
}

PhysicsServer3D::ShapeType BoxShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_BOX;
}

void BoxShapePhysX::setup(const Vector3 &p_half_extents) {
	G_TO_PX(p_half_extents, half_extents);
	notify_shape_changed();
}

bool BoxShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	PxVec3 geom_half_extents = half_extents.multiply(p_scale);
	geom_half_extents += PxVec3(p_extra_edge);
	p_geometry_holder.storeAny(PxBoxGeometry(geom_half_extents));

	return true;
}

/* Capsule */

const PxQuat CapsuleShapePhysX::CAPSULE_SETUP_ROTATION(PxHalfPi, PxVec3(0.0, 0.0, 1.0));

CapsuleShapePhysX::CapsuleShapePhysX() {
}

void CapsuleShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("radius"));
	ERR_FAIL_COND(!d.has("height"));
	setup(d["height"], d["radius"]);
}

Variant CapsuleShapePhysX::get_data() const {
	Dictionary d;
	d["radius"] = radius;
	d["height"] = height;
	return d;
}

PhysicsServer3D::ShapeType CapsuleShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_CAPSULE;
}

void CapsuleShapePhysX::setup(float p_height, float p_radius) {
	radius = p_radius;
	height = p_height;
	notify_shape_changed();
}

bool CapsuleShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	float geom_radius = radius * p_scale.x + p_extra_edge;
	float geom_half_height = height * 0.5 * p_scale.y + p_extra_edge;
	p_geometry_holder.storeAny(PxCapsuleGeometry(geom_radius, geom_half_height));

	return true;
}

void CapsuleShapePhysX::adjust_transform(PxTransform &p_local_transform) const {
	// Rotate capsule to make sure height is along Y axis
	p_local_transform.q *= CAPSULE_SETUP_ROTATION;
}

/* Cylinder */

CylinderShapePhysX::CylinderShapePhysX() {
}

void CylinderShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("radius"));
	ERR_FAIL_COND(!d.has("height"));
	setup(d["height"], d["radius"]);
}

Variant CylinderShapePhysX::get_data() const {
	Dictionary d;
	d["radius"] = radius;
	d["height"] = height;
	return d;
}

PhysicsServer3D::ShapeType CylinderShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_CYLINDER;
}

void CylinderShapePhysX::setup(float p_height, float p_radius) {
	radius = p_radius;
	height = p_height;
	notify_shape_changed();
}

bool CylinderShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	// TODO: create convex shape with cylinder geometry
	ERR_FAIL_V_MSG(false, "Cylinder shape is not implemented in PhysX.");
}

/* Convex polygon */

ConvexPolygonShapePhysX::ConvexPolygonShapePhysX() {
}

ConvexPolygonShapePhysX::~ConvexPolygonShapePhysX() {
	release();
}

void ConvexPolygonShapePhysX::release() {
	if (convex_mesh) {
		convex_mesh->release();
		convex_mesh = nullptr;
	}
	points.clear();
}

void ConvexPolygonShapePhysX::get_vertices(Vector<Vector3> &p_out_vertices) const {
	const int vertex_count = points.size();
	p_out_vertices.resize(vertex_count);
	for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
		PX_TO_G(points[vertex_index], p_out_vertices.write[vertex_index]);
	}
}

void ConvexPolygonShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	setup(p_cooking, p_data);
}

Variant ConvexPolygonShapePhysX::get_data() const {
	Vector<Vector3> out_vertices;
	get_vertices(out_vertices);
	return out_vertices;
}

PhysicsServer3D::ShapeType ConvexPolygonShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_CONVEX_POLYGON;
}

void ConvexPolygonShapePhysX::setup(PxCooking &p_cooking, const Vector<Vector3> &p_vertices) {
	release();

	const int vertex_count = p_vertices.size();
	points.resize(vertex_count);

	for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
		G_TO_PX(p_vertices[vertex_index], points.write[vertex_index]);
	}

	PxConvexMeshDesc convexDesc;
	convexDesc.points.count = vertex_count;
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.points.data = points.ptr();
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxDefaultMemoryOutputStream writeBuffer;
	PxConvexMeshCookingResult::Enum result;
	bool status = p_cooking.cookConvexMesh(convexDesc, writeBuffer, &result);
	ERR_FAIL_COND_MSG(!status, "Failed to cook convex mesh data with error: " + itos(result) + ".");

	PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	convex_mesh = PxGetPhysics().createConvexMesh(readBuffer);

	notify_shape_changed();
}

bool ConvexPolygonShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	ERR_FAIL_COND_V(!convex_mesh, false);
	p_geometry_holder.storeAny(PxConvexMeshGeometry(convex_mesh, PxMeshScale(p_scale)));

	return true;
}

/* Concave polygon */

ConcavePolygonShapePhysX::ConcavePolygonShapePhysX() {
}

ConcavePolygonShapePhysX::~ConcavePolygonShapePhysX() {
	release();
}

void ConcavePolygonShapePhysX::release() {
	if (triangle_mesh) {
		triangle_mesh->release();
		triangle_mesh = nullptr;
	}
	points.clear();
	triangles.clear();
}

void ConcavePolygonShapePhysX::get_faces(Vector<Vector3> &p_out_face) const {
	const int vertex_count = points.size();
	p_out_face.resize(vertex_count);
	for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
		PX_TO_G(points[vertex_index], p_out_face.write[vertex_index]);
	}
}

void ConcavePolygonShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	setup(p_cooking, p_data);
}

Variant ConcavePolygonShapePhysX::get_data() const {
	Vector<Vector3> out_faces;
	get_faces(out_faces);
	return out_faces;
}

PhysicsServer3D::ShapeType ConcavePolygonShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_CONCAVE_POLYGON;
}

void ConcavePolygonShapePhysX::setup(PxCooking &p_cooking, const Vector<Vector3> &p_faces) {
	release();

	const int vertex_count = p_faces.size();
	ERR_FAIL_COND(vertex_count % 3);

	points.resize(vertex_count);
	triangles.resize(vertex_count);

	for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
		G_TO_PX(p_faces[vertex_index], points.write[vertex_index]);
		triangles.write[vertex_index] = vertex_index;
	}

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = vertex_count;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.points.data = points.ptr();

	meshDesc.triangles.count = vertex_count / 3;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);
	meshDesc.triangles.data = triangles.ptr();

	PxDefaultMemoryOutputStream writeBuffer;
	PxTriangleMeshCookingResult::Enum result;
	bool status = p_cooking.cookTriangleMesh(meshDesc, writeBuffer, &result);
	ERR_FAIL_COND_MSG(!status, "Failed to cook triangle mesh data with error: " + itos(result) + ".");

	PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	triangle_mesh = PxGetPhysics().createTriangleMesh(readBuffer);

	notify_shape_changed();
}

bool ConcavePolygonShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	ERR_FAIL_COND_V(!triangle_mesh, false);
	p_geometry_holder.storeAny(PxTriangleMeshGeometry(triangle_mesh, PxMeshScale(p_scale)));

	return true;
}

PxShape *ConcavePolygonShapePhysX::create(const ShapeInstancePhysX &p_instance, const PxVec3 &p_global_scale) {
	PxShape *shape = ShapePhysX::create(p_instance, p_global_scale);
	ERR_FAIL_COND_V(!shape, nullptr);

	shape->setContactOffset(0.0);
	return shape;
}

/* Height map shape */

HeightMapShapePhysX::HeightMapShapePhysX() {
}

HeightMapShapePhysX::~HeightMapShapePhysX() {
	release();
}

void HeightMapShapePhysX::release() {
	if (height_field) {
		height_field->release();
		height_field = nullptr;
	}
	samples.clear();
}

void HeightMapShapePhysX::get_heights(Vector<float> &p_out_heights) const {
	ERR_FAIL_MSG("Height Map shape is not implemented in PhysX.");
}

void HeightMapShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary d = p_data;
	ERR_FAIL_COND(!d.has("width"));
	ERR_FAIL_COND(!d.has("depth"));
	ERR_FAIL_COND(!d.has("heights"));

	float l_min_height = 0.0;
	float l_max_height = 0.0;

	// If specified, min and max height will be used as precomputed values
	if (d.has("min_height")) {
		l_min_height = d["min_height"];
	}
	if (d.has("max_height")) {
		l_max_height = d["max_height"];
	}

	ERR_FAIL_COND(l_min_height > l_max_height);

	int l_width = d["width"];
	int l_depth = d["depth"];

	Vector<float> l_heights;
	Variant l_heights_v = d["heights"];

	if (l_heights_v.get_type() == Variant::PACKED_FLOAT32_ARRAY) {
		// Ready-to-use heights can be passed
		l_heights = l_heights_v;

	} else if (l_heights_v.get_type() == Variant::OBJECT) {
		// If an image is passed, we have to convert it to a format Bullet supports.
		// this would be expensive to do with a script, so it's nice to have it here.

		Ref<Image> l_image = l_heights_v;
		ERR_FAIL_COND(l_image.is_null());

		// Float is the only common format between Godot and Bullet that can be used for decent collision.
		// (Int16 would be nice too but we still don't have it)
		// We could convert here automatically but it's better to not be intrusive and let the caller do it if necessary.
		ERR_FAIL_COND(l_image->get_format() != Image::FORMAT_RF);

		PackedByteArray im_data = l_image->get_data();

		l_heights.resize(l_image->get_width() * l_image->get_height());

		float *w = l_heights.ptrw();
		const uint8_t *r = im_data.ptr();
		float *rp = (float *)r;
		// At this point, `rp` could be used directly for Bullet, but I don't know how safe it would be.

		for (int i = 0; i < l_heights.size(); ++i) {
			w[i] = rp[i];
		}

	} else {
		ERR_FAIL_MSG("Expected PackedFloat32Array or float Image.");
	}

	ERR_FAIL_COND(l_width <= 0);
	ERR_FAIL_COND(l_depth <= 0);
	ERR_FAIL_COND(l_heights.size() != (l_width * l_depth));

	// Compute min and max heights if not specified.
	if (!d.has("min_height") && !d.has("max_height")) {
		int heights_size = l_heights.size();
		for (int i = 0; i < heights_size; ++i) {
			float height = l_heights[i];
			if (height < l_min_height) {
				l_min_height = height;
			} else if (height > l_max_height) {
				l_max_height = height;
			}
		}
	}

	setup(p_cooking, l_heights, l_width, l_depth, l_min_height, l_max_height);
}

Variant HeightMapShapePhysX::get_data() const {
	Dictionary d;
	d["width"] = width;
	d["depth"] = depth;
	d["min_height"] = min_height;
	d["max_height"] = max_height;
	Vector<float> out_heights;
	get_heights(out_heights);
	d["heights"] = out_heights;
	return d;
}

PhysicsServer3D::ShapeType HeightMapShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_HEIGHTMAP;
}

void HeightMapShapePhysX::setup(PxCooking &p_cooking, const Vector<float> &p_heights, int p_width, int p_depth, float p_min_height, float p_max_height) {
	// TODO: create height field data
	ERR_FAIL_MSG("Height Map shape is not implemented in PhysX.");
}

bool HeightMapShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	ERR_FAIL_COND_V(!height_field, false);

	float height_scale = p_scale.y;
	float row_sale = p_scale.x;
	float column_scale = p_scale.z;

	PxMeshGeometryFlags geometry_flags;
	p_geometry_holder.storeAny(PxHeightFieldGeometry(height_field, geometry_flags, height_scale, row_sale, column_scale));

	return true;
}

/* Ray shape */
RayShapePhysX::RayShapePhysX() {
}

void RayShapePhysX::set_data(PxCooking &p_cooking, const Variant &p_data) {
	Dictionary d = p_data;
	setup(d["length"], d["slips_on_slope"]);
}

Variant RayShapePhysX::get_data() const {
	Dictionary d;
	d["length"] = length;
	d["slips_on_slope"] = slips_on_slope;
	return d;
}

PhysicsServer3D::ShapeType RayShapePhysX::get_type() const {
	return PhysicsServer3D::SHAPE_RAY;
}

void RayShapePhysX::setup(float p_length, bool p_slips_on_slope) {
	length = p_length;
	slips_on_slope = p_slips_on_slope;
	notify_shape_changed();
}

bool RayShapePhysX::create_geometry(PxGeometryHolder &p_geometry_holder, const PxVec3 &p_scale, float p_extra_edge) const {
	ERR_FAIL_V_MSG(false, "Ray shape shape is not implemented in PhysX.");
}
