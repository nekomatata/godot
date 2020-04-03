/*************************************************************************/
/*  direct_space_state_physx.cpp                                         */
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

#include "direct_space_state_physx.h"

#include "physics_server_physx.h"
#include "physx_collision_filter.h"
#include "physx_types_converter.h"
#include "shape_physx.h"
#include "space_physx.h"

#include "PxQueryFiltering.h"
#include "PxQueryReport.h"
#include "PxScene.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxSphereGeometry.h"

using namespace physx;

PhysXPhysicsDirectSpaceState3D::PhysXPhysicsDirectSpaceState3D(SpacePhysX *p_space) :
		space(p_space) {
}

int PhysXPhysicsDirectSpaceState3D::intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return 0;
	}

	PxTransform query_transform(PxIdentity);
	G_TO_PX(p_point, query_transform.p);

	PxSphereGeometry point_geometry(0.001f);

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxOverlapBuffer hit;
	space->get_px_scene()->overlap(point_geometry, query_transform, hit, filter);

	// TODO: fill results
	return hit.getNbTouches();
}

bool PhysXPhysicsDirectSpaceState3D::intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_pick_ray) {
	PxVec3 px_from;
	G_TO_PX(p_from, px_from);

	PxVec3 px_dir;
	G_TO_PX(p_to, px_dir);
	float max_distance = px_dir.normalize();

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxRaycastBuffer hit;
	PxHitFlags hit_flags(PxHitFlag::eDEFAULT);
	bool has_hit = space->get_px_scene()->raycast(px_from, px_dir, max_distance, hit, hit_flags, filter);

	// TODO: fill result
	r_result.collider = nullptr;

	return has_hit;
}

int PhysXPhysicsDirectSpaceState3D::intersect_shape(const RID &p_shape, const Transform &p_xform, float p_margin, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return 0;
	}

	ShapePhysX *shape = space->get_physics_server()->get_shape_owner()->getornull(p_shape);
	ERR_FAIL_COND_V(!shape, 0);

	PxTransform query_transform;
	G_TO_PX(p_xform, query_transform);

	PxVec3 shape_scale;
	G_TO_PX(p_xform.basis.get_scale_abs(), shape_scale);

	PxGeometryHolder geometry_holder;
	PxTransform shape_pose(PxIdentity);
	shape->adjust_transform(shape_pose);
	if (!shape->create_geometry(geometry_holder, shape_scale, p_margin)) {
		ERR_FAIL_V_MSG(0, "Failed to create query shape geometry.");
	}

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxOverlapBuffer hit;
	space->get_px_scene()->overlap(geometry_holder.any(), query_transform, hit, filter);

	// TODO: fill results
	return hit.getNbTouches();
}

bool PhysXPhysicsDirectSpaceState3D::cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion, float p_margin, float &r_closest_safe, float &r_closest_unsafe, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, ShapeRestInfo *r_info) {
	ShapePhysX *shape = space->get_physics_server()->get_shape_owner()->getornull(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	PxTransform query_transform;
	G_TO_PX(p_xform, query_transform);

	PxVec3 shape_scale;
	G_TO_PX(p_xform.basis.get_scale_abs(), shape_scale);

	PxGeometryHolder geometry_holder;
	PxTransform shape_pose(PxIdentity);
	shape->adjust_transform(shape_pose);
	if (!shape->create_geometry(geometry_holder, shape_scale, p_margin)) {
		ERR_FAIL_V_MSG(false, "Failed to create query shape geometry.");
	}

	PxVec3 px_dir;
	G_TO_PX(p_motion, px_dir);
	float max_distance = px_dir.normalize();

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxSweepBuffer hit;
	PxHitFlags hit_flags(PxHitFlag::eDEFAULT);
	bool has_hit = space->get_px_scene()->sweep(geometry_holder.any(), query_transform, px_dir, max_distance, hit, hit_flags, filter);

	// TODO: fill results
	return has_hit;
}

bool PhysXPhysicsDirectSpaceState3D::collide_shape(RID p_shape, const Transform &p_shape_xform, float p_margin, Vector3 *r_results, int p_result_max, int &r_result_count, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return false;
	}

	ShapePhysX *shape = space->get_physics_server()->get_shape_owner()->getornull(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	PxTransform query_transform;
	G_TO_PX(p_shape_xform, query_transform);

	PxVec3 shape_scale;
	G_TO_PX(p_shape_xform.basis.get_scale_abs(), shape_scale);

	PxGeometryHolder geometry_holder;
	PxTransform shape_pose(PxIdentity);
	shape->adjust_transform(shape_pose);
	if (!shape->create_geometry(geometry_holder, shape_scale, p_margin)) {
		ERR_FAIL_V_MSG(false, "Failed to create query shape geometry.");
	}

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxOverlapBuffer hit;
	bool has_hit = space->get_px_scene()->overlap(geometry_holder.any(), query_transform, hit, filter);

	// TODO: fill results
	return has_hit;
}

bool PhysXPhysicsDirectSpaceState3D::rest_info(RID p_shape, const Transform &p_shape_xform, float p_margin, ShapeRestInfo *r_info, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	ShapePhysX *shape = space->get_physics_server()->get_shape_owner()->getornull(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	PxTransform query_transform;
	G_TO_PX(p_shape_xform, query_transform);

	PxVec3 shape_scale;
	G_TO_PX(p_shape_xform.basis.get_scale_abs(), shape_scale);

	PxGeometryHolder geometry_holder;
	PxTransform shape_pose(PxIdentity);
	shape->adjust_transform(shape_pose);
	if (!shape->create_geometry(geometry_holder, shape_scale, p_margin)) {
		ERR_FAIL_V_MSG(false, "Failed to create query shape geometry.");
	}

	PxQueryFilterData filter;
	set_query_filter_data(filter.data, p_collision_mask, p_collide_with_bodies, p_collide_with_areas);

	PxOverlapBuffer hit;
	bool has_hit = space->get_px_scene()->overlap(geometry_holder.any(), query_transform, hit, filter);

	// TODO: fill results
	return has_hit;
}

Vector3 PhysXPhysicsDirectSpaceState3D::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {
	// TODO: implement closest point query
	ERR_FAIL_V_MSG(Vector3(), "Closest point query is not supported.");
}
