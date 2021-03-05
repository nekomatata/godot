/*************************************************************************/
/*  collision_solver_sw.cpp                                              */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2021 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2021 Godot Engine contributors (cf. AUTHORS.md).   */
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

#include "collision_solver_sw.h"
#include "collision_solver_sat.h"
#include "soft_body_sw.h"

#include "gjk_epa.h"

#define collision_solver sat_calculate_penetration
//#define collision_solver gjk_epa_calculate_penetration

bool CollisionSolverSW::solve_static_plane(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, CallbackResult p_result_callback, void *p_userdata, bool p_swap_result) {

	const PlaneShapeSW *plane = static_cast<const PlaneShapeSW *>(p_shape_A);
	if (p_shape_B->get_type() == PhysicsServer::SHAPE_PLANE)
		return false;
	Plane p = p_transform_A.xform(plane->get_plane());

	static const int max_supports = 16;
	Vector3 supports[max_supports];
	int support_count;
	ShapeSW::FeatureType support_type;

	p_shape_B->get_supports(p_transform_B.basis.xform_inv(-p.normal).normalized(), max_supports, supports, support_count, support_type);

	if (support_type == ShapeSW::FEATURE_CIRCLE) {
		ERR_FAIL_COND_V(support_count != 3, false);

		Vector3 circle_pos = supports[0];
		Vector3 circle_axis_1 = supports[1] - circle_pos;
		Vector3 circle_axis_2 = supports[2] - circle_pos;

		// Use 3 equidistant points on the circle.
		for (int i = 0; i < 3; ++i) {
			Vector3 vertex_pos = circle_pos;
			vertex_pos += circle_axis_1 * Math::cos(2.0 * Math_PI * i / 3.0);
			vertex_pos += circle_axis_2 * Math::sin(2.0 * Math_PI * i / 3.0);
			supports[i] = vertex_pos;
		}
	}

	bool found = false;

	for (int i = 0; i < support_count; i++) {

		supports[i] = p_transform_B.xform(supports[i]);
		if (p.distance_to(supports[i]) >= 0)
			continue;
		found = true;

		Vector3 support_A = p.project(supports[i]);

		if (p_result_callback) {
			if (p_swap_result)
				p_result_callback(supports[i], 0, support_A, 0, p_userdata);
			else
				p_result_callback(support_A, 0, supports[i], 0, p_userdata);
		}
	}

	return found;
}

bool CollisionSolverSW::solve_ray(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, CallbackResult p_result_callback, void *p_userdata, bool p_swap_result) {

	const RayShapeSW *ray = static_cast<const RayShapeSW *>(p_shape_A);

	Vector3 from = p_transform_A.origin;
	Vector3 to = from + p_transform_A.basis.get_axis(2) * ray->get_length();
	Vector3 support_A = to;

	Transform ai = p_transform_B.affine_inverse();

	from = ai.xform(from);
	to = ai.xform(to);

	Vector3 p, n;
	if (!p_shape_B->intersect_segment(from, to, p, n))
		return false;

	Vector3 support_B = p_transform_B.xform(p);
	if (ray->get_slips_on_slope()) {
		Vector3 global_n = ai.basis.xform_inv(n).normalized();
		support_B = support_A + (support_B - support_A).length() * global_n;
	}

	if (p_result_callback) {
		if (p_swap_result)
			p_result_callback(support_B, 0, support_A, 0, p_userdata);
		else
			p_result_callback(support_A, 0, support_B, 0, p_userdata);
	}
	return true;
}

struct _SoftBodyContactCollisionInfo {
	int index_A = 0;
	int index_B = 0;
	CollisionSolverSW::CallbackResult result_callback = nullptr;
	void *userdata = nullptr;
	bool swap_result = false;
	int contact_count = 0;
};

void CollisionSolverSW::soft_body_contact_callback(const Vector3 &p_point_A, int p_index_A, const Vector3 &p_point_B, int p_index_B, void *p_userdata) {
	_SoftBodyContactCollisionInfo &cinfo = *(_SoftBodyContactCollisionInfo *)(p_userdata);

	++cinfo.contact_count;

	if (cinfo.swap_result) {
		cinfo.result_callback(p_point_B, cinfo.index_B, p_point_A, cinfo.index_A, cinfo.userdata);
	} else {
		cinfo.result_callback(p_point_A, cinfo.index_A, p_point_B, cinfo.index_B, cinfo.userdata);
	}
}

bool CollisionSolverSW::solve_soft_body(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, CallbackResult p_result_callback, void *p_userdata, bool p_swap_result) {
	const SoftBodyShapeSW *soft_body_shape_B = static_cast<const SoftBodyShapeSW *>(p_shape_B);

	const SoftBodySW *soft_body = soft_body_shape_B->get_soft_body();
	const Transform &world_to_local = soft_body->get_inv_transform();

	_SoftBodyContactCollisionInfo cinfo;
	cinfo.result_callback = p_result_callback;
	cinfo.userdata = p_userdata;
	cinfo.swap_result = p_swap_result;

	// Brute force
	uint32_t node_count = soft_body->get_node_count();
	for (uint32_t node_index = 0; node_index < node_count; ++node_index) {
		Vector3 node_position = soft_body->get_node_position(node_index);
		Vector3 node_local_position = world_to_local.xform(node_position);

		Transform node_transform;
		node_transform.origin = p_transform_B.xform(node_local_position);
		//node_transform.origin = node_position;

		SphereShapeSW sphere_shape;
		sphere_shape.set_data(0.01);

		cinfo.index_B = node_index;
		solve_static(p_shape_A, p_transform_A, &sphere_shape, node_transform, soft_body_contact_callback, &cinfo);
	}

	return (cinfo.contact_count > 0);
}

struct _ConcaveContactCollisionInfo {
	CollisionSolverSW::CallbackResult result_callback = nullptr;
	void *userdata = nullptr;
	FaceShapeSW *face_shape = nullptr;
	bool swap_result = false;
	bool collided = false;
};

void CollisionSolverSW::concave_contact_callback(const Vector3 &p_point_A, int p_index_A, const Vector3 &p_point_B, int p_index_B, void *p_userdata) {

	_ConcaveContactCollisionInfo &cinfo = *(_ConcaveContactCollisionInfo *)(p_userdata);

	if ((p_point_B - p_point_A).dot(cinfo.face_shape->normal) >= 0.0) {
		cinfo.collided = true;
		if (cinfo.swap_result) {
			cinfo.result_callback(p_point_B, p_index_B, p_point_A, p_index_A, cinfo.userdata);
		} else {
			cinfo.result_callback(p_point_A, p_index_A, p_point_B, p_index_B, cinfo.userdata);
		}
	}
}

struct _ConcaveCollisionInfo {

	const Transform *transform_A;
	const ShapeSW *shape_A;
	const Transform *transform_B;
	CollisionSolverSW::CallbackResult result_callback;
	void *userdata;
	bool swap_result;
	bool collided;
	int aabb_tests;
	int collisions;
	bool tested;
	real_t margin_A;
	real_t margin_B;
	Vector3 close_A, close_B;
};

void CollisionSolverSW::concave_callback(void *p_userdata, FaceShapeSW *p_face_shape) {

	_ConcaveCollisionInfo &cinfo = *(_ConcaveCollisionInfo *)(p_userdata);
	cinfo.aabb_tests++;

	_ConcaveContactCollisionInfo contact_cinfo;
	contact_cinfo.result_callback = cinfo.result_callback;
	contact_cinfo.userdata = cinfo.userdata;
	contact_cinfo.swap_result = cinfo.swap_result;
	contact_cinfo.face_shape = p_face_shape;

	collision_solver(cinfo.shape_A, *cinfo.transform_A, p_face_shape, *cinfo.transform_B, concave_contact_callback, &contact_cinfo, false, NULL, cinfo.margin_A, cinfo.margin_B);

	if (contact_cinfo.collided) {
		cinfo.collided = true;
		cinfo.collisions++;
	}
}

bool CollisionSolverSW::solve_concave(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, CallbackResult p_result_callback, void *p_userdata, bool p_swap_result, real_t p_margin_A, real_t p_margin_B) {

	const ConcaveShapeSW *concave_B = static_cast<const ConcaveShapeSW *>(p_shape_B);

	_ConcaveCollisionInfo cinfo;
	cinfo.transform_A = &p_transform_A;
	cinfo.shape_A = p_shape_A;
	cinfo.transform_B = &p_transform_B;
	cinfo.result_callback = p_result_callback;
	cinfo.userdata = p_userdata;
	cinfo.swap_result = p_swap_result;
	cinfo.collided = false;
	cinfo.collisions = 0;
	cinfo.margin_A = p_margin_A;
	cinfo.margin_B = p_margin_B;

	cinfo.aabb_tests = 0;

	Transform rel_transform = p_transform_A;
	rel_transform.origin -= p_transform_B.origin;

	//quickly compute a local AABB

	AABB local_aabb;
	for (int i = 0; i < 3; i++) {

		Vector3 axis(p_transform_B.basis.get_axis(i));
		real_t axis_scale = 1.0 / axis.length();
		axis *= axis_scale;

		real_t smin, smax;
		p_shape_A->project_range(axis, rel_transform, smin, smax);
		smin -= p_margin_A;
		smax += p_margin_A;
		smin *= axis_scale;
		smax *= axis_scale;

		local_aabb.position[i] = smin;
		local_aabb.size[i] = smax - smin;
	}

	local_aabb.grow_by(1.0);

	concave_B->cull(local_aabb, concave_callback, &cinfo);

	return cinfo.collided;
}

bool CollisionSolverSW::solve_static(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, CallbackResult p_result_callback, void *p_userdata, Vector3 *r_sep_axis, real_t p_margin_A, real_t p_margin_B) {

	PhysicsServer::ShapeType type_A = p_shape_A->get_type();
	PhysicsServer::ShapeType type_B = p_shape_B->get_type();
	bool concave_A = p_shape_A->is_concave();
	bool concave_B = p_shape_B->is_concave();

	bool swap = false;

	if (type_A > type_B) {
		SWAP(type_A, type_B);
		SWAP(concave_A, concave_B);
		swap = true;
	}

	if (type_A == PhysicsServer::SHAPE_PLANE) {

		if (type_B == PhysicsServer::SHAPE_PLANE) {
			return false;
		}
		if (type_B == PhysicsServer::SHAPE_RAY) {
			return false;
		}
		if (type_B == PhysicsServer::SHAPE_SOFT_BODY) {
			return false;
		}

		if (swap) {
			return solve_static_plane(p_shape_B, p_transform_B, p_shape_A, p_transform_A, p_result_callback, p_userdata, true);
		} else {
			return solve_static_plane(p_shape_A, p_transform_A, p_shape_B, p_transform_B, p_result_callback, p_userdata, false);
		}

	} else if (type_A == PhysicsServer::SHAPE_RAY) {

		if (type_B == PhysicsServer::SHAPE_RAY) {
			return false;
		}

		if (swap) {
			return solve_ray(p_shape_B, p_transform_B, p_shape_A, p_transform_A, p_result_callback, p_userdata, true);
		} else {
			return solve_ray(p_shape_A, p_transform_A, p_shape_B, p_transform_B, p_result_callback, p_userdata, false);
		}

	} else if (type_B == PhysicsServer::SHAPE_SOFT_BODY) {
		if (type_A == PhysicsServer::SHAPE_SOFT_BODY) {
			return false;
		}

		if (swap) {
			return solve_soft_body(p_shape_B, p_transform_B, p_shape_A, p_transform_A, p_result_callback, p_userdata, true);
		} else {
			return solve_soft_body(p_shape_A, p_transform_A, p_shape_B, p_transform_B, p_result_callback, p_userdata, false);
		}

	} else if (concave_B) {

		if (concave_A)
			return false;

		if (!swap)
			return solve_concave(p_shape_A, p_transform_A, p_shape_B, p_transform_B, p_result_callback, p_userdata, false, p_margin_A, p_margin_B);
		else
			return solve_concave(p_shape_B, p_transform_B, p_shape_A, p_transform_A, p_result_callback, p_userdata, true, p_margin_A, p_margin_B);

	} else {

		return collision_solver(p_shape_A, p_transform_A, p_shape_B, p_transform_B, p_result_callback, p_userdata, false, r_sep_axis, p_margin_A, p_margin_B);
	}
}

void CollisionSolverSW::concave_distance_callback(void *p_userdata, FaceShapeSW *p_face_shape) {

	_ConcaveCollisionInfo &cinfo = *(_ConcaveCollisionInfo *)(p_userdata);
	cinfo.aabb_tests++;
	if (cinfo.collided)
		return;

	Vector3 close_A, close_B;
	cinfo.collided = !gjk_epa_calculate_distance(cinfo.shape_A, *cinfo.transform_A, p_face_shape, *cinfo.transform_B, close_A, close_B);

	if (cinfo.collided)
		return;
	if (!cinfo.tested || close_A.distance_squared_to(close_B) < cinfo.close_A.distance_squared_to(cinfo.close_B)) {

		cinfo.close_A = close_A;
		cinfo.close_B = close_B;
		cinfo.tested = true;
	}

	cinfo.collisions++;
}

bool CollisionSolverSW::solve_distance_plane(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, Vector3 &r_point_A, Vector3 &r_point_B) {

	const PlaneShapeSW *plane = static_cast<const PlaneShapeSW *>(p_shape_A);
	if (p_shape_B->get_type() == PhysicsServer::SHAPE_PLANE)
		return false;
	Plane p = p_transform_A.xform(plane->get_plane());

	static const int max_supports = 16;
	Vector3 supports[max_supports];
	int support_count;
	ShapeSW::FeatureType support_type;

	p_shape_B->get_supports(p_transform_B.basis.xform_inv(-p.normal).normalized(), max_supports, supports, support_count, support_type);

	if (support_type == ShapeSW::FEATURE_CIRCLE) {
		ERR_FAIL_COND_V(support_count != 3, false);

		Vector3 circle_pos = supports[0];
		Vector3 circle_axis_1 = supports[1] - circle_pos;
		Vector3 circle_axis_2 = supports[2] - circle_pos;

		// Use 3 equidistant points on the circle.
		for (int i = 0; i < 3; ++i) {
			Vector3 vertex_pos = circle_pos;
			vertex_pos += circle_axis_1 * Math::cos(2.0 * Math_PI * i / 3.0);
			vertex_pos += circle_axis_2 * Math::sin(2.0 * Math_PI * i / 3.0);
			supports[i] = vertex_pos;
		}
	}

	bool collided = false;
	Vector3 closest;
	real_t closest_d = 0;

	for (int i = 0; i < support_count; i++) {

		supports[i] = p_transform_B.xform(supports[i]);
		real_t d = p.distance_to(supports[i]);
		if (i == 0 || d < closest_d) {
			closest = supports[i];
			closest_d = d;
			if (d <= 0)
				collided = true;
		}
	}

	r_point_A = p.project(closest);
	r_point_B = closest;

	return collided;
}

bool CollisionSolverSW::solve_distance(const ShapeSW *p_shape_A, const Transform &p_transform_A, const ShapeSW *p_shape_B, const Transform &p_transform_B, Vector3 &r_point_A, Vector3 &r_point_B, const AABB &p_concave_hint, Vector3 *r_sep_axis) {

	if (p_shape_A->is_concave())
		return false;

	if (p_shape_B->get_type() == PhysicsServer::SHAPE_PLANE) {

		Vector3 a, b;
		bool col = solve_distance_plane(p_shape_B, p_transform_B, p_shape_A, p_transform_A, a, b);
		r_point_A = b;
		r_point_B = a;
		return !col;

	} else if (p_shape_B->is_concave()) {

		if (p_shape_A->is_concave())
			return false;

		const ConcaveShapeSW *concave_B = static_cast<const ConcaveShapeSW *>(p_shape_B);

		_ConcaveCollisionInfo cinfo;
		cinfo.transform_A = &p_transform_A;
		cinfo.shape_A = p_shape_A;
		cinfo.transform_B = &p_transform_B;
		cinfo.result_callback = NULL;
		cinfo.userdata = NULL;
		cinfo.swap_result = false;
		cinfo.collided = false;
		cinfo.collisions = 0;
		cinfo.aabb_tests = 0;
		cinfo.tested = false;

		Transform rel_transform = p_transform_A;
		rel_transform.origin -= p_transform_B.origin;

		//quickly compute a local AABB

		bool use_cc_hint = p_concave_hint != AABB();
		AABB cc_hint_aabb;
		if (use_cc_hint) {
			cc_hint_aabb = p_concave_hint;
			cc_hint_aabb.position -= p_transform_B.origin;
		}

		AABB local_aabb;
		for (int i = 0; i < 3; i++) {

			Vector3 axis(p_transform_B.basis.get_axis(i));
			real_t axis_scale = ((real_t)1.0) / axis.length();
			axis *= axis_scale;

			real_t smin, smax;

			if (use_cc_hint) {
				cc_hint_aabb.project_range_in_plane(Plane(axis, 0), smin, smax);
			} else {
				p_shape_A->project_range(axis, rel_transform, smin, smax);
			}

			smin *= axis_scale;
			smax *= axis_scale;

			local_aabb.position[i] = smin;
			local_aabb.size[i] = smax - smin;
		}

		concave_B->cull(local_aabb, concave_distance_callback, &cinfo);
		if (!cinfo.collided) {
			r_point_A = cinfo.close_A;
			r_point_B = cinfo.close_B;
		}

		return !cinfo.collided;
	} else {

		return gjk_epa_calculate_distance(p_shape_A, p_transform_A, p_shape_B, p_transform_B, r_point_A, r_point_B); //should pass sepaxis..
	}
}
