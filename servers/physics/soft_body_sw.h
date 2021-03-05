/*************************************************************************/
/*  soft_body_sw.h                                                       */
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

#ifndef SOFT_BODY_SW_H
#define SOFT_BODY_SW_H

#include "collision_object_sw.h"

#include "core/local_vector.h"
#include "core/set.h"
#include "core/math/aabb.h"
#include "core/math/vector3.h"
#include "scene/resources/mesh.h"

class ConstraintSW;

class SoftBodySW : public CollisionObjectSW {
	Ref<Mesh> soft_mesh;

	struct Node {
		Vector3 s; // Source position
		Vector3 x; // Position
		Vector3 q; // Previous step position/Test position
		Vector3 bv; // Biased Velocity
		Vector3 v; // Velocity
		Vector3 vsplit; // Temporary Velocity in addintion to velocity used in split impulse
		Vector3 vn; // Previous step velocity
		Vector3 f; // Force accumulator
		Vector3 n; // Normal
		real_t im = 0.0; // 1/mass
		real_t area = 0.0; // Area
		//btDbvtNode *leaf; // Leaf data
		//int battach : 1; // Attached
		int index = 0;
	};

	struct Link {
		Vector3 c3; // gradient
		Node *n[2] = { nullptr, nullptr }; // Node pointers
		real_t rl = 0.0; // Rest length
		//int bending : 1; // Bending link
		real_t c0 = 0.0; // (ima+imb)*kLST
		real_t c1 = 0.0; // rl^2
		real_t c2 = 0.0; // |gradient|^2/c0
	};

	struct Face {
		Node *n[3] = { nullptr, nullptr, nullptr }; // Node pointers
		Vector3 normal; // Normal
		real_t ra; // Rest area
		//btDbvtNode *leaf; // Leaf data
		real_t pcontact[4] = { 0, 0, 0, 0 }; // barycentric weights of the persistent contact
		int index = 0;
	};

	LocalVector<Node> nodes;
	LocalVector<Link> links;
	LocalVector<Face> faces;
	LocalVector<uint32_t> map_visual_to_physics;
	LocalVector<LocalVector<int> > indices_table;

	AABB bounds;

	real_t total_mass = 1.0;
	real_t inv_total_mass = 1.0;
	Vector3 center_of_mass;
	Basis _inv_inertia_tensor;

	int iteration_count = 5;
	real_t linear_stiffness = 0.5; // [0,1]
	real_t angular_stiffness = 0.5; // [0,1]
	real_t volume_stiffness = 0.5; // [0,1]
	real_t pressure_coefficient = 0.0; // [-inf,+inf]
	real_t pose_matching_coefficient = 0.0; // [0,1]
	real_t damping_coefficient = 0.01; // [0,1]
	real_t drag_coefficient = 0.0; // [0,1]
	LocalVector<int> pinned_vertices;

	SelfList<SoftBodySW> active_list;

	Set<ConstraintSW *> constraints;

public:
	SoftBodySW();

	const AABB &get_bounds() const { return bounds; }

	void set_state(PhysicsServer::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer::BodyState p_state) const;

	_FORCE_INLINE_ void add_constraint(ConstraintSW *p_constraint) { constraints.insert(p_constraint); }
	_FORCE_INLINE_ void remove_constraint(ConstraintSW *p_constraint) { constraints.erase(p_constraint); }
	_FORCE_INLINE_ const Set<ConstraintSW *> &get_constraints() const { return constraints; }
	_FORCE_INLINE_ void clear_constraints() { constraints.clear(); }

	virtual void set_space(SpaceSW *p_space);

	void set_mesh(const Ref<Mesh> &p_mesh);

	void update_visual_server(VisualServerHandler *p_visual_server_handler);

	Vector3 get_vertex_position(int p_index) const;
	void set_vertex_position(int p_index, const Vector3 &p_position);

	void pin_vertex(int p_index);
	void unpin_vertex(int p_index);
	void unpin_all_vertices();
	bool is_vertex_pinned(int p_index) const;

	uint32_t get_node_count() const;
	real_t get_node_inv_mass(uint32_t p_node_index) const;
	Vector3 get_node_position(uint32_t p_node_index) const;
	Vector3 get_node_velocity(uint32_t p_node_index) const;
	Vector3 get_node_biased_velocity(uint32_t p_node_index) const;
	void apply_node_impulse(uint32_t p_node_index, const Vector3 &p_impulse);
	void apply_node_bias_impulse(uint32_t p_node_index, const Vector3 &p_impulse);

	void set_iteration_count(int p_val);
	_FORCE_INLINE_ real_t get_iteration_count() const { return iteration_count; }

	void set_total_mass(real_t p_val);
	_FORCE_INLINE_ real_t get_total_mass() const { return total_mass; }
	_FORCE_INLINE_ real_t get_total_inv_mass() const { return inv_total_mass; }

	_FORCE_INLINE_ const Vector3 &get_center_of_mass() const { return center_of_mass; }
	_FORCE_INLINE_ const Basis &get_inv_inertia_tensor() const { return _inv_inertia_tensor; }

	void set_linear_stiffness(real_t p_val);
	_FORCE_INLINE_ real_t get_linear_stiffness() const { return linear_stiffness; }

	void set_angular_stiffness(real_t p_val);
	_FORCE_INLINE_ real_t get_angular_stiffness() const { return angular_stiffness; }

	void set_volume_stiffness(real_t p_val);
	_FORCE_INLINE_ real_t get_volume_stiffness() const { return volume_stiffness; }

	void set_pressure_coefficient(real_t p_val);
	_FORCE_INLINE_ real_t get_pressure_coefficient() const { return pressure_coefficient; }

	void set_pose_matching_coefficient(real_t p_val);
	_FORCE_INLINE_ real_t get_pose_matching_coefficient() const { return pose_matching_coefficient; }

	void set_damping_coefficient(real_t p_val);
	_FORCE_INLINE_ real_t get_damping_coefficient() const { return damping_coefficient; }

	void set_drag_coefficient(real_t p_val);
	_FORCE_INLINE_ real_t get_drag_coefficient() const { return drag_coefficient; }

	void predict_motion(real_t p_delta);
	void solve_constraints(real_t p_delta);

protected:
	virtual void _shapes_changed();

private:
	void update_bounds();
	void update_inertia();
	void update_constants();
	void reset_link_rest_lengths();
	void update_link_constants();

	void apply_nodes_transform(const Transform &p_transform);

	void add_velocity(const Vector3 &p_velocity);
	void apply_forces();

	bool create_from_trimesh(const PoolVector<int> &p_indices, const PoolVector<Vector3> &p_vertices);
	void generate_bending_constraints(int p_distance);
	void append_link(uint32_t p_node1, uint32_t p_node2);
	void append_face(uint32_t p_node1, uint32_t p_node2, uint32_t p_node3);

	void p_solve_links(real_t kst, real_t ti);
	void v_solve_links(real_t kst);

	void initialize_shape(bool p_force_move = true);
	void deinitialize_shape();

	void destroy();
};

class SoftBodyShapeSW : public ShapeSW {
	SoftBodySW *soft_body = nullptr;

public:
	SoftBodySW *get_soft_body() const { return soft_body; }

	virtual PhysicsServer::ShapeType get_type() const { return PhysicsServer::SHAPE_SOFT_BODY; }
	virtual void project_range(const Vector3 &p_normal, const Transform &p_transform, real_t &r_min, real_t &r_max) const { r_min = r_max = 0.0; }
	virtual Vector3 get_support(const Vector3 &p_normal) const { return Vector3(); }
	virtual void get_supports(const Vector3 &p_normal, int p_max, Vector3 *r_supports, int &r_amount, FeatureType &r_type) const { r_amount = 0; }

	virtual bool intersect_segment(const Vector3 &p_begin, const Vector3 &p_end, Vector3 &r_result, Vector3 &r_normal) const;
	virtual bool intersect_point(const Vector3 &p_point) const;
	virtual Vector3 get_closest_point_to(const Vector3 &p_point) const;
	virtual Vector3 get_moment_of_inertia(real_t p_mass) const { return Vector3(); }

	virtual void set_data(const Variant &p_data) {}
	virtual Variant get_data() const { return Variant(); }

	void update_bounds();

	SoftBodyShapeSW(SoftBodySW *p_soft_body);
	~SoftBodyShapeSW() {}
};

#endif // SOFT_BODY_SW_H
