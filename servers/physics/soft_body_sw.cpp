/*************************************************************************/
/*  soft_body_sw.cpp                                                     */
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

#include "soft_body_sw.h"
#include "space_sw.h"

#include "core/map.h"

//#define MAX_DISPLACEMENT 1000.0

SoftBodySW::SoftBodySW() :
		CollisionObjectSW(TYPE_SOFT_BODY),
		active_list(this) {

	_set_static(false);

	// TODO: dangular stiffness is actually not used in bullet
	// TODO: volume stiffness is actually not used in bullet
	// TODO: pose matching coefficient is actually not used in bullet

	// TODO: check if refine can be useful
}

void SoftBodySW::_shapes_changed() {
}

void SoftBodySW::set_state(PhysicsServer::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer::BODY_STATE_TRANSFORM: {
			_set_transform(p_variant);
			_set_inv_transform(get_transform().inverse());

			apply_nodes_transform(get_transform());

			// TODO: broadphase update

		} break;
		case PhysicsServer::BODY_STATE_LINEAR_VELOCITY: {
			// Not supported.
			ERR_FAIL_MSG("Linear velocity is not supported for Soft bodies.");
		} break;
		case PhysicsServer::BODY_STATE_ANGULAR_VELOCITY: {
			ERR_FAIL_MSG("Angular velocity is not supported for Soft bodies.");
		} break;
		case PhysicsServer::BODY_STATE_SLEEPING: {
			ERR_FAIL_MSG("Sleeping state is not supported for Soft bodies.");
		} break;
		case PhysicsServer::BODY_STATE_CAN_SLEEP: {
			ERR_FAIL_MSG("Sleeping state is not supported for Soft bodies.");
		} break;
	}
}

Variant SoftBodySW::get_state(PhysicsServer::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer::BODY_STATE_TRANSFORM: {
			return get_transform();
		} break;
		case PhysicsServer::BODY_STATE_LINEAR_VELOCITY: {
			ERR_FAIL_V_MSG("Linear velocity is not supported for Soft bodies.", Vector3());
		} break;
		case PhysicsServer::BODY_STATE_ANGULAR_VELOCITY: {
			ERR_FAIL_V_MSG("Angular velocity is not supported for Soft bodies.", Vector3());
			return Vector3();
		} break;
		case PhysicsServer::BODY_STATE_SLEEPING: {
			ERR_FAIL_V_MSG("Sleeping state is not supported for Soft bodies.", false);
		} break;
		case PhysicsServer::BODY_STATE_CAN_SLEEP: {
			ERR_FAIL_V_MSG("Sleeping state is not supported for Soft bodies.", false);
		} break;
	}

	return Variant();
}

void SoftBodySW::set_space(SpaceSW *p_space) {
	if (get_space()) {
		get_space()->soft_body_remove_from_active_list(&active_list);

		deinitialize_shape();
	}

	_set_space(p_space);

	if (get_space()) {
		get_space()->soft_body_add_to_active_list(&active_list);

		if (bounds != AABB()) {
			initialize_shape(true);
		}
	}
}

void SoftBodySW::set_mesh(const Ref<Mesh> &p_mesh) {
	destroy();

	soft_mesh = p_mesh;

	if (soft_mesh.is_null()) {
		return;
	}

	Array arrays = soft_mesh->surface_get_arrays(0);
	ERR_FAIL_COND(!(soft_mesh->surface_get_format(0) & VS::ARRAY_FORMAT_INDEX));

	bool success = create_from_trimesh(arrays[VS::ARRAY_INDEX], arrays[VS::ARRAY_VERTEX]);
	if (!success) {
		destroy();
		soft_mesh = nullptr;
	}
}

void SoftBodySW::update_visual_server(VisualServerHandler *p_visual_server_handler) {
	if (soft_mesh.is_null()) {
		return;
	}

	const uint32_t nodes_count = nodes.size();
	for (uint32_t node_index = 0; node_index < nodes_count; ++node_index) {
		const Node &node = nodes[node_index];

		const Vector3 &vertex_position = node.x;
		const Vector3 &vertex_normal = node.n;

		// TODO: check if indices_table can be removed & map_visual_to_physics be used instead
		const LocalVector<int> &vs_indices = indices_table[node_index];
		const uint32_t index_count = vs_indices.size();
		for (uint32_t i = 0; i < index_count; ++i) {
			int vs_index = vs_indices[i];
			p_visual_server_handler->set_vertex(vs_index, &vertex_position);
			p_visual_server_handler->set_normal(vs_index, &vertex_normal);
		}
	}

	p_visual_server_handler->set_aabb(bounds);
}

void SoftBodySW::update_bounds() {
	AABB prev_bounds = bounds;
	prev_bounds.grow_by(collision_margin);

	bounds = AABB();

	const uint32_t nodes_count = nodes.size();
	if (nodes_count == 0) {
		deinitialize_shape();
		return;
	}

	bool first = true;
	bool moved = false;
	for (uint32_t node_index = 0; node_index < nodes_count; ++node_index) {
		const Node &node = nodes[node_index];
		if (!prev_bounds.has_point(node.x)) {
			moved = true;
		}
		if (first) {
			bounds.position = node.x;
			first = false;
		} else {
			bounds.expand_to(node.x);
		}
	}

	if (get_space()) {
		initialize_shape(moved);
	}
}

void SoftBodySW::update_constants() {
	reset_link_rest_lengths();
	update_link_constants();
}

void SoftBodySW::reset_link_rest_lengths() {
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &link = links[i];
		link.rl = (link.n[0]->x - link.n[1]->x).length();
		link.c1 = link.rl * link.rl;
	}
}

void SoftBodySW::update_link_constants() {
	real_t inv_linear_stiffness = 1.0 / linear_stiffness;
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &link = links[i];
		link.c0 = (link.n[0]->im + link.n[1]->im) * inv_linear_stiffness;
	}
}

void SoftBodySW::apply_nodes_transform(const Transform &p_transform) {
	if (soft_mesh.is_null()) {
		return;
	}

	size_t node_count = nodes.size();
	Vector3 leaf_size = Vector3(collision_margin, collision_margin, collision_margin) * 2.0;
	for (int node_index = 0; node_index < node_count; ++node_index) {
		Node &node = nodes[node_index];

		node.x = p_transform.xform(node.x);
		node.q = node.x;
		node.v = Vector3();
		node.bv = Vector3();

		AABB node_aabb(node.x, leaf_size);
		node_tree.update(node.leaf, node_aabb);
	}

	// TODO: update normals
	//updateNormals();
	update_bounds();
	update_constants();
}

Vector3 SoftBodySW::get_vertex_position(int p_index) const {
	if (soft_mesh.is_null()) {
		return Vector3();
	}

	ERR_FAIL_INDEX_V(p_index, map_visual_to_physics.size(), Vector3());
	uint32_t node_index = map_visual_to_physics[p_index];

	ERR_FAIL_INDEX_V(node_index, nodes.size(), Vector3());
	return nodes[node_index].x;
}

void SoftBodySW::set_vertex_position(int p_index, const Vector3 &p_position) {
	if (soft_mesh.is_null()) {
		return;
	}

	ERR_FAIL_INDEX(p_index, map_visual_to_physics.size());
	uint32_t node_index = map_visual_to_physics[p_index];

	ERR_FAIL_INDEX(node_index, nodes.size());
	Node &node = nodes[node_index];
	node.q = node.x;
	node.x = p_position;
}

void SoftBodySW::pin_vertex(int p_index) {
	if (is_vertex_pinned(p_index)) {
		return;
	}

	pinned_vertices.push_back(p_index);

	if (!soft_mesh.is_null()) {
		ERR_FAIL_INDEX(p_index, map_visual_to_physics.size());
		uint32_t node_index = map_visual_to_physics[p_index];

		ERR_FAIL_INDEX(node_index, nodes.size());
		Node &node = nodes[node_index];
		node.im = 0.0;
	}
}

void SoftBodySW::unpin_vertex(int p_index) {
	size_t pinned_count = pinned_vertices.size();
	for (uint32_t i = 0; i < pinned_count; ++i) {
		if (p_index == pinned_vertices[i]) {
			pinned_vertices.remove(i);

			if (!soft_mesh.is_null()) {
				ERR_FAIL_INDEX(p_index, map_visual_to_physics.size());
				uint32_t node_index = map_visual_to_physics[p_index];

				ERR_FAIL_INDEX(node_index, nodes.size());
				real_t inv_node_mass = nodes.size() * inv_total_mass;

				Node &node = nodes[node_index];
				node.im = inv_node_mass;
			}

			return;
		}
	}
}

void SoftBodySW::unpin_all_vertices() {
	if (!soft_mesh.is_null()) {
		real_t inv_node_mass = nodes.size() * inv_total_mass;
		size_t pinned_count = pinned_vertices.size();
		for (uint32_t i = 0; i < pinned_count; ++i) {
			uint32_t vertex_index = pinned_vertices[i];

			ERR_CONTINUE(vertex_index >= map_visual_to_physics.size());
			uint32_t node_index = map_visual_to_physics[vertex_index];

			ERR_CONTINUE(node_index >= nodes.size());
			Node &node = nodes[node_index];
			node.im = inv_node_mass;
		}
	}

	pinned_vertices.clear();
}

bool SoftBodySW::is_vertex_pinned(int p_index) const {
	size_t pinned_count = pinned_vertices.size();
	for (uint32_t i = 0; i < pinned_count; ++i) {
		if (p_index == pinned_vertices[i]) {
			return true;
		}
	}

	return false;
}

uint32_t SoftBodySW::get_node_count() const {
	return nodes.size();
}

real_t SoftBodySW::get_node_inv_mass(uint32_t p_node_index) const {
	ERR_FAIL_INDEX_V(p_node_index, nodes.size(), 0.0);
	return nodes[p_node_index].im;
}

Vector3 SoftBodySW::get_node_position(uint32_t p_node_index) const {
	ERR_FAIL_INDEX_V(p_node_index, nodes.size(), Vector3());
	return nodes[p_node_index].x;
}

Vector3 SoftBodySW::get_node_velocity(uint32_t p_node_index) const {
	ERR_FAIL_INDEX_V(p_node_index, nodes.size(), Vector3());
	return nodes[p_node_index].v;
}

Vector3 SoftBodySW::get_node_biased_velocity(uint32_t p_node_index) const {
	ERR_FAIL_INDEX_V(p_node_index, nodes.size(), Vector3());
	return nodes[p_node_index].bv;
}

void SoftBodySW::apply_node_impulse(uint32_t p_node_index, const Vector3 &p_impulse) {
	ERR_FAIL_INDEX(p_node_index, nodes.size());
	Node &node = nodes[p_node_index];
	node.v += p_impulse * node.im;
}

void SoftBodySW::apply_node_bias_impulse(uint32_t p_node_index, const Vector3 &p_impulse) {
	ERR_FAIL_INDEX(p_node_index, nodes.size());
	Node &node = nodes[p_node_index];
	node.bv += p_impulse * node.im;
}

bool SoftBodySW::create_from_trimesh(const PoolVector<int> &p_indices, const PoolVector<Vector3> &p_vertices) {
	uint32_t node_count = 0;
	LocalVector<Vector3> vertices;
	const int visual_vertex_count(p_vertices.size());

	LocalVector<int> triangles;
	const uint32_t triangle_count(p_indices.size() / 3);
	triangles.resize(triangle_count * 3);

	// Merge all overlapping vertices and create a map of physical vertices to visual vertices.
	{
		// Process vertices.
		{
			uint32_t index = 0;
			Map<Vector3, uint32_t> unique_vertices;

			vertices.resize(visual_vertex_count);
			map_visual_to_physics.resize(visual_vertex_count);

			PoolVector<Vector3>::Read vertices_read = p_vertices.read();

			for (int visual_vertex_index = 0; visual_vertex_index < visual_vertex_count; ++visual_vertex_index) {
				const Vector3 &vertex = vertices_read[visual_vertex_index];

				Map<Vector3, uint32_t>::Element *e = unique_vertices.find(vertex);
				uint32_t vertex_id;
				if (e) {
					// Already existing.
					vertex_id = e->value();
				} else {
					// Create new one.
					vertex_id = index++;
					unique_vertices[vertex] = vertex_id;
					indices_table.push_back(LocalVector<int>());
					vertices[vertex_id] = vertex;
				}

				indices_table[vertex_id].push_back(visual_vertex_index);
				map_visual_to_physics[visual_vertex_index] = vertex_id;
			}

			const uint32_t indices_map_size(indices_table.size());
			vertices.resize(indices_map_size);
		}

		// Process triangles.
		{
			PoolVector<int>::Read indices_read = p_indices.read();

			for (int triangle_index = 0; triangle_index < triangle_count; ++triangle_index) {
				for (int i = 0; i < 3; ++i) {
					int visual_index = 3 * triangle_index + i;
					int physics_index = map_visual_to_physics[indices_read[visual_index]];
					triangles[visual_index] = physics_index;
					node_count = MAX(node_count, physics_index);
				}
			}
		}
	}

	++node_count;

	// Create nodes from vertices.
	nodes.resize(node_count);
	real_t inv_node_mass = node_count * inv_total_mass;
	Vector3 leaf_size = Vector3(collision_margin, collision_margin, collision_margin) * 2.0;
	for (uint32_t i = 0; i < node_count; ++i) {
		Node &node = nodes[i];
		node.s = vertices[i];
		node.x = node.s;
		node.q = node.s;
		node.im = inv_node_mass;

		AABB node_aabb(node.x, leaf_size);
		node.leaf = node_tree.insert(node_aabb, &node);

		node.index = i;
	}

	// Create links and faces from triangles.
	LocalVector<bool> chks;
	chks.resize(node_count * node_count);
	memset(chks.ptr(), 0, chks.size() * sizeof(bool));

	for (uint32_t i = 0; i < triangle_count * 3; i += 3) {
		const int idx[] = { triangles[i], triangles[i + 1], triangles[i + 2] };

		for (int j = 2, k = 0; k < 3; j = k++) {
			int chk = idx[k] * node_count + idx[j];
			if (!chks[chk]) {
				chks[chk] = true;
				int inv_chk = idx[j] * node_count + idx[k];
				chks[inv_chk] = true;

				append_link(idx[j], idx[k]);
			}
		}

		append_face(idx[0], idx[1], idx[2]);
	}

	// Set pinned nodes.
	size_t pinned_count = pinned_vertices.size();
	for (uint32_t i = 0; i < pinned_count; ++i) {
		int pinned_vertex = pinned_vertices[i];

		ERR_CONTINUE(pinned_vertex >= visual_vertex_count);
		uint32_t node_index = map_visual_to_physics[pinned_vertex];

		ERR_CONTINUE(node_index >= node_count);
		Node &node = nodes[node_index];
		node.im = 0.0;
	}

	generate_bending_constraints(2);

	update_constants();

	// TODO: btSoftBodyHelpers::ReoptimizeLinkOrder

	// TODO: update normals
	//updateNormals();
	update_bounds();

	return true;
}

void SoftBodySW::generate_bending_constraints(int p_distance) {
	uint32_t i, j;

	if (p_distance > 1) {
		// Build graph.
		const uint32_t n = nodes.size();
		const unsigned inf = (~(unsigned)0) >> 1;
		unsigned *adj = new unsigned[n * n];

#define IDX(_x_, _y_) ((_y_)*n + (_x_))
		for (j = 0; j < n; ++j) {
			for (i = 0; i < n; ++i) {
				int idx_ij = j * n + i;
				int idx_ji = i * n + j;
				if (i != j) {
					adj[idx_ij] = adj[idx_ji] = inf;
				} else {
					adj[idx_ij] = adj[idx_ji] = 0;
				}
			}
		}
		for (i = 0; i < links.size(); ++i) {
			const int ia = (int)(links[i].n[0] - &nodes[0]);
			const int ib = (int)(links[i].n[1] - &nodes[0]);
			int idx = ib * n + ia;
			int idx_inv = ia * n + ib;
			adj[idx] = 1;
			adj[idx_inv] = 1;
		}

		// Special optimized case for distance == 2.
		if (p_distance == 2) {
			LocalVector<LocalVector<int>> node_links;

			// Build node links.
			node_links.resize(nodes.size());

			for (i = 0; i < links.size(); ++i) {
				const int ia = (int)(links[i].n[0] - &nodes[0]);
				const int ib = (int)(links[i].n[1] - &nodes[0]);
				if (node_links[ia].find(ib) == -1) {
					node_links[ia].push_back(ib);
				}

				if (node_links[ib].find(ia) == -1) {
					node_links[ib].push_back(ia);
				}
			}
			for (uint32_t ii = 0; ii < node_links.size(); ii++) {
				int i = ii;

				for (int jj = 0; jj < node_links[ii].size(); jj++) {
					int k = node_links[ii][jj];
					for (int kk = 0; kk < node_links[k].size(); kk++) {
						int j = node_links[k][kk];
						if (i != j) {
							int idx_ik = k * n + i;
							int idx_kj = j * n + k;
							const unsigned sum = adj[idx_ik] + adj[idx_kj];
							ERR_FAIL_COND(sum != 2);
							int idx_ij = j * n + i;
							if (adj[idx_ij] > sum) {
								int idx_ji = j * n + i;
								adj[idx_ij] = adj[idx_ji] = sum;
							}
						}
					}
				}
			}
		} else {
			// Generic Floyd's algorithm.
			for (int k = 0; k < n; ++k) {
				for (j = 0; j < n; ++j) {
					for (i = j + 1; i < n; ++i) {
						int idx_ik = k * n + i;
						int idx_kj = j * n + k;
						const unsigned sum = adj[idx_ik] + adj[idx_kj];
						int idx_ij = j * n + i;
						if (adj[idx_ij] > sum) {
							int idx_ji = j * n + i;
							adj[idx_ij] = adj[idx_ji] = sum;
						}
					}
				}
			}
		}

		// Build links.
		int nlinks = 0;
		for (j = 0; j < n; ++j) {
			for (i = j + 1; i < n; ++i) {
				int idx_ij = j * n + i;
				if (adj[idx_ij] == (unsigned)p_distance) {
					append_link(i, j);
					++nlinks;
				}
			}
		}
		delete[] adj;
	}
}

void SoftBodySW::append_link(uint32_t p_node1, uint32_t p_node2) {
	if (p_node1 == p_node2) {
		return;
	}

	Node *node1 = &nodes[p_node1];
	Node *node2 = &nodes[p_node2];

	Link link;
	link.n[0] = node1;
	link.n[1] = node2;
	link.rl = (node1->x - node2->x).length();

	links.push_back(link);
}

void SoftBodySW::append_face(uint32_t p_node1, uint32_t p_node2, uint32_t p_node3) {
	if (p_node1 == p_node2) {
		return;
	}
	if (p_node1 == p_node3) {
		return;
	}
	if (p_node2 == p_node3) {
		return;
	}

	Node *node1 = &nodes[p_node1];
	Node *node2 = &nodes[p_node2];
	Node *node3 = &nodes[p_node3];

	Face face;
	face.n[0] = node1;
	face.n[1] = node2;
	face.n[2] = node3;

	Vector3 a = node2->x - node1->x;
	Vector3 b = node3->x - node1->x;
	Vector3 cr = vec3_cross(a, b);
	face.ra = cr.length();

	faces.push_back(face);
}

void SoftBodySW::set_iteration_count(int p_val) {
	iteration_count = p_val;
}

void SoftBodySW::set_total_mass(real_t p_val) {
	ERR_FAIL_COND(p_val < 0.0);

	inv_total_mass = 1.0 / p_val;
	real_t mass_factor = total_mass * inv_total_mass;
	total_mass = p_val;

	size_t node_count = nodes.size();
	for (int node_index = 0; node_index < node_count; ++node_index) {
		Node &node = nodes[node_index];
		node.im *= mass_factor;
	}

	update_constants();
}

void SoftBodySW::set_collision_margin(real_t p_val) {
	collision_margin = p_val;
}

void SoftBodySW::set_linear_stiffness(real_t p_val) {
	linear_stiffness = p_val;
}

void SoftBodySW::set_angular_stiffness(real_t p_val) {
	angular_stiffness = p_val;
}

void SoftBodySW::set_volume_stiffness(real_t p_val) {
	volume_stiffness = p_val;
}

void SoftBodySW::set_pressure_coefficient(real_t p_val) {
	pressure_coefficient = p_val;
}

void SoftBodySW::set_pose_matching_coefficient(real_t p_val) {
	pose_matching_coefficient = p_val;
}

void SoftBodySW::set_damping_coefficient(real_t p_val) {
	damping_coefficient = p_val;
}

void SoftBodySW::set_drag_coefficient(real_t p_val) {
	drag_coefficient = p_val;
}

void SoftBodySW::add_velocity(const Vector3 &p_velocity) {
	for (uint32_t i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &node = nodes[i];
		if (node.im > 0) {
			node.v += p_velocity;
		}
	}
}

void SoftBodySW::predict_motion(real_t p_delta) {
	const real_t inv_delta = 1.0 / p_delta;

	uint32_t i, ni;

	// Forces.
	// TODO: gravity from area detection.
	Vector3 gravity(0.0, -9.80665, 0.0);
	add_velocity(gravity * p_delta);

	// Integrate.
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &node = nodes[i];
		node.q = node.x;
		node.x += node.v * p_delta;
	}

	// Bounds and tree update.
	update_bounds();

	// Node tree update.
	Vector3 leaf_size = Vector3(collision_margin, collision_margin, collision_margin) * 2.0;
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		const Node &node = nodes[i];
		AABB node_aabb(node.x, leaf_size);
		node_tree.update(node.leaf, node_aabb);
	}

	// Face tree update.
	/*if (!m_fdbvt.empty()) {
		for (int i = 0; i < m_faces.size(); ++i) {
			Face &f = m_faces[i];
			const btVector3 v = (f.m_n[0]->m_v +
										f.m_n[1]->m_v +
										f.m_n[2]->m_v) /
								3;
			vol = VolumeOf(f, m_sst.radmrg);
			m_fdbvt.update(f.m_leaf,
					vol,
					v * m_sst.velmrg,
					m_sst.updmrg);
		}
	}
	// Clear contacts.
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);*/

	// Optimize node tree.
	node_tree.optimize_incremental(1);
}

void SoftBodySW::solve_constraints(real_t p_delta) {
	const real_t inv_delta = 1.0 / p_delta;

	uint32_t i, ni;

	for (i = 0, ni = links.size(); i < ni; ++i) {
		Link &link = links[i];
		link.c3 = link.n[1]->q - link.n[0]->q;
		link.c2 = 1 / (link.c3.length_squared() * link.c0);
	}

	// Solve velocities.
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &node = nodes[i];
		node.x = node.q + node.v * p_delta;
	}

	// Solve positions.
	for (uint32_t isolve = 0; isolve < iteration_count; ++isolve) {
		const real_t ti = isolve / (real_t)iteration_count;
		solve_links(1.0, ti);
	}
	const real_t vc = (1.0 - damping_coefficient) * inv_delta;
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &node = nodes[i];

		node.x += node.bv * p_delta;
		node.bv = Vector3();

		node.v = (node.x - node.q) * vc;

		node.q = node.x;
	}

	// TODO: update normals
	//updateNormals();
}

void SoftBodySW::solve_links(real_t kst, real_t ti) {
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &link = links[i];
		if (link.c0 > 0) {
			Node &node_a = *link.n[0];
			Node &node_b = *link.n[1];
			const Vector3 del = node_b.x - node_a.x;
			const real_t len = del.length_squared();
			if (link.c1 + len > CMP_EPSILON) {
				const real_t k = ((link.c1 - len) / (link.c0 * (link.c1 + len))) * kst;
				node_a.x -= del * (k * node_a.im);
				node_b.x += del * (k * node_b.im);
			}
		}
	}
}

struct QueryResult {
	const SoftBodySW *soft_body = nullptr;
	void *userdata = nullptr;
	SoftBodySW::QueryResultCallback result_callback = nullptr;

	_FORCE_INLINE_ bool operator()(void *p_data) {
		uint32_t node_index = soft_body->get_node_index(p_data);
		return result_callback(node_index, userdata);
	};
};

void SoftBodySW::query_aabb(const AABB &p_aabb, SoftBodySW::QueryResultCallback p_result_callback, void *p_userdata) {
	QueryResult query_result;
	query_result.soft_body = this;
	query_result.result_callback = p_result_callback;
	query_result.userdata = p_userdata;

	node_tree.aabb_query(p_aabb, query_result);
}

void SoftBodySW::query_ray(const Vector3 &p_from, const Vector3 &p_to, SoftBodySW::QueryResultCallback p_result_callback, void *p_userdata) {
	QueryResult query_result;
	query_result.soft_body = this;
	query_result.result_callback = p_result_callback;
	query_result.userdata = p_userdata;

	node_tree.ray_query(p_from, p_to, query_result);
}

void SoftBodySW::initialize_shape(bool p_force_move) {
	if (get_shape_count() == 0) {
		SoftBodyShapeSW *soft_body_shape = memnew(SoftBodyShapeSW(this));
		add_shape(soft_body_shape);
	} else if (p_force_move) {
		SoftBodyShapeSW *soft_body_shape = static_cast<SoftBodyShapeSW *>(get_shape(0));
		soft_body_shape->update_bounds();
	}
}

void SoftBodySW::deinitialize_shape() {
	if (get_shape_count() > 0) {
		ShapeSW *shape = get_shape(0);
		remove_shape(shape);
		memdelete(shape);
	}
}

void SoftBodySW::destroy() {
	indices_table.clear();
	map_visual_to_physics.clear();

	node_tree.clear();

	nodes.clear();
	links.clear();
	faces.clear();

	bounds = AABB();
	deinitialize_shape();
}

void SoftBodyShapeSW::update_bounds() {
	ERR_FAIL_COND(!soft_body);

	AABB collision_aabb = soft_body->get_bounds();
	collision_aabb.grow_by(soft_body->get_collision_margin());
	configure(collision_aabb);
}

SoftBodyShapeSW::SoftBodyShapeSW(SoftBodySW *p_soft_body) {
	soft_body = p_soft_body;
	update_bounds();
}

bool SoftBodyShapeSW::intersect_segment(const Vector3 &p_begin, const Vector3 &p_end, Vector3 &r_result, Vector3 &r_normal) const {
	// TODO: queries
	return false;
}

bool SoftBodyShapeSW::intersect_point(const Vector3 &p_point) const {
	// TODO: queries
	return false;
}

Vector3 SoftBodyShapeSW::get_closest_point_to(const Vector3 &p_point) const {
	// TODO: queries
	return Vector3();
}
