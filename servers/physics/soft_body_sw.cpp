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

#define MAX_DISPLACEMENT 1000.0

SoftBodySW::SoftBodySW() :
		CollisionObjectSW(TYPE_SOFT_BODY),
		active_list(this) {

	// TODO: dangular stiffness is actually not used in bullet
	// TODO: volume stiffness is actually not used in bullet
	// TODO: pose matching coefficient is actually not used in bullet

	// TODO: check if refine can be useful
}

void SoftBodySW::_shapes_changed() {
	// TODO: inertia
}

void SoftBodySW::set_state(PhysicsServer::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer::BODY_STATE_TRANSFORM: {
			_set_transform(p_variant);
			_set_inv_transform(get_transform().inverse());

			apply_nodes_transform(get_transform());

			// TODO: broadphase update
			// TODO: activation
			//wakeup();

		} break;
		case PhysicsServer::BODY_STATE_LINEAR_VELOCITY: {
			// TODO: velocity
		} break;
		case PhysicsServer::BODY_STATE_ANGULAR_VELOCITY: {
			// TODO: velocity
		} break;
		case PhysicsServer::BODY_STATE_SLEEPING: {
			// TODO: activation
		} break;
		case PhysicsServer::BODY_STATE_CAN_SLEEP: {
			// TODO: activation
		} break;
	}
}

Variant SoftBodySW::get_state(PhysicsServer::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer::BODY_STATE_TRANSFORM: {
			return get_transform();
		} break;
		case PhysicsServer::BODY_STATE_LINEAR_VELOCITY: {
			// TODO: velocity
			return Vector3();
		} break;
		case PhysicsServer::BODY_STATE_ANGULAR_VELOCITY: {
			// TODO: velocity
			return Vector3();
		} break;
		case PhysicsServer::BODY_STATE_SLEEPING: {
			// TODO: activation
			return false;
		} break;
		case PhysicsServer::BODY_STATE_CAN_SLEEP: {
			// TODO: activation
			return false;
		} break;
	}

	return Variant();
}

void SoftBodySW::set_space(SpaceSW *p_space) {
	if (get_space()) {
		// TODO: activation
		get_space()->soft_body_remove_from_active_list(&active_list);
	}

	_set_space(p_space);

	if (get_space()) {
		// TODO: activation
		get_space()->soft_body_add_to_active_list(&active_list);
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

void SoftBodySW::update_constants() {
	reset_link_rest_lengths();
	update_link_constants();
	// TODO: area
	//updateArea();
}

void SoftBodySW::reset_link_rest_lengths() {
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &l = links[i];
		l.rl = (l.n[0]->x - l.n[1]->x).length();
		l.c1 = l.rl * l.rl;
	}
}

void SoftBodySW::update_link_constants() {
	real_t inv_linear_stiffness = 1.0 / linear_stiffness;
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &l = links[i];
		l.c0 = (l.n[0]->im + l.n[1]->im) * inv_linear_stiffness;
	}
}

void SoftBodySW::apply_nodes_transform(const Transform &p_transform) {
	if (soft_mesh.is_null()) {
		return;
	}

	size_t node_count = nodes.size();
	for (int node_index = 0; node_index < node_count; ++node_index) {
		Node &node = nodes[node_index];

		//node.x = p_transform.xform(node.s);
		node.x = p_transform.xform(node.x);
		node.q = node.x;
		node.v = Vector3();
		node.f = Vector3();

		// TODO: node tree
		//m_ndbvt.update(node.leaf, btDbvtVolume::FromCR(node.x, margin));
	}

	// TODO: update more stuff
	//updateNormals();
	//updateBounds();
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
				real_t inv_node_mass = nodes.size() / total_mass;

				Node &node = nodes[node_index];
				node.im = inv_node_mass;
			}

			return;
		}
	}
}

void SoftBodySW::unpin_all_vertices() {
	if (!soft_mesh.is_null()) {
		real_t inv_node_mass = nodes.size() / total_mass;
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
					// TODO: check if inversion is needed.
					int visual_index = 3 * triangle_index + 2 - i;
					int physics_index = map_visual_to_physics[indices_read[visual_index]];
					triangles[3 * triangle_index + i] = physics_index;
					node_count = MAX(node_count, physics_index);
				}
			}
		}
	}

	++node_count;

	// Create nodes from vertices.
	nodes.resize(node_count);
	real_t inv_node_mass = node_count / total_mass;
	for (uint32_t i = 0; i < node_count; ++i) {
		Node &node = nodes[i];
		node.s = vertices[i];
		node.x = node.s;
		node.q = node.s;
		node.im = inv_node_mass;

		// TODO: node tree
		//node.leaf = m_ndbvt.insert(btDbvtVolume::FromCR(node.x, margin), &node);

		// TODO: material?
		//node.material = pm;
	}

	// Create links and faces from triangles.
	LocalVector<bool> chks;
	chks.resize(node_count * node_count);

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

	// TODO: Check if postpone update is needed
	update_constants();
	//m_bUpdateRtCst = true;

	// TODO: update bounds.
	// TODO: btSoftBodyHelpers::ReoptimizeLinkOrder

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
					//links[links.size() - 1].bending = 1;
					++nlinks;
				}
			}
		}
		delete[] adj;
	}
}

void SoftBodySW::append_link(uint32_t p_node1, uint32_t p_node2) {
	if (p_node1 == p_node2) {
		// TODO: Check if needed (not skipped in bullet)
		return;
	}

	Node *node1 = &nodes[p_node1];
	Node *node2 = &nodes[p_node2];

	Link link;
	link.n[0] = node1;
	link.n[1] = node2;
	link.rl = (node1->x - node2->x).length();

	links.push_back(link);
	//m_bUpdateRtCst = true;
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
	//m_bUpdateRtCst = true;
}

void SoftBodySW::set_iteration_count(int p_val) {
	iteration_count = p_val;
}

void SoftBodySW::set_total_mass(real_t p_val) {
	ERR_FAIL_COND(p_val < 0.0);

	real_t mass_factor = total_mass / p_val;
	total_mass = p_val;

	size_t node_count = nodes.size();
	for (int node_index = 0; node_index < node_count; ++node_index) {
		Node &node = nodes[node_index];
		node.im *= mass_factor;
	}

	//m_bUpdateRtCst = true;
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

void SoftBodySW::apply_forces() {
	// TODO drag coefficient
	// TODO pressure coefficient
}

void SoftBodySW::predict_motion(real_t p_delta) {
	const real_t inv_delta = 1.0 / p_delta;

	uint32_t i, ni;

	// Update.
	/*if (m_bUpdateRtCst) {
		m_bUpdateRtCst = false;
		updateConstants();
		m_fdbvt.clear();
		if (m_cfg.collisions & fCollision::VF_SS) {
			initializeFaceTree();
		}
	}*/
    
    // Prepare.
	/*m_sst.sdt = dt * m_cfg.timescale;
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;*/

	// Forces.
	// TODO: gravity from area detection
	Vector3 gravity(0.0, -9.80665, 0.0);
	add_velocity(gravity * p_delta);
	apply_forces();
	// Integrate.
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &n = nodes[i];
		n.q = n.x;
		// TODO: integrate forces.
		/*Vector3 deltaV = n.f * n.im * p_delta;
		{
			real_t clampDeltaV = MAX_DISPLACEMENT * inv_delta;
			for (int c = 0; c < 3; c++) {
				if (deltaV[c] > clampDeltaV) {
					deltaV[c] = clampDeltaV;
				}
				if (deltaV[c] < -clampDeltaV) {
					deltaV[c] = -clampDeltaV;
				}
			}
		}
		n.v += deltaV;*/
		n.x += n.v * p_delta;
		n.f = Vector3();
	}
	// Bounds.
	// TODO bounds and tree update
	//updateBounds();
	// Nodes.
	/*ATTRIBUTE_ALIGNED16(btDbvtVolume)
	vol;
	for (i = 0, ni = m_nodes.size(); i < ni; ++i) {
		Node &n = m_nodes[i];
		vol = btDbvtVolume::FromCR(n.m_x, m_sst.radmrg);
		m_ndbvt.update(n.m_leaf,
				vol,
				n.m_v * m_sst.velmrg,
				m_sst.updmrg);
	}
	// Faces.
	if (!m_fdbvt.empty()) {
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
	// Optimize dbvt's
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);*/
}

void SoftBodySW::solve_constraints(real_t p_delta) {
	const real_t inv_delta = 1.0 / p_delta;

	uint32_t i, ni;

	for (i = 0, ni = links.size(); i < ni; ++i) {
		Link &l = links[i];
		l.c3 = l.n[1]->q - l.n[0]->q;
		l.c2 = 1 / (l.c3.length_squared() * l.c0);
	}

	// Solve velocities.
	// TODO: test with velocity solver.
	/*for (uint32_t isolve = 0; isolve < iteration_count; ++isolve) {
		v_solve_links(1.0);
	}*/
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &n = nodes[i];
		n.x = n.q + n.v * p_delta;
	}

	// Solve positions.
	for (uint32_t isolve = 0; isolve < iteration_count; ++isolve) {
		const real_t ti = isolve / (real_t)iteration_count;
		p_solve_links(1.0, ti);
		// TODO: Solve contacts
	}
	const real_t vc = (1.0 - damping_coefficient) / p_delta;
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &n = nodes[i];
		n.v = (n.x - n.q) * vc;
		n.f = Vector3();
	}

	// Solve drift.
	const real_t vcf = inv_delta;
	for (i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &n = nodes[i];
		n.q = n.x;
	}
	/*for (uint32_t idrift = 0; idrift < iteration_count; ++idrift) {
		for (int iseq = 0; iseq < m_cfg.m_dsequence.size(); ++iseq) {
			getSolver(m_cfg.m_dsequence[iseq])(this, 1, 0);
		}
	}*/
	for (uint32_t i = 0, ni = nodes.size(); i < ni; ++i) {
		Node &n = nodes[i];
		n.v += (n.x - n.q) * vcf;
	}
}

void SoftBodySW::p_solve_links(real_t kst, real_t ti) {
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &l = links[i];
		if (l.c0 > 0) {
			Node &a = *l.n[0];
			Node &b = *l.n[1];
			const Vector3 del = b.x - a.x;
			const real_t len = del.length_squared();
			if (l.c1 + len > CMP_EPSILON) {
				const real_t k = ((l.c1 - len) / (l.c0 * (l.c1 + len))) * kst;
				a.x -= del * (k * a.im);
				b.x += del * (k * b.im);
			}
		}
	}
}

void SoftBodySW::v_solve_links(real_t kst) {
	for (uint32_t i = 0, ni = links.size(); i < ni; ++i) {
		Link &l = links[i];
		Node **n = l.n;
		const real_t j = -vec3_dot(l.c3, n[0]->v - n[1]->v) * l.c2 * kst;
		n[0]->v += l.c3 * (j * n[0]->im);
		n[1]->v -= l.c3 * (j * n[1]->im);
	}
}

void SoftBodySW::destroy() {
	indices_table.clear();
	map_visual_to_physics.clear();

	nodes.clear();
	links.clear();
	faces.clear();
}
