/*************************************************************************/
/*  body_pair_sw.h                                                       */
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

#ifndef BODY_PAIR_SW_H
#define BODY_PAIR_SW_H

#include "body_sw.h"
#include "constraint_sw.h"
#include "soft_body_sw.h"

class BodyContactSW : public ConstraintSW {
protected:
	enum {
		MAX_CONTACTS = 4
	};

	struct Contact {
		Vector3 position;
		Vector3 normal;
		int index_A, index_B;
		Vector3 local_A, local_B;
		real_t acc_normal_impulse; // accumulated normal impulse (Pn)
		Vector3 acc_tangent_impulse; // accumulated tangent impulse (Pt)
		real_t acc_bias_impulse; // accumulated normal impulse for position bias (Pnb)
		real_t acc_bias_impulse_center_of_mass; // accumulated normal impulse for position bias applied to com
		real_t mass_normal;
		real_t bias;
		real_t bounce;

		real_t depth;
		bool active;
		Vector3 rA, rB; // Offset in world orientation with respect to center of mass
	};

	Vector3 offset_B; //use local A coordinates to avoid numerical issues on collision detection

	Vector3 sep_axis;
	Contact contacts[MAX_CONTACTS];
	int contact_count;
	bool collided;

	static void _contact_added_callback(const Vector3 &p_point_A, int p_index_A, const Vector3 &p_point_B, int p_index_B, void *p_userdata);

	void contact_added_callback(const Vector3 &p_point_A, int p_index_A, const Vector3 &p_point_B, int p_index_B);

	void validate_contacts();

	SpaceSW *space;

	virtual CollisionObjectSW *get_object_a() const = 0;
	virtual CollisionObjectSW *get_object_b() const = 0;

	BodyContactSW(BodySW **p_body_ptr = nullptr, int p_body_count = 0) :
			ConstraintSW(p_body_ptr, p_body_count) {
	}

public:
	virtual ~BodyContactSW() {}
};

class BodyPairSW : public BodyContactSW {
	union {
		struct {
			BodySW *A;
			BodySW *B;
		};

		BodySW *_arr[2];
	};

	int shape_A;
	int shape_B;

	bool _test_ccd(real_t p_step, BodySW *p_A, int p_shape_A, const Transform &p_xform_A, BodySW *p_B, int p_shape_B, const Transform &p_xform_B);

protected:
	virtual CollisionObjectSW *get_object_a() const override { return A; }
	virtual CollisionObjectSW *get_object_b() const override { return B; }

public:
	bool setup(real_t p_step);
	void solve(real_t p_step);

	BodyPairSW(BodySW *p_A, int p_shape_A, BodySW *p_B, int p_shape_B);
	~BodyPairSW();
};

class BodySoftPairSW : public BodyContactSW {
	BodySW *body;
	SoftBodySW *soft_body;

	int body_shape;

protected:
	virtual CollisionObjectSW *get_object_a() const override { return body; }
	virtual CollisionObjectSW *get_object_b() const override { return soft_body; }

public:
	bool setup(real_t p_step);
	void solve(real_t p_step);

	BodySoftPairSW(BodySW *p_A, int p_shape_A, SoftBodySW *p_B);
	~BodySoftPairSW();
};

#endif // BODY_PAIR__SW_H
