#pragma once

#include "fix16.h"
#include "fixvector3d.h"
#include "fixmatrix.h"
#include "fixquat.h"	//TODO: Make a note about quaternion a,b,c,d

#define CONST_EPSILON	0x0000FFFF	//0.99999
#define CONST_ONE		0x00010000	//1
#define CONST_ZERO_FIVE	0x00008000	//0.5
#define CONST_TWO		0x00020000	//2

//Quaternion layout
//q.a -> q.w
//q.b -> q.x
//q.c -> q.y
//q.d -> q.z

static inline fix16_t v3d_sq_norm(const v3d *a) {
	return fix16_add(fix16_add(fix16_sq(a->x), fix16_sq(a->y)), fix16_sq(a->z));
}

static inline void qf16_inverse(qf16 *dest, const qf16 *q) {
	qf16 q_temp;

	//q_dot = conjugate(q)/norm(q)
	qf16_conj(&q_temp, q);
	qf16_div_s(dest, &q_temp, qf16_norm(q));
}

//Returns the rotation between two vectors
static inline void qf16_from_shortest_path(qf16 *dest, const v3d *v1, const v3d *v2) {
	fix16_t v_dot = v3d_dot(v1, v2);

	//Check to see if they are parallel
	if(v_dot > CONST_EPSILON) {	//The vectors are parallel
		//Identity quaternion
		dest->a = 1;
		dest->b = 0;
		dest->c = 0;
		dest->d = 0;
	} else if(v_dot < -CONST_EPSILON) {	//The vectors are opposite
		//180Deg Roll Quaternion
		dest->a = 0;
		dest->b = 1;
		dest->c = 0;
		dest->d = 0;
	} else {	//The vectors aren't parallel
		qf16 q;

		//Calculate the rotation
		v3d v_c;
		v3d_cross(&v_c, v1, v2);

		//q.w = sqrt((v1.length ^ 2) * (v2.length ^ 2)) + dotproduct(v1, v2)
		//q.a = fix16_add(fix16_sqrt(fix16_mul(fix16_sq(v3d_norm(v1)), fix16_sq(v3d_norm(v2)))), v_dot);

		q.a = fix16_add(CONST_ONE, v_dot);
		q.b = v_c.x;
		q.c = v_c.y;
		q.d = v_c.z;

		qf16_normalize(dest, &q);
	}
}

static inline void euler_from_quat(qf16 *q, fix16_t *phi, fix16_t *theta, fix16_t *psi) {
  *phi = fix16_atan2(fix16_mul(CONST_TWO, fix16_add(fix16_mul(q->a, q->b), fix16_mul(q->c, q->d))),
                      fix16_sub(CONST_ONE, fix16_mul(CONST_TWO, fix16_add(fix16_mul(q->b, q->b), fix16_mul(q->c, q->c)))));

  *theta = fix16_asin(fix16_mul(CONST_TWO, fix16_sub(fix16_mul(q->a, q->c), fix16_mul(q->d, q->b))));

  *psi = fix16_atan2(fix16_mul(CONST_TWO, fix16_add(fix16_mul(q->a, q->d), fix16_mul(q->b, q->c))),
                     fix16_sub(CONST_ONE, fix16_mul(CONST_TWO, fix16_add(fix16_mul(q->c, q->c), fix16_mul(q->d, q->d)))));
}

static inline void quat_from_euler(qf16 *q, fix16_t phi, fix16_t theta, fix16_t psi) {
	fix16_t t0 = fix16_cos(fix16_mul(psi, CONST_ZERO_FIVE));
	fix16_t t1 = fix16_sin(fix16_mul(psi, CONST_ZERO_FIVE));
	fix16_t t2 = fix16_cos(fix16_mul(phi, CONST_ZERO_FIVE));
	fix16_t t3 = fix16_sin(fix16_mul(phi, CONST_ZERO_FIVE));
	fix16_t t4 = fix16_cos(fix16_mul(theta, CONST_ZERO_FIVE));
	fix16_t t5 = fix16_sin(fix16_mul(theta, CONST_ZERO_FIVE));

	q->a = fix16_add(fix16_mul(t0, fix16_mul(t2, t4)), fix16_mul(t1, fix16_mul(t3, t5)));
	q->b = fix16_sub(fix16_mul(t0, fix16_mul(t3, t4)), fix16_mul(t1, fix16_mul(t2, t5)));
	q->c = fix16_add(fix16_mul(t0, fix16_mul(t2, t5)), fix16_mul(t1, fix16_mul(t3, t4)));
	q->d = fix16_sub(fix16_mul(t1, fix16_mul(t2, t4)), fix16_mul(t0, fix16_mul(t3, t5)));
}

static inline void matrix_to_qf16(qf16 *dest, const mf16 *mat) {
	dest->a = fix16_mul(CONST_ZERO_FIVE, fix16_sqrt(fix16_add(mat->data[0][0], fix16_add(mat->data[1][1], fix16_add(mat->data[2][2], 1)))));
	dest->b = fix16_div(fix16_sub(mat->data[1][2], mat->data[2][1]), fix16_mul(4, dest->a));
	dest->c = fix16_div(fix16_sub(mat->data[2][0], mat->data[0][2]), fix16_mul(4, dest->a));
	dest->d = fix16_div(fix16_sub(mat->data[0][1], mat->data[1][0]), fix16_mul(4, dest->a));
}

