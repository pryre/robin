#pragma once

#include "fix16.h"
#include "fixvector3d.h"
#include "fixmatrix.h"
#include "fixquat.h"	//TODO: Make a note about quaternion a,b,c,d

//Quaternion layout
//q.a -> q.w
//q.b -> q.x
//q.c -> q.y
//q.d -> q.z

//Quick defines of reused fix16 numbers
#define CONST_ZERO_ZERO_ONE				0x0000028F	//0.01
#define CONST_ZERO_ZERO_EIGHT_THREE_REC 0x00001555	//0.083333
#define CONST_ZERO_FOUR_SIX_REC			0x00006AAA	//0.46666
#define CONST_ZERO_FIVE					0x00008000	//0.5
#define CONST_ZERO_SIX_REC				0x0000AAAA	//0.66666
#define CONST_ZERO_EIGHT_FIVE			0x0000D999	//0.85
#define CONST_ONE						0x00010000	//1
#define CONST_ONE_ONE_FIVE				0x00012666	//1.15
#define CONST_TWO						0x00020000	//2

#define CONST_EPSILON					0x0000FFFF	//0.99999

static inline fix16_t fix16_constrain(fix16_t i, const fix16_t min, const fix16_t max) {
	if(i < min) {
		i = min;
	} else if(i > max) {
		i = max;
	}

	return i;
}

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

//TODO: Should use the mavlink conversions as a base if possible
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
	//Method pulled from the ROS tf2 library
	fix16_t trace = fix16_add(mat->data[0][0], fix16_add(mat->data[1][1], mat->data[2][2]));
	fix16_t temp[4];

	if( trace > 0 ) {
		fix16_t s = fix16_sqrt(fix16_add(trace, CONST_ONE));
		temp[3] = fix16_mul(s, CONST_ZERO_FIVE);
		s = fix16_div(CONST_ZERO_FIVE, s);

		temp[0] = fix16_mul(fix16_sub(mat->data[2][1], mat->data[1][2]), s);
		temp[1] = fix16_mul(fix16_sub(mat->data[0][2], mat->data[2][0]), s);
		temp[2] = fix16_mul(fix16_sub(mat->data[1][0], mat->data[0][1]), s);
	} else {
		int i = mat->data[0][0] < mat->data[1][1] ?
			(mat->data[1][1] < mat->data[2][2] ? 2 : 1) :
			(mat->data[0][0] < mat->data[2][2] ? 2 : 0);
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		fix16_t s = fix16_sqrt(fix16_sub(mat->data[i][i], fix16_sub(mat->data[j][j], fix16_add(mat->data[k][k], CONST_ONE))));
		temp[i] = fix16_mul(s, CONST_ZERO_FIVE);
		s = fix16_div(CONST_ZERO_FIVE, s);

		temp[3] = fix16_mul(fix16_sub(mat->data[k][j], mat->data[j][k]), s);
		temp[j] = fix16_mul(fix16_add(mat->data[j][i], mat->data[i][j]), s);
		temp[k] = fix16_mul(fix16_add(mat->data[k][i], mat->data[i][k]), s);
	}

	dest->a = temp[3];
	dest->b = temp[0];
	dest->c = temp[1];
	dest->d = temp[2];
}
