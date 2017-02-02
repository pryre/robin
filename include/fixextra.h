#pragma once

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"	//TODO: Make a note about quaternion a,b,c,d

#define CONST_EPSILON 0x0000FFFF	//0.99999

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
		q.a = fix16_add(fix16_sqrt(fix16_mul(fix16_sq(v3d_norm(v1)), fix16_sq(v3d_norm(v2)))), v_dot);

		q.b = v_c.x;
		q.c = v_c.y;
		q.d = v_c.z;

		qf16_normalize(dest, &q);
	}
}
