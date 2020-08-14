#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "fixextra.h"

const qf16 NED_ENU_Q = {0, _fc_sqrt_0_5, -_fc_sqrt_0_5, 0};
const v3d V3D_ZERO = {0, 0, 0};
const qf16 QF16_NO_ROT = { .a = _fc_1, .b = 0, .c = 0, .d = 0};

fix16_t fix16_wrap_pi( const fix16_t x ) {
	fix16_t a = x;

	if ( a > 0 ) {
		while ( a > fix16_pi ) {
			a = fix16_sub( a, fix16_pi );
		}
	} else {
		while ( a < -fix16_pi ) {
			a = fix16_add( a, fix16_pi );
		}
	}

	return a;
}

fix16_t v3d_sq_norm( const v3d* a ) {
	return fix16_add( fix16_add( fix16_sq( a->x ), fix16_sq( a->y ) ), fix16_sq( a->z ) );
}

fix16_t qf16_dot_full( const qf16* q1, const qf16* q2 ) {
	return fix16_add( fix16_add( fix16_add( fix16_mul( q1->a, q2->a ),
											fix16_mul( q1->b, q2->b ) ),
								 fix16_mul( q1->c, q2->c ) ),
					  fix16_mul( q1->d, q2->d ) );
}

void qf16_inverse( qf16* dest, const qf16* q ) {
	fix16_t normSq = qf16_dot_full( q, q );
	dest->a = fix16_div( q->a, normSq );
	dest->b = -fix16_div( q->b, normSq );
	dest->c = -fix16_div( q->c, normSq );
	dest->d = -fix16_div( q->d, normSq );
}


/*
static inline void qf16_dcm_x( v3d* b_x, const qf16* q ) {
	b_x->x = fix16_add( fix16_sq( q->a ), fix16_add( fix16_sq( q->b ), fix16_add( -fix16_sq( q->c ), fix16_sq( -q->d ) ) ) );
	b_x->y = fix16_mul( _fc_2, fix16_add( fix16_mul( q->b, q->c ), fix16_mul( q->a, q->d ) ) );
	b_x->z = fix16_mul( _fc_2, fix16_sub( fix16_mul( q->b, q->d ), fix16_mul( q->a, q->c ) ) );
}

static inline void qf16_dcm_y( v3d* b_y, const qf16* q ) {
	b_y->x = fix16_mul( _fc_2, fix16_sub( fix16_mul( q->b, q->c ), fix16_mul( q->a, q->d ) ) );
	b_y->y = fix16_add( fix16_sq( q->a ), fix16_add( -fix16_sq( q->b ), fix16_add( fix16_sq( q->c ), fix16_sq( -q->d ) ) ) );
	b_y->z = fix16_mul( _fc_2, fix16_add( fix16_mul( q->a, q->b ), fix16_mul( q->c, q->d ) ) );
}
*/

void qf16_dcm_z( v3d* b_z, const qf16* q ) {
	b_z->x = fix16_mul( _fc_2, fix16_add( fix16_mul( q->a, q->c ), fix16_mul( q->b, q->d ) ) );
	b_z->y = fix16_mul( _fc_2, fix16_sub( fix16_mul( q->c, q->d ), fix16_mul( q->a, q->b ) ) );
	b_z->z = fix16_add( fix16_sq( q->a ), fix16_add( -fix16_sq( q->b ), fix16_add( -fix16_sq( q->c ), fix16_sq( q->d ) ) ) );
}

//Returns the rotation between two vectors
//Method used derrived from PX4/Matrix
void qf16_from_shortest_path( qf16* dest, const v3d* v1, const v3d* v2 ) {
	v3d cr;
	v3d_cross( &cr, v1, v2 );
	fix16_t dt = v3d_dot( v1, v2 );

	if ( ( v3d_norm( &cr ) <= _fc_eps ) && ( dt < 0 ) ) {
		// handle corner cases with 180 degree rotations
		// if the two vectors are parallel, cross product is zero
		// if they point opposite, the dot product is negative
		v3d_abs( &cr, v1 );
		if ( cr.x < cr.y ) {
			if ( cr.x < cr.z ) {
				cr.x = _fc_1;
				cr.y = 0;
				cr.z = 0;
			} else {
				cr.x = 0;
				cr.y = 0;
				cr.z = _fc_1;
			}
		} else {
			if ( cr.y < cr.z ) {
				cr.x = 0;
				cr.y = _fc_1;
				cr.z = 0;
			} else {
				cr.x = 0;
				cr.y = 0;
				cr.z = _fc_1;
			}
		}

		dest->a = 0;
		v3d_cross( &cr, v1, &cr );
	} else {
		// normal case, do half-way quaternion solution
		dest->a = fix16_add( dt, fix16_sqrt( fix16_mul( fix16_sq( v3d_norm( v1 ) ), fix16_sq( v3d_norm( v2 ) ) ) ) );
	}

	dest->b = cr.x;
	dest->c = cr.y;
	dest->d = cr.z;
	qf16_normalize_to_unit( dest, dest );
}

//TODO: Should use the mavlink conversions as a base if we move to float
fix16_t heading_from_quat( qf16* q ) {
	return fix16_atan2( fix16_mul( _fc_2, fix16_add( fix16_mul( q->a, q->d ), fix16_mul( q->b, q->c ) ) ),
						fix16_sub( _fc_1, fix16_mul( _fc_2, fix16_add( fix16_mul( q->c, q->c ), fix16_mul( q->d, q->d ) ) ) ) );

}

void euler_from_quat( qf16* q, fix16_t* phi, fix16_t* theta, fix16_t* psi ) {
	*phi = fix16_atan2( fix16_mul( _fc_2, fix16_add( fix16_mul( q->a, q->b ), fix16_mul( q->c, q->d ) ) ),
						fix16_sub( _fc_1, fix16_mul( _fc_2, fix16_add( fix16_mul( q->b, q->b ), fix16_mul( q->c, q->c ) ) ) ) );

	*theta = fix16_asin( fix16_mul( _fc_2, fix16_sub( fix16_mul( q->a, q->c ), fix16_mul( q->d, q->b ) ) ) );

	*psi = heading_from_quat(q);
}

void quat_from_euler( qf16* q, fix16_t phi, fix16_t theta, fix16_t psi ) {
	fix16_t t0 = fix16_cos( fix16_mul( psi, _fc_0_5 ) );
	fix16_t t1 = fix16_sin( fix16_mul( psi, _fc_0_5 ) );
	fix16_t t2 = fix16_cos( fix16_mul( phi, _fc_0_5 ) );
	fix16_t t3 = fix16_sin( fix16_mul( phi, _fc_0_5 ) );
	fix16_t t4 = fix16_cos( fix16_mul( theta, _fc_0_5 ) );
	fix16_t t5 = fix16_sin( fix16_mul( theta, _fc_0_5 ) );

	q->a = fix16_add( fix16_mul( t0, fix16_mul( t2, t4 ) ), fix16_mul( t1, fix16_mul( t3, t5 ) ) );
	q->b = fix16_sub( fix16_mul( t0, fix16_mul( t3, t4 ) ), fix16_mul( t1, fix16_mul( t2, t5 ) ) );
	q->c = fix16_add( fix16_mul( t0, fix16_mul( t2, t5 ) ), fix16_mul( t1, fix16_mul( t3, t4 ) ) );
	q->d = fix16_sub( fix16_mul( t1, fix16_mul( t2, t4 ) ), fix16_mul( t0, fix16_mul( t3, t5 ) ) );
}

void matrix_to_qf16( qf16* dest, const mf16* mat ) {
	//Method pulled from the ROS tf2 library
	fix16_t trace = fix16_add( mat->data[0][0], fix16_add( mat->data[1][1], mat->data[2][2] ) );
	fix16_t temp[4];

	if ( trace > 0 ) {
		fix16_t s = fix16_sqrt( fix16_add( trace, _fc_1 ) );
		temp[3] = fix16_mul( s, _fc_0_5 );
		s = fix16_div( _fc_0_5, s );

		temp[0] = fix16_mul( fix16_sub( mat->data[2][1], mat->data[1][2] ), s );
		temp[1] = fix16_mul( fix16_sub( mat->data[0][2], mat->data[2][0] ), s );
		temp[2] = fix16_mul( fix16_sub( mat->data[1][0], mat->data[0][1] ), s );
	} else {
		int i = mat->data[0][0] < mat->data[1][1] ? ( mat->data[1][1] < mat->data[2][2] ? 2 : 1 ) : ( mat->data[0][0] < mat->data[2][2] ? 2 : 0 );
		int j = ( i + 1 ) % 3;
		int k = ( i + 2 ) % 3;

		fix16_t s = fix16_sqrt( fix16_sub( mat->data[i][i], fix16_sub( mat->data[j][j], fix16_add( mat->data[k][k], _fc_1 ) ) ) );
		temp[i] = fix16_mul( s, _fc_0_5 );
		s = fix16_div( _fc_0_5, s );

		temp[3] = fix16_mul( fix16_sub( mat->data[k][j], mat->data[j][k] ), s );
		temp[j] = fix16_mul( fix16_add( mat->data[j][i], mat->data[i][j] ), s );
		temp[k] = fix16_mul( fix16_add( mat->data[k][i], mat->data[i][k] ), s );
	}

	dest->a = temp[3];
	dest->b = temp[0];
	dest->c = temp[1];
	dest->d = temp[2];
}

void dcm_to_basis( v3d* b_x, v3d* b_y, v3d* b_z, const mf16* dcm ) {
	dcm_to_basis_x( b_x, dcm );
	dcm_to_basis_y( b_y, dcm );
	dcm_to_basis_z( b_z, dcm );
}

void dcm_from_basis( mf16* dcm, const v3d* b_x, const v3d* b_y, const v3d* b_z ) {
	dcm->rows = dcm->columns = 3;
	dcm->errors = 0;

	dcm->data[0][0] = b_x->x;
	dcm->data[0][1] = b_x->y;
	dcm->data[0][2] = b_x->z;

	dcm->data[1][0] = b_y->x;
	dcm->data[1][1] = b_y->y;
	dcm->data[1][2] = b_y->z;

	dcm->data[2][0] = b_z->x;
	dcm->data[2][1] = b_z->y;
	dcm->data[2][2] = b_z->z;
}

void qf16_to_dcm( mf16* dcm, const qf16* q ) {
	const fix16_t a = q->a;
	const fix16_t b = q->b;
	const fix16_t c = q->c;
	const fix16_t d = q->d;
	const fix16_t aa = fix16_mul( a, a );
	const fix16_t ab = fix16_mul( a, b );
	const fix16_t ac = fix16_mul( a, c );
	const fix16_t ad = fix16_mul( a, d );
	const fix16_t bb = fix16_mul( b, b );
	const fix16_t bc = fix16_mul( b, c );
	const fix16_t bd = fix16_mul( b, d );
	const fix16_t cc = fix16_mul( c, c );
	const fix16_t cd = fix16_mul( c, d );
	const fix16_t dd = fix16_mul( d, d );

	dcm->rows = dcm->columns = 3;
	dcm->errors = 0;

	dcm->data[0][0] = fix16_sub( fix16_sub( fix16_add( aa, bb ), cc ), dd );
	dcm->data[0][1] = fix16_mul( _fc_2, fix16_sub( bc, ad ) );
	dcm->data[0][2] = fix16_mul( _fc_2, fix16_add( ac, bd ) );
	dcm->data[1][0] = fix16_mul( _fc_2, fix16_add( bc, ad ) );
	dcm->data[1][1] = fix16_sub( fix16_add( fix16_sub( aa, bb ), cc ), dd );
	dcm->data[1][2] = fix16_mul( _fc_2, fix16_sub( cd, ab ) );
	dcm->data[2][0] = fix16_mul( _fc_2, fix16_sub( bd, ac ) );
	dcm->data[2][1] = fix16_mul( _fc_2, fix16_add( ab, cd ) );
	dcm->data[2][2] = fix16_add( fix16_sub( fix16_sub( aa, bb ), cc ), dd );
}

void vee_up( mf16* W, const v3d* w ) {
	W->rows = W->columns = 3;
	W->errors = 0;

	W->data[0][0] = 0;
	W->data[0][1] = w->z;
	W->data[0][2] = -w->y;
	W->data[1][0] = -w->z;
	W->data[1][1] = 0;
	W->data[1][2] = w->x;
	W->data[2][0] = w->y;
	W->data[2][1] = -w->x;
	W->data[2][2] = 0;
}

void vee_down( v3d* w, const mf16* W ) {
	w->x = W->data[2][1];
	w->y = W->data[0][2];
	w->z = W->data[1][0];
}

void qf16_align_to_axis( qf16* dest, const qf16* input, const qf16* reference, const qf16_axis_lock_t axis ) {
	qf16 q_temp;
	qf16 dq;

	qf16_inverse( &q_temp, input );
	qf16_mul( &dq, reference, &q_temp ); //Difference between reference and input
	qf16_normalize_to_unit( &dq, &dq );

	v3d fv; //Flat vector

	fv.x = _fc_1; //Set to directly forward, no rotation
	fv.y = 0;
	fv.z = 0;

	qf16_rotate( &fv, &dq, &fv ); //Rotate the flat vector by the difference in rotation

	v3d rot_axis;	  //Axis of rotation
	fix16_t rot_a = 0; //Rotation angle

	switch ( axis ) {
	case AXIS_LOCK_X: {
		fv.x = 0;
		v3d_normalize( &fv, &fv ); //Re-flatten, and normalize

		rot_a = fix16_atan2( fv.z, fv.y ); //Get the angular difference

		rot_axis.x = _fc_1;
		rot_axis.y = 0;
		rot_axis.z = 0;

		break;
	}
	case AXIS_LOCK_Y: {
		fv.y = 0;
		v3d_normalize( &fv, &fv ); //Re-flatten, and normalize

		rot_a = fix16_atan2( fv.x, fv.z ); //Get the angular difference

		rot_axis.x = 0;
		rot_axis.y = _fc_1;
		rot_axis.z = 0;

		break;
	}
	case AXIS_LOCK_Z: {
		fv.z = 0;
		v3d_normalize( &fv, &fv ); //Re-flatten, and normalize

		rot_a = fix16_atan2( fv.y, fv.x ); //Get the angular difference

		rot_axis.x = 0;
		rot_axis.y = 0;
		rot_axis.z = _fc_1;

		break;
	}
	default: {
		rot_a = 0;

		rot_axis.x = _fc_1;
		rot_axis.y = 0;
		rot_axis.z = 0;
	}
	}

	qf16 q_rot;
	qf16_from_axis_angle( &q_rot, &rot_axis, rot_a ); //Create a rotation quaternion for the flat rotation the axis
	qf16_mul( dest, &q_rot, input );				  //Rotate the control input

	//Normalize quaternion
	qf16_normalize_to_unit( dest, dest );
}

//Calculates the basis error representing body the rate rotations required
//to rotate q1 to q2, in the frame of q1
//This could be used to calculate angular control error, with q1 being the
//state, and q2 being the reference
void qf16_basis_error( v3d* we, qf16* qe, const qf16* q1, const qf16* q2 ) {
	//qf16 qe;
	qf16_inverse( qe, q1 );
	qf16_normalize_to_unit( qe, qe ); //TODO: Probably not needed
	qf16_mul( qe, qe, q2 );
	qf16_normalize_to_unit( qe, qe );

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	qf16_to_v3d( we, qe );
	v3d_mul_s( we, we, fix16_mul( _fc_2, fix16_sign_no_zero( qe->a ) ) );
}

void qf16_i_rotate( v3d* dest, const qf16* q, const v3d* v ) {
	//Could be done much more efficiently by doing the reverse operation instead
	qf16 qi;
	qf16_inverse( &qi, q );
	qf16_rotate( dest, &qi, v );
}

v3d v3d_enu_to_ned( const v3d* enu ) {
	v3d ned;
	qf16_rotate( &ned, &NED_ENU_Q, enu );
	return ned;
}

v3d v3d_ned_to_enu( const v3d* ned ) {
	v3d enu;
	qf16_rotate( &enu, &NED_ENU_Q, ned );
	return enu;
}

#ifdef __cplusplus
}
#endif
