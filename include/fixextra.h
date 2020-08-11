#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

//Quaternion layout
//q.a -> q.w
//q.b -> q.x
//q.c -> q.y
//q.d -> q.z

//Quick defines of reused fix16 numbers
/*
#define _fc_0_01 0x0000028F		//0.01
#define _fc_0_05 0x00000CCC		//0.05
#define _fc_0_083_ 0x00001555	//0.0833...
#define _fc_0_1 0x00001996		//0.1
#define _fc_0_2 0x0000332C		//0.2
#define _fc_0_46_ 0x00006AAA	//0.466...
#define _fc_0_5 0x00008000		//0.5
#define _fc_0_6_ 0x0000AAAA		//0.66...
#define _fc_0_85 0x0000D999		//0.85
#define _fc_0_95 0x0000F333		//0.95
#define _fc_1 0x00010000		//1.0
#define _fc_1_15 0x00012666		//1.15
#define _fc_1_41 0x00016A09		//1.4142... (sqrt 2)
#define _fc_2 0x00020000		//2.0
#define _fc_3_3 0x00034CCC		//3.3
#define _fc_10 0x000A0000		//10
#define _fc_11 0x000B0000		//11
#define _fc_20 0x00140000		//20
#define _fc_100 0x00640000		//100
#define _fc_1000 0x03E80000		//1000
*/
#define _fc_0_01	0x0000028F		//0.01
#define _fc_0_05	0x00000CCC		//0.05
#define _fc_0_083_	0x00001555		//0.0833...
#define _fc_0_1		0x00001999		//0.1
#define _fc_0_2		0x00003333		//0.2
#define _fc_0_46_	0x00007777		//0.466...
#define _fc_0_5		0x00008000		//0.5
#define _fc_0_6_	0x0000AAAA		//0.66...
#define _fc_0_85	0x0000D999		//0.85
#define _fc_0_95	0x0000F333		//0.95
#define _fc_1		0x00010000		//1.0
#define _fc_1_15	0x00012666		//1.15
#define _fc_1_41	0x00016A09		//1.4142... (sqrt 2)
#define _fc_2		0x00020000		//2.0
#define _fc_3_3		0x00034CCC		//3.30
#define _fc_10		0x000A0000		//10.0
#define _fc_11		0x000B0000		//11.0
#define _fc_20		0x00140000		//20.0
#define _fc_100		0x00640000		//100.0
#define _fc_1000	0x03E80000		//1000.0

#define _fc_sqrt_0_5 0x0000B504 //0.70710754395 (slightly higher than sqrt 0.5)
#define _fc_epsilon 0x0000FFFF  //0.99...
#define _fc_eps 0x00000001		//0.0...1 (actually is 0.00002)
#define _fc_gravity 0x0009CE80  //Is equal to 9.80665 (Positive!) in Q16.16

typedef enum {
	AXIS_LOCK_X,
	AXIS_LOCK_Y,
	AXIS_LOCK_Z
} qf16_axis_lock_t;

extern const qf16 NED_ENU_Q;
extern const v3d V3D_ZERO;
extern const qf16 QF16_NO_ROT;

fix16_t fix16_wrap_pi( const fix16_t x );
fix16_t v3d_sq_norm( const v3d* a );
fix16_t qf16_dot_full( const qf16* q1, const qf16* q2 );
void qf16_inverse( qf16* dest, const qf16* q );
void qf16_dcm_z( v3d* b_z, const qf16* q );
//Returns the rotation between two vectors
//Method used derrived from PX4/Matrix
void qf16_from_shortest_path( qf16* dest, const v3d* v1, const v3d* v2 );
//TODO: Should use the mavlink conversions as a base if we move to float
fix16_t heading_from_quat( qf16* q );
void euler_from_quat( qf16* q, fix16_t* phi, fix16_t* theta, fix16_t* psi );
void quat_from_euler( qf16* q, fix16_t phi, fix16_t theta, fix16_t psi );
void matrix_to_qf16( qf16* dest, const mf16* mat );
void dcm_to_basis( v3d* b_x, v3d* b_y, v3d* b_z, const mf16* dcm );
void dcm_from_basis( mf16* dcm, const v3d* b_x, const v3d* b_y, const v3d* b_z );
void qf16_to_dcm( mf16* dcm, const qf16* q );
void vee_up( mf16* W, const v3d* w );
void vee_down( v3d* w, const mf16* W );
void qf16_align_to_axis( qf16* dest, const qf16* input, const qf16* reference, const qf16_axis_lock_t axis );
//Calculates the basis error representing body the rate rotations required
//to rotate q1 to q2, in the frame of q1
//This could be used to calculate angular control error, with q1 being the
//state, and q2 being the reference
void qf16_basis_error( v3d* we, qf16* qe, const qf16* q1, const qf16* q2 );
void qf16_i_rotate( v3d* dest, const qf16* q, const v3d* v );
v3d v3d_enu_to_ned( const v3d* enu );
v3d v3d_ned_to_enu( const v3d* ned );

static inline fix16_t fix16_normalize( const fix16_t i, const fix16_t min, const fix16_t max ) {
	return fix16_div( fix16_ssub(i, min), fix16_ssub(max, min) );
}

static inline fix16_t fix16_normalize_clamp( const fix16_t i, const fix16_t min, const fix16_t max ) {
	//Limit the output to the min/max
	fix16_t ic = ( i < min ) ? min : ( i > max ) ? max : i;

	return fix16_normalize(ic, min, max);
}

static inline fix16_t fix16_constrain( const fix16_t i, const fix16_t min, const fix16_t max ) {
	return ( i < min ) ? min : ( i > max ) ? max : i;
}

static inline fix16_t fix16_sign( const fix16_t x ) {
	return ( x > 0 ) ? _fc_1 : ( ( x == 0 ) ? 0 : -_fc_1 );
}

static inline fix16_t fix16_sign_no_zero( const fix16_t x ) {
	return ( x >= 0 ) ? _fc_1 : -_fc_1;
}

//dest and a can alias
static inline void v3d_inv( v3d* dest, const v3d* a ) {
	dest->x = -a->x;
	dest->y = -a->y;
	dest->z = -a->z;
}

static inline fix16_t qf16_norm_full( const qf16* q ) {
	return fix16_sqrt( qf16_dot_full( q, q ) );
}

static inline void qf16_normalize_to_unit( qf16* dest, const qf16* q ) {
	qf16_div_s( dest, q, qf16_norm_full( q ) );
}

static inline void v3d_abs( v3d* dest, const v3d* v ) {
	dest->x = fix16_abs( v->x );
	dest->y = fix16_abs( v->y );
	dest->z = fix16_abs( v->z );
}
static inline void dcm_to_basis_x( v3d* b_x, const mf16* dcm ) {
	b_x->x = dcm->data[0][0];
	b_x->y = dcm->data[0][1];
	b_x->z = dcm->data[0][2];
}

static inline void dcm_to_basis_y( v3d* b_y, const mf16* dcm ) {
	b_y->x = dcm->data[1][0];
	b_y->y = dcm->data[1][1];
	b_y->z = dcm->data[1][2];
}

static inline void dcm_to_basis_z( v3d* b_z, const mf16* dcm ) {
	b_z->x = dcm->data[2][0];
	b_z->y = dcm->data[2][1];
	b_z->z = dcm->data[2][2];
}

#ifdef __cplusplus
}
#endif
