#ifdef __cplusplus
extern "C" {
#endif

//#include <stdlib.h>
#include <stdbool.h>
//#include <math.h>

#include "breezystm32.h"
#include "sensors.h"
#include "params.h"
#include "estimator.h"

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"
#include "fixextra.h"

state_t _state_estimator;
v3d _adaptive_gyro_bias;
sensor_readings_t _sensors;

static v3d g;	//Gravity vector

static v3d w1;
static v3d w2;
static v3d wbar;
static v3d wfinal;
static v3d w_acc;
static v3d b;
static qf16 q_tilde;
static qf16 q_hat;
static uint32_t time_last;

static fix16_t kp_;
static fix16_t ki_;
static uint32_t init_time;

static v3d _accel_LPF;
static v3d _gyro_LPF;

void estimator_init( void ) {
	_state_estimator.p = 0;
	_state_estimator.q = 0;
	_state_estimator.r = 0;
	_state_estimator.attitude.a = 1;
	_state_estimator.attitude.b = 0;
	_state_estimator.attitude.c = 0;
	_state_estimator.attitude.d = 0;

	q_hat.a = _fc_1;
	q_hat.b = 0;
	q_hat.c = 0;
	q_hat.d = 0;

	q_tilde.a = _fc_1;
	q_tilde.b = 0;
	q_tilde.c = 0;
	q_tilde.d = 0;

	g.x = 0;
	g.y = 0;
	g.z = _fc_1;

	w1.x = 0;
	w1.y = 0;
	w1.z = 0;

	w2.x = 0;
	w2.y = 0;
	w2.z = 0;

	w_acc.x = 0;
	w_acc.y = 0;
	w_acc.z = 0;

	b.x = 0;
	b.y = 0;
	b.z = 0;

	_adaptive_gyro_bias.x = 0;
	_adaptive_gyro_bias.y = 0;
	_adaptive_gyro_bias.z = 0;

	_accel_LPF.x = 0;
	_accel_LPF.y = 0;
	_accel_LPF.z = _fc_gravity;

	_gyro_LPF.x = 0;
	_gyro_LPF.y = 0;
	_gyro_LPF.z = 0;

	kp_ = get_param_fix16( PARAM_FILTER_KP );
	ki_ = get_param_fix16( PARAM_FILTER_KI );
	init_time = get_param_uint( PARAM_INIT_TIME ) * 1000;	//nano->microseconds

	time_last = 0;
}


void reset_adaptive_gyro_bias() {
	b.x = 0;
	b.y = 0;
	b.z = 0;
}

static void lpf_update(void) {
	//value_lpf = ((1 - alpha) * value) + (alpha * value_lpf);
	fix16_t alpha_acc = get_param_fix16(PARAM_ACC_ALPHA);
	_accel_LPF.x = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_acc), _sensors.imu.accel.x), fix16_smul(alpha_acc, _accel_LPF.x));
	_accel_LPF.y = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_acc), _sensors.imu.accel.y), fix16_smul(alpha_acc, _accel_LPF.y));;
	_accel_LPF.z = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_acc), _sensors.imu.accel.z), fix16_smul(alpha_acc, _accel_LPF.z));;

	fix16_t alpha_gyro = get_param_fix16(PARAM_GYRO_ALPHA);
	_gyro_LPF.x = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_gyro), _sensors.imu.gyro.x), fix16_smul(alpha_gyro, _gyro_LPF.x));
	_gyro_LPF.y = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_gyro), _sensors.imu.gyro.y), fix16_smul(alpha_gyro, _gyro_LPF.y));
	_gyro_LPF.z = fix16_sadd(fix16_smul(fix16_ssub(_fc_1, alpha_gyro), _sensors.imu.gyro.z), fix16_smul(alpha_gyro, _gyro_LPF.z));
}

void estimator_update( uint32_t time_now ) {
	fix16_t kp;
	fix16_t ki;

	//XXX: This will exit on the first loop, not a nice way of doing it though
	if ( time_last == 0 ) {
		time_last = time_now;
		return;
	}

	//Converts dt from micros to secs
	fix16_t dt = fix16_sdiv( ( time_now - time_last ), 1e6f );
	time_last = time_now;

	//Crank up the gains for the first few seconds for quick convergence
	if ( time_now < init_time ) {
		kp = fix16_smul( kp_, _fc_10 );
		ki = fix16_smul( ki_, _fc_10 );
	} else {
		kp = kp_;
		ki = ki_;
	}

	//Run LPF to reject a lot of noise
	lpf_update();

	//Add in accelerometer
	//a_sqrd_norm = x^2 + y^2 + z^2
	fix16_t a_sqrd_norm = v3d_sq_norm( &_accel_LPF );

	//If we should use accelerometer compensation, and the reading is reasonably small
	if( ( get_param_uint( PARAM_EST_USE_ACC_COR ) ) &&
	  (a_sqrd_norm < fix16_mul( fix16_sq( _fc_1_15 ), fix16_sq( _fc_gravity ) ) ) &&
	  (a_sqrd_norm > fix16_mul( fix16_sq( _fc_0_85 ), fix16_sq( _fc_gravity ) ) ) ) {

		// Get error estimated by accelerometer measurement
		v3d a;
		v3d_normalize(&a, &_accel_LPF);

		// Get the quaternion from accelerometer (low-frequency measure q)
		// (Not in either paper)
		// Rotation from accel reading to gravity
		qf16 q_acc;
		qf16 q_acc_inv;
		qf16_from_shortest_path(&q_acc, &a, &g);
		qf16_inverse(&q_acc_inv, &q_acc);

		// Get the error quaternion between observer and low-freq q
		// First we need to take out the Z (Yaw) component as it is unobservable with just imu
		qf16 q_hat_acc;
		mf16 rot_mat;
		qf16_to_matrix(&rot_mat, &q_hat);

		v3d yaw_c;
		v3d body_x;
		v3d body_y;
		v3d body_z;
		yaw_c.x = 0;
		yaw_c.y = _fc_1;	//TODO: This should be aligned with NED and Compass North: yaw_c = [-sin(yaw), cos(yaw), 0.0f] XXX: MAYBE!;
		yaw_c.z = 0;
		body_z.x = rot_mat.data[2][0];
		body_z.y = rot_mat.data[2][1];
		body_z.z = rot_mat.data[2][2];

		v3d_cross(&body_x, &yaw_c, &body_z);

		//Invert the frame if the FCU is upside down
		//if (body_z.z < 0) {
		//		body_x.x = -body_x.x;
		//		body_x.y = -body_x.y;
		//		body_x.z = -body_x.z;
		//}

		v3d_normalize(&body_x, &body_x);

		v3d_cross(&body_y, &body_z, &body_x);
		v3d_normalize(&body_y, &body_y);

		rot_mat.data[0][0] = body_x.x;
		rot_mat.data[0][1] = body_x.y;
		rot_mat.data[0][2] = body_x.z;
		rot_mat.data[1][0] = body_y.x;
		rot_mat.data[1][1] = body_y.y;
		rot_mat.data[1][2] = body_y.z;
		rot_mat.data[2][0] = body_z.x;
		rot_mat.data[2][1] = body_z.y;
		rot_mat.data[2][2] = body_z.z;

		matrix_to_qf16(&q_hat_acc, &rot_mat );
		qf16_normalize(&q_hat_acc, &q_hat_acc);

		// Below Eq. 45 Mahoney Paper
		qf16_mul(&q_tilde, &q_acc_inv, &q_hat_acc);

		// Correction Term of Eq. 47a and 47b Mahoney Paper
		// w_acc = -2*s_tilde*v_tilde
		w_acc.x = fix16_mul(-_fc_2, fix16_mul(q_tilde.a, q_tilde.b));
		w_acc.y = fix16_mul(-_fc_2, fix16_mul(q_tilde.a, q_tilde.c));
		w_acc.z = fix16_mul(-_fc_2, fix16_mul(q_tilde.a, q_tilde.d));

		// integrate biases from accelerometer feedback
		// (eq 47b Mahoney Paper, using correction term w_acc found above)
		b.x = fix16_sub(b.x, fix16_mul(ki, fix16_mul(w_acc.x, dt)));
		b.y = fix16_sub(b.y, fix16_mul(ki, fix16_mul(w_acc.y, dt)));
		b.z = fix16_sub(b.z, fix16_mul(ki, fix16_mul(w_acc.z, dt)));
	} else {
		w_acc.x = 0;
		w_acc.y = 0;
		w_acc.z = 0;
	}

	// Pull out Gyro measurements
	if( get_param_uint( PARAM_EST_USE_QUAD_INT ) ) {
		// Quadratic Integration (Eq. 14 Casey Paper)
		// this integration step adds 12 us on the STM32F10x chips
		//wbar = ((-1.0f / 12.0f) * w2) + ((8.0f / 12.0f) * w1) + ((5.0f / 12.0f) * _gyro_LPF);

		v3d w1_temp;
		v3d w2_temp;
		v3d w_sum_temp;
		v3d gyro_temp;

		v3d_mul_s( &w1_temp, &w1, _fc_0_6_ );
		v3d_mul_s( &w2_temp, &w2, -_fc_0_083_ );

		v3d_add( &w_sum_temp, &w1_temp, &w2_temp );

		v3d_mul_s( &gyro_temp, &_gyro_LPF, _fc_0_46_ );

		v3d_add( &wbar, &w_sum_temp, &gyro_temp );

		w2 = w1;
		w1 = _gyro_LPF;
	} else {
		wbar = _gyro_LPF;
	}

	// Build the composite omega vector for kinematic propagation
	// This is the stuff inside the p function in eq. 47a - Mahoney Paper
	//wfinal = (wbar - b) + (kp * w_acc);
	v3d wfinal_temp_scale;
	v3d wfinal_temp_sub;

	v3d_mul_s( &wfinal_temp_scale, &w_acc, kp );
	v3d_sub( &wfinal_temp_sub, &wbar, &b );
	v3d_add( &wfinal, &wfinal_temp_sub, &wfinal_temp_scale );

	//Propagate Dynamics (only if we've moved)
	fix16_t sqrd_norm_w = v3d_sq_norm( &wfinal );

	if( sqrd_norm_w > 0 ) {
		fix16_t p = wfinal.x;	//Roll Rate
		fix16_t q = wfinal.y;	//Pitch Rate
		fix16_t r = wfinal.z;	//Yaw Rate

		qf16 q_hat_temp;
		qf16 qdot;

		if( get_param_uint( PARAM_EST_USE_MAT_EXP ) ) {
			// Matrix Exponential Approximation (From Attitude Representation and Kinematic
			// Propagation for Low-Cost UAVs by Robert T. Casey)
			// (Eq. 12 Casey Paper)
			// This adds 90 us on STM32F10x chips
			fix16_t norm_w = fix16_sqrt( sqrd_norm_w );

			//This is can cause some serious RAM issues if either caching or lookup tables are enabled
			//XXX: Even with caching turned off, this should give a good performance increase (hopefully around 25%)
			fix16_t t1 = fix16_cos(fix16_div(fix16_mul(norm_w, dt), _fc_2));
			fix16_t t2 = fix16_mul(fix16_div(_fc_1, norm_w), fix16_sin(fix16_div(fix16_mul(norm_w, dt), _fc_2)));

			/*
			qhat_np1.w = t1*q_hat.w   + t2*(          - p*q_hat.x - q*q_hat.y - r*q_hat.z);
			qhat_np1.x = t1*q_hat.x   + t2*(p*q_hat.w             + r*q_hat.y - q*q_hat.z);
			qhat_np1.y = t1*q_hat.y   + t2*(q*q_hat.w - r*q_hat.x             + p*q_hat.z);
			qhat_np1.z = t1*q_hat.z   + t2*(r*q_hat.w + q*q_hat.x - p*q_hat.y);
			*/

			//qdot.w = t2*(((-p*q_hat.x) + (-q*q_hat.y)) + (-r*q_hat.z)
			qdot.a = fix16_mul(t2, fix16_add(fix16_add(fix16_mul(-p, q_hat.b), fix16_mul(-q, q_hat.c)), fix16_mul(-r, q_hat.d)));
			qdot.b = fix16_mul(t2, fix16_add(fix16_add(fix16_mul(p, q_hat.a), fix16_mul(r, q_hat.c)), fix16_mul(-q, q_hat.d)));
			qdot.c = fix16_mul(t2, fix16_add(fix16_add(fix16_mul(q, q_hat.a), fix16_mul(-r, q_hat.b)), fix16_mul(p, q_hat.d)));
			qdot.d = fix16_mul(t2, fix16_add(fix16_add(fix16_mul(r, q_hat.a), fix16_mul(q, q_hat.b)), fix16_mul(-p, q_hat.c)));

			//qhat_np1.w = (t1*q_hat.w) + qdot.w);
			q_hat_temp.a = fix16_add(fix16_mul(t1, q_hat.a), qdot.a);
			q_hat_temp.b = fix16_add(fix16_mul(t1, q_hat.b), qdot.b);
			q_hat_temp.c = fix16_add(fix16_mul(t1, q_hat.c), qdot.c);
			q_hat_temp.d = fix16_add(fix16_mul(t1, q_hat.d), qdot.d);

			qf16_normalize(&q_hat, &q_hat_temp);
		} else {
			// Euler Integration
			// (Eq. 47a Mahoney Paper), but this is pretty straight-forward
			/*
			quaternion_t qdot = {0.5f * (           - p*q_hat.x - q*q_hat.y - r*q_hat.z),
							     0.5f * ( p*q_hat.w             + r*q_hat.y - q*q_hat.z),
							     0.5f * ( q*q_hat.w - r*q_hat.x             + p*q_hat.z),
							     0.5f * ( r*q_hat.w + q*q_hat.x - p*q_hat.y            )
							    };
			*/

			qdot.a = fix16_mul(_fc_0_5, fix16_add(fix16_mul(-p, q_hat.b), fix16_add(fix16_mul(-q, q_hat.c), fix16_mul(-r, q_hat.d))));
			qdot.b = fix16_mul(_fc_0_5, fix16_add(fix16_mul(p, q_hat.a), fix16_add(fix16_mul(r, q_hat.c), fix16_mul(-q, q_hat.d))));
			qdot.c = fix16_mul(_fc_0_5, fix16_add(fix16_mul(q, q_hat.a), fix16_add(fix16_mul(-r, q_hat.b), fix16_mul(p, q_hat.d))));
			qdot.d = fix16_mul(_fc_0_5, fix16_add(fix16_mul(r, q_hat.a), fix16_add(fix16_mul(q, q_hat.b), fix16_mul(-p, q_hat.c))));

			q_hat_temp.a = fix16_add(q_hat.a, fix16_mul(qdot.a, dt));
			q_hat_temp.b = fix16_add(q_hat.b, fix16_mul(qdot.b, dt));
			q_hat_temp.c = fix16_add(q_hat.c, fix16_mul(qdot.c, dt));
			q_hat_temp.d = fix16_add(q_hat.d, fix16_mul(qdot.d, dt));

			qf16_normalize(&q_hat, &q_hat_temp);
		}
	}

	//q_hat is given as z-down, rotate to NED
	_state_estimator.attitude = q_hat;
	// Extract Euler Angles for controller
	//euler_from_quat(&q_hat, &_state_estimator.phi, &_state_estimator.theta, &_state_estimator.psi);

	if( !get_param_uint( PARAM_EST_USE_ADPT_BIAS ) )
		reset_adaptive_gyro_bias();	//TODO: XXX: The adaptive bias' are good, but without proper mag support, they cause lasting errors if the mav is turned upside down

	// Save old adjust gyro measurements with estimated biases for control
	v3d wbar_old = wbar;
	v3d_sub( &wbar, &wbar_old, &b );

	_state_estimator.p = fix16_sub( _gyro_LPF.x, _adaptive_gyro_bias.x );
	_state_estimator.q = fix16_sub( _gyro_LPF.y, _adaptive_gyro_bias.y );
	_state_estimator.r = fix16_sub( _gyro_LPF.z, _adaptive_gyro_bias.z );

	// Save gyro biases for streaming to computer
	_adaptive_gyro_bias.x = b.x;
	_adaptive_gyro_bias.y = b.y;
	_adaptive_gyro_bias.z = b.z;
}

#ifdef __cplusplus
}
#endif

