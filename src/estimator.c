#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "estimator.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"

#include "drivers/drv_system.h"
#include "fix16.h"
#include "fixextra.h"
#include "fixquat.h"
#include "fixvector3d.h"

state_t _state_estimator;
sensor_readings_t _sensors;


//static v3d w1;	 //Integrator for quad int
//static v3d w2;	 //Integrator for quad int
static v3d w_bias_;	  //Integrator for adaptive bias
static qf16 q_hat_; //Attitude estimate
static v3d g_; //Gravity vector

static uint32_t time_last_;
static uint32_t init_time_;

static v3d accel_lpf_;
static v3d gyro_lpf_;

void estimator_init( void ) {
	_state_estimator.ax = 0;
	_state_estimator.ay = 0;
	_state_estimator.az = 0;
	_state_estimator.p = 0;
	_state_estimator.q = 0;
	_state_estimator.r = 0;
	_state_estimator.attitude.a = _fc_1;
	_state_estimator.attitude.b = 0;
	_state_estimator.attitude.c = 0;
	_state_estimator.attitude.d = 0;

	q_hat_.a = _fc_1;
	q_hat_.b = 0;
	q_hat_.c = 0;
	q_hat_.d = 0;

	g_.x = 0;
	g_.y = 0;
	g_.z = _fc_1;
	/*
	w1.x = 0;
	w1.y = 0;
	w1.z = 0;

	w2.x = 0;
	w2.y = 0;
	w2.z = 0;
	*/
	w_bias_.x = 0;
	w_bias_.y = 0;
	w_bias_.z = 0;

	accel_lpf_.x = 0;
	accel_lpf_.y = 0;
	accel_lpf_.z = _fc_gravity;

	gyro_lpf_.x = 0;
	gyro_lpf_.y = 0;
	gyro_lpf_.z = 0;

	init_time_ = get_param_uint( PARAM_EST_INIT_TIME ) * 1000; //nano->microseconds

	time_last_ = 0;
}

void reset_adaptive_gyro_bias() {
	w_bias_.x = 0;
	w_bias_.y = 0;
	w_bias_.z = 0;
}

static void lpf_update( v3d* accel, v3d* gyro ) {
	//value_lpf = ((1 - alpha) * value) + (alpha * value_lpf);
	fix16_t alpha_acc = get_param_fix16( PARAM_ACC_ALPHA );
	accel_lpf_.x = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_acc ), accel_lpf_.x ), fix16_mul( alpha_acc, accel->x ) );
	accel_lpf_.y = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_acc ), accel_lpf_.y ), fix16_mul( alpha_acc, accel->y ) );
	accel_lpf_.z = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_acc ), accel_lpf_.z ), fix16_mul( alpha_acc, accel->z ) );

	fix16_t alpha_gyro = get_param_fix16( PARAM_GYRO_ALPHA );
	gyro_lpf_.x = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_gyro ), gyro_lpf_.x ), fix16_mul( alpha_gyro, gyro->x ) );
	gyro_lpf_.y = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_gyro ), gyro_lpf_.y ), fix16_mul( alpha_gyro, gyro->y ) );
	gyro_lpf_.z = fix16_add( fix16_mul( fix16_sub( _fc_1, alpha_gyro ), gyro_lpf_.z ), fix16_mul( alpha_gyro, gyro->z ) );
}

static void corr_heading_estimate( v3d* corr, const qf16* q, const v3d* m, const fix16_t alpha, const fix16_t mag_decl ) {
	/*
	qf16 q_temp;
	qf16 dq;

	qf16_inverse( &q_temp, q_est );
	qf16_mul( &dq, q_mes, &q_temp ); //Difference between current estimate and measured
	qf16_normalize_to_unit( &dq, &dq );

	v3d fv;
	fv.x = _fc_1;
	fv.y = 0;
	fv.z = 0;

	qf16_rotate( &fv, &dq, &fv );
	fv.z = 0;
	v3d_normalize( &fv, &fv ); //Rotated vector in the XY plane

	fix16_t rot_a = fix16_atan2( fv.y, fv.x );
	rot_a = fix16_mul( alpha, rot_a );
	v3d rot_axis;
	rot_axis.x = 0;
	rot_axis.y = 0;
	rot_axis.z = _fc_1;
	qf16_from_axis_angle( &q_temp, &rot_axis, rot_a ); //Create a rotation quaternion for the flat rotation around Z axis

	qf16_mul( q_est, &q_temp, q_est ); //Rotate the estimate
	*/


	v3d mw;	//World measurement
	qf16_rotate( &mw, q, m );
	fix16_t mocap_hdg_err = wrap_pi(fix16_sub(fix16_atan2(mw.y,mw.x), mag_decl));
	v3d yaw_c;
	yaw_c.x = 0;
	yaw_c.y = 0;
	yaw_c.z = -mocap_hdg_err;
	// Project correction to body frame
	qf16 qi;
	qf16_inverse(&qi,q);
	v3d err;
	qf16_rotate( &err, &qi, &yaw_c );
	v3d_mul_s(corr, &err, alpha);
}

//  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
static void estimator_update( uint32_t time_now, v3d* accel, v3d* gyro ) {
	//XXX: This will exit on the first loop, not a nice way of doing it though
	if ( time_last_ == 0 ) {
		time_last_ = time_now;
		return;
	}

	//Converts dt from micros to secs
	fix16_t dt = fix16_from_float( 1e-6 * (float)( time_now - time_last_ ) );
	time_last_ = time_now;

	//Run LPF to reject a lot of noise
	lpf_update( accel, gyro );

	//Accelerometer compensation
	//The reading must be reasonably close to something that resembles a hover
	v3d w_acc;
	w_acc.x = 0;
	w_acc.y = 0;
	w_acc.z = 0;
	fix16_t a_sqrd_norm = v3d_sq_norm( &accel_lpf_ );

	qf16 q_acc;
	if ( ( get_param_fix16( PARAM_EST_ACC_KP ) > 0 ) && ( a_sqrd_norm < fix16_mul( fix16_sq( _fc_1_15 ), fix16_sq( _fc_gravity ) ) ) && ( a_sqrd_norm > fix16_mul( fix16_sq( _fc_0_85 ), fix16_sq( _fc_gravity ) ) ) ) {
		//Gains for accelerometer compensation
		fix16_t kp = get_param_fix16( PARAM_EST_ACC_KP );
		fix16_t ki = get_param_fix16( PARAM_EST_BIAS_KP );
		//Crank up the gains for the first few seconds for quick convergence
		if ( time_now < init_time_ ) {
			kp = fix16_smul( kp, _fc_10 );
			ki = fix16_smul( ki, _fc_10 );
		}

		//Determine the rotation estimate as read by the accelerometer
		qf16_from_shortest_path( &q_acc, &g_, &accel_lpf_ );

		//Determine the yaw-free rotation of the current attitude estimate
		v3d body_z;
		qf16 q_hat_inv;
		qf16_inverse( &q_hat_inv, &q_hat_ ); //Need to invert to bring the vector into the body frame
		qf16_dcm_z( &body_z, &q_hat_inv );
		//qf16_rotate(&body_z, &q_hat_, &g_);

		qf16 q_hat_acc;
		qf16_from_shortest_path( &q_hat_acc, &g_, &body_z );

		//Use the basis error method to calculate body-fixed rotation error
		v3d e_R;
		qf16_basis_error( &e_R, &q_hat_acc, &q_acc );

		// Calculate correction rates and integrate biases from accelerometer feedback
		v3d_mul_s( &w_acc, &e_R, kp );

		//b.x = fix16_add( b.x, fix16_mul( ki, fix16_mul( e_R.x, dt ) ) );
		//b.y = fix16_add( b.y, fix16_mul( ki, fix16_mul( e_R.y, dt ) ) );
		//b.z = fix16_add( b.z, fix16_mul( ki, fix16_mul( e_R.z, dt ) ) );

		/*
		// Accelerometer correction
		v3d b_z;
		qf16_dcm_z( &b_z, &q_hat_ );
		//v3d_normalize( &b_z, &b_z );

		v3d a_z;
		v3d_normalize( &a_z, &accel_lpf_ );

		v3d e_R;
		v3d_cross(&e_R, &b_z, &a_z);
		v3d_mul_s(&w_acc, &e_R, kp);
		*/
	}

	/*
	// Pull out Gyro measurements
	if ( get_param_uint( PARAM_EST_USE_QUAD_INT ) ) {
		// Quadratic Integration (Eq. 14 Casey Paper)
		// this integration step adds 12 us on the STM32F10x chips
		//wbar = ((-1.0f / 12.0f) * w2) + ((8.0f / 12.0f) * w1) + ((5.0f / 12.0f) * gyro_lpf_);

		v3d w1_temp;
		v3d w2_temp;
		v3d w_sum_temp;
		v3d gyro_temp;

		v3d_mul_s( &w1_temp, &w1, _fc_0_6_ );
		v3d_mul_s( &w2_temp, &w2, -_fc_0_083_ );

		v3d_add( &w_sum_temp, &w1_temp, &w2_temp );

		v3d_mul_s( &gyro_temp, &gyro_lpf_, _fc_0_46_ );

		v3d_add( &wbar, &w_sum_temp, &gyro_temp );

		w2 = w1;
		w1 = gyro_lpf_;
	}
	*/

	//Heading correction
	v3d m_mag;	//TODO: From mag sensor
	v3d w_mag;
	w_mag.x = 0;
	w_mag.y = 0;
	w_mag.z = 0;

	v3d north;	//XXX: remove
	north.x = _fc_1;
	qf16_rotate(&m_mag, &q_acc, &north);

	//corr_heading_estimate( &w_mag, &q_hat_, &m_mag, get_param_fix16( PARAM_FUSE_MAG_HDG_W ), 0 );	//TODO: declination

	// Build the composite omega vector for kinematic propagation
	// This is the stuff inside the p function in eq. 47a - Mahoney Paper
	//wbar = w + w_acc + w_bias_;
	//XXX:v3d wbar = gyro_lpf_;
	v3d wbar;
	v3d_add( &wbar, &w_mag, &w_acc );
	//v3d_add( &wbar, &gyro_lpf_, &w_acc );
	//v3d_add( &wbar, &wbar, &w_bias_ );

	/*
	//Propagate Dynamics (only if we've moved)
	fix16_t sqrd_norm_w = v3d_sq_norm( &wfinal );

	if ( sqrd_norm_w > 0 ) {
		fix16_t p = wfinal.x; //Roll Rate
		fix16_t q = wfinal.y; //Pitch Rate
		fix16_t r = wfinal.z; //Yaw Rate

		qf16 q_hat__temp;
		qf16 qdot;

		if ( get_param_uint( PARAM_EST_USE_MAT_EXP ) ) {
			// Matrix Exponential Approximation (From Attitude Representation and Kinematic
			// Propagation for Low-Cost UAVs by Robert T. Casey)
			// (Eq. 12 Casey Paper)
			// This adds 90 us on STM32F10x chips
			fix16_t norm_w = fix16_sqrt( sqrd_norm_w );

			//This is can cause some serious RAM issues if either caching or lookup tables are enabled
			//XXX: Even with caching turned off, this should give a good performance increase (hopefully around 25%)
			fix16_t t1 = fix16_cos( fix16_div( fix16_mul( norm_w, dt ), _fc_2 ) );
			fix16_t t2 = fix16_mul( fix16_div( _fc_1, norm_w ), fix16_sin( fix16_div( fix16_mul( norm_w, dt ), _fc_2 ) ) );

			//qhat_np1.w = t1*q_hat_.w   + t2*(          - p*q_hat_.x - q*q_hat_.y - r*q_hat_.z);
			//qhat_np1.x = t1*q_hat_.x   + t2*(p*q_hat_.w             + r*q_hat_.y - q*q_hat_.z);
			//qhat_np1.y = t1*q_hat_.y   + t2*(q*q_hat_.w - r*q_hat_.x             + p*q_hat_.z);
			//qhat_np1.z = t1*q_hat_.z   + t2*(r*q_hat_.w + q*q_hat_.x - p*q_hat_.y);


			//qdot.w = t2*(((-p*q_hat_.x) + (-q*q_hat_.y)) + (-r*q_hat_.z)
			qdot.a = fix16_mul( t2, fix16_add( fix16_add( fix16_mul( -p, q_hat_.b ), fix16_mul( -q, q_hat_.c ) ), fix16_mul( -r, q_hat_.d ) ) );
			qdot.b = fix16_mul( t2, fix16_add( fix16_add( fix16_mul( p, q_hat_.a ), fix16_mul( r, q_hat_.c ) ), fix16_mul( -q, q_hat_.d ) ) );
			qdot.c = fix16_mul( t2, fix16_add( fix16_add( fix16_mul( q, q_hat_.a ), fix16_mul( -r, q_hat_.b ) ), fix16_mul( p, q_hat_.d ) ) );
			qdot.d = fix16_mul( t2, fix16_add( fix16_add( fix16_mul( r, q_hat_.a ), fix16_mul( q, q_hat_.b ) ), fix16_mul( -p, q_hat_.c ) ) );

			//qhat_np1.w = (t1*q_hat_.w) + qdot.w);
			q_hat__temp.a = fix16_add( fix16_mul( t1, q_hat_.a ), qdot.a );
			q_hat__temp.b = fix16_add( fix16_mul( t1, q_hat_.b ), qdot.b );
			q_hat__temp.c = fix16_add( fix16_mul( t1, q_hat_.c ), qdot.c );
			q_hat__temp.d = fix16_add( fix16_mul( t1, q_hat_.d ), qdot.d );

			qf16_normalize_to_unit( &q_hat_, &q_hat__temp );
		} else {
			// Euler Integration
			// (Eq. 47a Mahoney Paper), but this is pretty straight-forward

			//quaternion_t qdot = {0.5f * (           - p*q_hat_.x - q*q_hat_.y - r*q_hat_.z),
			//				     0.5f * ( p*q_hat_.w             + r*q_hat_.y - q*q_hat_.z),
			//				     0.5f * ( q*q_hat_.w - r*q_hat_.x             + p*q_hat_.z),
			//				     0.5f * ( r*q_hat_.w + q*q_hat_.x - p*q_hat_.y            )
			//				    };


			qdot.a = fix16_mul( _fc_0_5, fix16_add( fix16_mul( -p, q_hat_.b ), fix16_add( fix16_mul( -q, q_hat_.c ), fix16_mul( -r, q_hat_.d ) ) ) );
			qdot.b = fix16_mul( _fc_0_5, fix16_add( fix16_mul( p, q_hat_.a ), fix16_add( fix16_mul( r, q_hat_.c ), fix16_mul( -q, q_hat_.d ) ) ) );
			qdot.c = fix16_mul( _fc_0_5, fix16_add( fix16_mul( q, q_hat_.a ), fix16_add( fix16_mul( -r, q_hat_.b ), fix16_mul( p, q_hat_.d ) ) ) );
			qdot.d = fix16_mul( _fc_0_5, fix16_add( fix16_mul( r, q_hat_.a ), fix16_add( fix16_mul( q, q_hat_.b ), fix16_mul( -p, q_hat_.c ) ) ) );

			q_hat__temp.a = fix16_add( q_hat_.a, fix16_mul( qdot.a, dt ) );
			q_hat__temp.b = fix16_add( q_hat_.b, fix16_mul( qdot.b, dt ) );
			q_hat__temp.c = fix16_add( q_hat_.c, fix16_mul( qdot.c, dt ) );
			q_hat__temp.d = fix16_add( q_hat_.d, fix16_mul( qdot.d, dt ) );

			qf16_normalize_to_unit( &q_hat_, &q_hat__temp );
		}
	}
	*/
	/*
	mf16 R_hat;
	qf16_to_dcm( &R_hat, &q_hat_ );

	mf16 Wfinal;
	vee_up( &Wfinal, &wfinal );

	mf16 R_hat_temp;
	mf16_mul( &R_hat_temp, &R_hat, &Wfinal );

	matrix_to_qf16( &q_hat_, &R_hat_temp );
	qf16_normalize_to_unit( &q_hat_, &q_hat_ );
	*/

	//q_hat += q_hat.derivative1(wbar) * dt;

	qf16 q_w;	//q_hat.derivative1(wbar)
	qf16_from_v3d(&q_w, &wbar, 0);
	qf16_mul(&q_w,&q_hat_,&q_w);
	qf16_mul_s(&q_w,&q_w,_fc_0_5);

	qf16_mul_s(&q_w, &q_w, dt);

	qf16_add(&q_hat_, &q_hat_, &q_w);

	qf16_normalize_to_unit(&q_hat_, &q_hat_);

	//==-- Heading data fusion
	/*
	if ( ( _system_status.sensors.ext_pose.health == SYSTEM_HEALTH_OK ) && _sensors.ext_pose.status.new_data ) {
		fuse_heading_estimate( &q_hat_, &_sensors.ext_pose.q, get_param_fix16( PARAM_FUSE_EXT_HDG_W ) );
		_sensors.ext_pose.status.new_data = false;
	} else if ( ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) && _sensors.mag.status.new_data ) {
		fuse_heading_estimate( &q_hat_, &_sensors.mag.q, get_param_fix16( PARAM_FUSE_MAG_HDG_W ) );
		_sensors.mag.status.new_data = false;
	}
	*/

	//q_hat_ is given as z-down (NED)

	//Perform level horizon measurements
	if ( get_param_uint( PARAM_EST_USE_LEVEL_HORIZON ) ) {
		qf16 q_lh;
		qf16 q_lh_inv;
		qf16 q_hat__lvl;
		q_lh.a = get_param_fix16( PARAM_EST_LEVEL_HORIZON_W );
		q_lh.b = get_param_fix16( PARAM_EST_LEVEL_HORIZON_X );
		q_lh.c = get_param_fix16( PARAM_EST_LEVEL_HORIZON_Y );
		q_lh.d = get_param_fix16( PARAM_EST_LEVEL_HORIZON_Z );
		qf16_normalize_to_unit( &q_lh, &q_lh );
		qf16_inverse( &q_lh_inv, &q_lh );

		qf16_mul( &q_hat__lvl, &q_hat_, &q_lh_inv );
		qf16_normalize_to_unit( &_state_estimator.attitude, &q_hat__lvl );
	} else {
		_state_estimator.attitude = q_hat_;
	}

	// Extract Euler Angles for controller
	//euler_from_quat(&q_hat_, &_state_estimator.phi, &_state_estimator.theta, &_state_estimator.psi);

	if ( !get_param_uint( PARAM_EST_USE_ADPT_BIAS ) )
		reset_adaptive_gyro_bias(); //TODO: XXX: The adaptive bias' are good, but without proper mag support, they cause lasting errors if the mav is turned upside down

	_state_estimator.p = fix16_add( gyro_lpf_.x, w_bias_.x );
	_state_estimator.q = fix16_add( gyro_lpf_.y, w_bias_.y );
	_state_estimator.r = fix16_add( gyro_lpf_.z, w_bias_.z );

	_state_estimator.ax = accel_lpf_.x;
	_state_estimator.ay = accel_lpf_.y;
	_state_estimator.az = accel_lpf_.z;
}

void estimator_update_sensors( uint32_t now ) {
	estimator_update( now, &_sensors.imu.accel, &_sensors.imu.gyro );

	_sensors.imu.status.new_data = false;
}

void estimator_update_hil( uint32_t now ) {
	estimator_update( now, &_sensors.hil.accel, &_sensors.hil.gyro );

	_sensors.hil.status.new_data = false;
}

void estimator_calc_lvl_horz( qf16* qlh ) {
	mf16 ratt;
	mf16 rlh;
	v3d bx;
	v3d by;
	v3d bz;
	v3d lhx;
	v3d lhy;
	v3d lhz;
	//Get DCM of current attitude estimate and extract axes
	qf16_to_matrix( &ratt, &q_hat_ );
	dcm_to_basis( &bx, &by, &bz, &ratt );

	//Build the new rotation matrix with pure roll/pitch
	lhy.x = 0;
	lhy.y = _fc_1;
	lhy.z = 0;
	lhz = bz;

	v3d_cross( &lhx, &lhy, &lhz );
	v3d_normalize( &lhx, &lhx );

	v3d_cross( &lhy, &lhz, &lhx );
	v3d_normalize( &lhy, &lhy );

	dcm_from_basis( &rlh, &lhx, &lhy, &lhz );

	matrix_to_qf16( qlh, &rlh );
	qf16_normalize_to_unit( qlh, qlh );
}

#ifdef __cplusplus
}
#endif
