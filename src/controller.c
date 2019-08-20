#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"
#include "fixextra.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "mavlink_system.h"

#include "controller.h"
#include "estimator.h"
#include "mixer.h"
#include "params.h"
#include "pid_controller.h"
#include "safety.h"
#include "sensors.h"

#include "drivers/drv_system.h"

system_status_t _system_status;
sensor_readings_t _sensors;
state_t _state_estimator;

control_timing_t _control_timing;
command_input_t _cmd_ob_input;
command_input_t _control_input;
control_output_t _control_output;

pid_controller_t _pid_roll_rate;
pid_controller_t _pid_pitch_rate;
pid_controller_t _pid_yaw_rate;

v3d integrator_;

static void controller_reset( void ) {
	pid_reset( &_pid_roll_rate, _state_estimator.p );
	pid_reset( &_pid_pitch_rate, _state_estimator.q );
	pid_reset( &_pid_yaw_rate, _state_estimator.r );

	integrator_.x = 0;
	integrator_.y = 0;
	integrator_.z = 0;

	_control_input.r = 0;
	_control_input.p = 0;
	_control_input.y = 0;
	_control_input.q.a = _fc_1;
	_control_input.q.b = 0;
	_control_input.q.c = 0;
	_control_input.q.d = 0;
	_control_input.T = 0;
	_control_input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

	_control_output.r = 0;
	_control_output.p = 0;
	_control_output.y = 0;
	_control_output.T = 0;
}

static void controller_set_input_failsafe( command_input_t* input ) {
	input->r = 0;
	input->p = 0;
	input->y = 0;
	input->q.a = _fc_1;
	input->q.b = 0;
	input->q.c = 0;
	input->q.d = 0;
	input->T = get_param_fix16( PARAM_FAILSAFE_THROTTLE );
	input->input_mask |= CMD_IN_IGNORE_ATTITUDE; // Set it to just hold rpy rates
												 // (as this skips unnessessary
												 // computing during boot, and is
												 // possibly safer)
}

// Technique adapted from the Pixhawk multirotor control scheme (~December 2018)
static void rot_error_from_attitude( v3d* e_R, qf16* qe, const qf16* q_sp, const qf16* q,
								 const fix16_t yaw_w ) {


	/*
	//XXX: Shorthand method that doesn't allow for separate yaw tuning
	v3d e_R;
	qf16_basis_error(&e_R, q, q_sp);
	v3d_mul_s( rates, &e_R, get_param_fix16(PARAM_MC_ANGLE_P) );
	*/

	v3d e_z;
	v3d e_z_d;
	qf16_dcm_z( &e_z, q );
	qf16_dcm_z( &e_z_d, q_sp );

	qf16 qd_red;
	qf16_from_shortest_path( &qd_red, &e_z, &e_z_d );
	//qf16_normalize_to_unit( &qd_red, &qd_red );

	// Handle co-linear vectoring cases
	if ( ( fix16_abs( qd_red.b ) >= _fc_epsilon ) || ( fix16_abs( qd_red.c ) >= _fc_epsilon ) ) {
		// They are in opposite directions which presents an ambiguous solution
		// The best we can momenterily is to just accept bad weightings from the
		// mixing and
		// do the 'best' movement possible for 1 time step until things can be
		// calculated
		qd_red = *q_sp;
	} else {
		// Transform rotation from current to desired thrust vector into a world
		// frame reduced desired attitude reference
		qf16_mul( &qd_red, &qd_red, q );
	}

	//XXX: Could take a shortcut if yaw_w == 0 (for manual control)
	//qf16 qd;
	//if( yaw_w > 0 ) {
	qf16_normalize_to_unit( &qd_red, &qd_red );

	// mix full and reduced desired attitude
	qf16 q_mix;
	qf16 qd_red_i;

	qf16_inverse( &qd_red_i, &qd_red );
	qf16_normalize_to_unit( &qd_red_i, &qd_red_i );

	qf16_mul( &q_mix, &qd_red_i, q_sp );
	qf16_normalize_to_unit( &q_mix, &q_mix );
	qf16_mul_s( &q_mix, &q_mix, fix16_sign_no_zero( q_mix.a ) );
	// catch numerical problems with the domain of acosf and asinf
	fix16_t q_mix_w = fix16_constrain( q_mix.a, -_fc_1, _fc_1 );
	fix16_t q_mix_z = fix16_constrain( q_mix.d, -_fc_1, _fc_1 );
	q_mix.a = fix16_cos( fix16_mul( yaw_w, fix16_acos( q_mix_w ) ) );
	q_mix.b = 0;
	q_mix.c = 0;
	q_mix.d = fix16_sin( fix16_mul( yaw_w, fix16_asin( q_mix_z ) ) );
	qf16_normalize_to_unit( &q_mix, &q_mix );

	qf16 qd;
	qf16_mul( &qd, &qd_red, &q_mix );
	qf16_normalize_to_unit( &qd, &qd );
	//} else {
	//	//XXX: If yaw_w is 0.0 (e.g. no z-axis rot), then shortcut straight to qd
	//	qf16_normalize_to_unit( &qd, &qd_red );
	//}

	qf16_basis_error( e_R, qe, q, &qd );
}

/*
static void calc_tracking_rate_error(v3d* we, const v3d* w, const qf16* qe, const v3d* w_sp) {
	v3d w_sp_b;
	qf16_rotate(&w_sp_b, qe, w_sp);

	v3d_sub(we, &w_sp_b, w);
}
*/

static void controller_run( uint32_t time_now ) {
	// Variables that store the computed attitude goal rates

	command_input_t input;
	controller_set_input_failsafe( &input ); // Failsafe input to be safe

	// Handle controller input
	if ( _system_status.state != MAV_STATE_CRITICAL ) {
		//_system_status.mode |= MAV_MODE_FLAG_MANUAL_ENABLED
		switch ( _system_status.control_mode ) {
		case MAIN_MODE_OFFBOARD: {
			// Offboard mode
			input = _cmd_ob_input;

			break;
		}
		case MAIN_MODE_STABILIZED: {
			// Manual stabilize mode
			//XXX: Currently this method (just this section) takes ~300ms

			// Roll/pitch angle and yaw rate
			input.input_mask = 0;
			input.input_mask |= CMD_IN_IGNORE_ROLL_RATE;
			input.input_mask |= CMD_IN_IGNORE_PITCH_RATE;

			// Generate the desired thrust vector
			v3d stab_z;
			stab_z.x = fix16_sin( fix16_mul( _sensors.rc_input.c_p, get_param_fix16( PARAM_MAX_PITCH_ANGLE ) ) );
			stab_z.y = -fix16_sin( fix16_mul( _sensors.rc_input.c_r, get_param_fix16( PARAM_MAX_ROLL_ANGLE ) ) );
			stab_z.z = fix16_sqrt( fix16_sub(_fc_1, fix16_add( fix16_sq(stab_z.x), fix16_sq(stab_z.y) ) ) );

			//Quaternion from vertical
			v3d unit_z;
			unit_z.x = 0;
			unit_z.y = 0;
			unit_z.z = _fc_1;
			qf16 q_stab_b;
			qf16_from_shortest_path(&q_stab_b, &unit_z, &stab_z);

			//Heading rotation for body->inertial
			qf16 q_rot_base;
			qf16_from_axis_angle( &q_rot_base, &unit_z, heading_from_quat( &_state_estimator.attitude ) );

			qf16_mul( &input.q, &q_rot_base, &q_stab_b );
			//XXX: Should already be close to normalised, but will be redone later before use

			input.q.a = _fc_1;

			input.r = 0;
			input.p = 0;
			input.y = fix16_mul( _sensors.rc_input.c_y, get_param_fix16( PARAM_MAX_YAW_RATE ) );

			input.T = _sensors.rc_input.c_T;

			break;
		}
		case MAIN_MODE_ACRO: {
			// Manual acro mode
			// Roll/pitch/yaw rate
			input.input_mask = 0;
			input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

			input.q.a = _fc_1;
			input.q.b = 0;
			input.q.c = 0;
			input.q.d = 0;

			input.r = fix16_mul( _sensors.rc_input.c_r,
								 get_param_fix16( PARAM_MAX_ROLL_RATE ) );
			input.p = fix16_mul( _sensors.rc_input.c_p,
								 get_param_fix16( PARAM_MAX_PITCH_RATE ) );
			input.y = fix16_mul( _sensors.rc_input.c_y, get_param_fix16( PARAM_MAX_YAW_RATE ) );

			input.T = _sensors.rc_input.c_T;

			break;
		}
		default: {
			// Keep failsafe
			break;
		}
		}
	}

	fix16_t dt = fix16_from_float( 1e-6 * (float)( time_now - _control_timing.time_last ) ); // Delta time in seconds

	if ( ( dt == 0 ) || ( time_now - _control_timing.time_last > _control_timing.period_stale ) ) {
		dt = 0;
		controller_reset();
	}

	// Get the control input mask to use
	_control_input.input_mask = input.input_mask;

	//==-- Throttle Control
	fix16_t goal_throttle = 0;

	// Trottle
	if ( !( _control_input.input_mask & CMD_IN_IGNORE_THROTTLE ) ) {
		// Use the commanded throttle
		goal_throttle = input.T;
	}

	_control_output.T = goal_throttle;

	// Save intermittent goals
	_control_input.T = goal_throttle;

	// Don't allow for derivative or integral calculations if
	// there is a very low throttle
	if ( goal_throttle < _fc_0_05 ) {
		dt = 0;
	}

	//==-- Attitude Control
	// Save intermittent goals
	qf16_normalize_to_unit( &_control_input.q, &input.q );
	v3d rates_ref = {0,0,0};

	// If we should listen to attitude input
	//XXX: Currently this method takes ~700ms just for the atittude controller
	if ( !( _control_input.input_mask & CMD_IN_IGNORE_ATTITUDE ) ) {
		fix16_t yaw_w = get_param_fix16( PARAM_MC_ANGLE_YAW_W );

		// If we are going to override the calculated yaw rate, just ignore it
		if ( !( _control_input.input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
			yaw_w = 0;
		}

		v3d angle_error = {0,0,0};
		qf16 qe = {_fc_1,0,0,0}; //No angle reference, so no angle error (R==R_sp==Identity)
		rot_error_from_attitude( &angle_error,
								 &qe,
								 &_control_input.q,
								 &_state_estimator.attitude,
								 yaw_w );

		v3d_mul_s(&rates_ref, &angle_error, get_param_fix16( PARAM_MC_ANGLE_P ));
	}

	//==-- Rate Control PIDs
	// Roll Rate
	if ( !( _control_input.input_mask & CMD_IN_IGNORE_ROLL_RATE ) ) {
		// Use the commanded roll rate goal
		rates_ref.x = input.r;
	}

	// Pitch Rate
	if ( !( _control_input.input_mask & CMD_IN_IGNORE_PITCH_RATE ) ) {
		// Use the commanded pitch rate goal
		rates_ref.y = input.p;
	}

	// Yaw Rate
	if ( !( _control_input.input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
		// Use the commanded yaw rate goal
		rates_ref.z = input.y;
	} else {
		// If we're in offboard mode, and we aren't going to override yaw rate, and
		// we want to fuse
		// XXX: Ideally this would be handled as an additional case using the IGNORE
		// flags, somehow...
		if ( ( _system_status.control_mode == MAIN_MODE_OFFBOARD ) &&
			 get_param_uint( PARAM_CONTROL_OB_FUSE_YAW_RATE ) ) {
			// Add in the additional yaw rate input
			// TODO: This is more of a hack. Needs to be conerted from euler rate to body rates (should effect goal_y, goal_p, and goal_r)
			rates_ref.z = fix16_add( rates_ref.z, input.y );
		}
	}

	// Constrain rates to set params
	rates_ref.x = fix16_constrain( rates_ref.x, -get_param_fix16( PARAM_MAX_ROLL_RATE ),
							  get_param_fix16( PARAM_MAX_ROLL_RATE ) );
	rates_ref.y = fix16_constrain( rates_ref.y, -get_param_fix16( PARAM_MAX_PITCH_RATE ),
							  get_param_fix16( PARAM_MAX_PITCH_RATE ) );
	rates_ref.z= fix16_constrain( rates_ref.z, -get_param_fix16( PARAM_MAX_YAW_RATE ),
							  get_param_fix16( PARAM_MAX_YAW_RATE ) );

	// Save intermittent goals and calculate rate error
	_control_input.r = rates_ref.x;
	_control_input.p = rates_ref.y;
	_control_input.y = rates_ref.z;

	// Rate PID Controllers
	v3d tau;
	tau.x = pid_step( &_pid_roll_rate, dt, rates_ref.x, _state_estimator.p );
	tau.y = pid_step( &_pid_pitch_rate, dt, rates_ref.y, _state_estimator.q );
	tau.z = pid_step( &_pid_yaw_rate, dt, rates_ref.z, _state_estimator.r );

	// XXX:
	//"Post-Scale/Normalize" the commands to act within
	// a range that is approriate for the motors. This
	// allows us to have higher PID gains for the rates
	//(by a factor of 100), and avoids complications
	// of getting close to the fixed-point step size
	_control_output.r = fix16_div( tau.x, _fc_100 );
	_control_output.p = fix16_div( tau.y, _fc_100 );
	_control_output.y = fix16_div( tau.z, _fc_100 );
}

void control_init( void ) {
	_system_status.control_mode = 0;
	_control_timing.time_last = 0;
	_control_timing.period_update = 1000 * fix16_to_int( fix16_div( _fc_1000, get_param_fix16( PARAM_RATE_CONTROL ) ) );
	_control_timing.period_stale = 4 * _control_timing.period_update; // Allow for some variance in update rate
	_control_timing.average_update = 0;

	pid_init( &_pid_roll_rate, get_param_fix16( PARAM_PID_ROLL_RATE_P ),
			  get_param_fix16( PARAM_PID_ROLL_RATE_I ),
			  get_param_fix16( PARAM_PID_ROLL_RATE_D ), _state_estimator.p, 0, 0,
			  -_fc_100,
			  _fc_100 ); // XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init( &_pid_pitch_rate, get_param_fix16( PARAM_PID_PITCH_RATE_P ),
			  get_param_fix16( PARAM_PID_PITCH_RATE_I ),
			  get_param_fix16( PARAM_PID_PITCH_RATE_D ), _state_estimator.q, 0, 0,
			  -_fc_100,
			  _fc_100 ); // XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init( &_pid_yaw_rate, get_param_fix16( PARAM_PID_YAW_RATE_P ),
			  get_param_fix16( PARAM_PID_YAW_RATE_I ),
			  get_param_fix16( PARAM_PID_YAW_RATE_D ), _state_estimator.r, 0, 0,
			  -_fc_100,
			  _fc_100 ); // XXX: Mixer input is normalized from 100/-100 to 1/-1

	_cmd_ob_input.r = 0;
	_cmd_ob_input.p = 0;
	_cmd_ob_input.y = 0;
	_cmd_ob_input.q.a = _fc_1;
	_cmd_ob_input.q.b = 0;
	_cmd_ob_input.q.c = 0;
	_cmd_ob_input.q.d = 0;
	_cmd_ob_input.T = 0;
	_cmd_ob_input.input_mask = 0;

	controller_reset();
	calc_mixer_output();	//XXX: Calculate a clean output to start with
}

void control_run( uint32_t now ) {
	// Run the control loop at a slower frequency so it is more resilient against noise
	// Ideally this will be locked to some update rate that is both slower and in-sync
	// with the estimator
	if( ( _state_estimator.time_updated - _control_timing.time_last ) > _control_timing.period_update ) {
		//Before we run control, the following requirements should be met:
		//	- The system is armed
		//	- We are running in an non-emergency state
		//	- The estimator has finished its initialization
		if( ( safety_is_armed() ) &&
			( _system_status.state != MAV_STATE_EMERGENCY ) &&
			( _state_estimator.time_updated > get_param_uint( PARAM_EST_INIT_TIME ) ) ) {
			//==-- Update Controller
			// Apply the current commands and update the PID controllers
			controller_run( _state_estimator.time_updated );
		} else {
			//==-- Reset Controller
			controller_reset(); // Reset the PIDs and output flat 0s for control
		}

		//XXX:	Calculate mixer output here, as we only need to update the
		//		output when we update the control values.
		calc_mixer_output();

		// Calculate timings for feedback
		_control_timing.average_update = fix16_div(
			_fc_1000, fix16_from_int( ( now - _control_timing.time_last ) / 1000 ) );

		_control_timing.time_last = now;
	}
}

#ifdef __cplusplus
}
#endif
