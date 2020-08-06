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

#include "controllers/control_lib.h"

void controller_set_input_zero( command_input_t* input ) {
	input->r = 0;
	input->p = 0;
	input->y = 0;
	input->q.a = _fc_1;
	input->q.b = 0;
	input->q.c = 0;
	input->q.d = 0;
	input->T = 0;
	input->input_mask = 0;
}

void controller_set_output_zero( command_output_t *output ) {
	output->r = 0;
	output->p = 0;
	output->y = 0;
	output->T = 0;
}

void controller_set_input_from_mode( command_input_t* input ) {
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
			// Failsafe
			//Set our initial failsafe parameters (low throttle and stay steady in acro)
			input.T = get_param_fix16( PARAM_FAILSAFE_THROTTLE );
			input.input_mask |= CMD_IN_IGNORE_ATTITUDE; // Set it to just hold rpy rates
														// (as this skips unnessessary
														// computing during boot, and is
														// possibly safer)

			break;
		}
	}
}

// Technique adapted from the Pixhawk multirotor control scheme (~December 2018)
void rot_error_from_attitude( v3d* e_R, qf16* qe, const qf16* q_sp, const qf16* q, const fix16_t yaw_w ) {
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




#ifdef __cplusplus
}
#endif
