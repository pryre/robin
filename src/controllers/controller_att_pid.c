#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixextra.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "controllers/lib_control.h"
#include "controllers/lib_control_pid.h"
#include "controllers/att_controller_pid.h"

pid_controller_t _pid_roll_rate;
pid_controller_t _pid_pitch_rate;
pid_controller_t _pid_yaw_rate;

static void controller_att_reset( void ) {
	pid_reset( &_pid_roll_rate, _state_estimator.p );
	pid_reset( &_pid_pitch_rate, _state_estimator.q );
	pid_reset( &_pid_yaw_rate, _state_estimator.r );
}

void controller_att_pid_init() {
	pid_init( &_pid_roll_rate,
			  get_param_fix16( PARAM_PID_ROLL_RATE_P ),
			  get_param_fix16( PARAM_PID_ROLL_RATE_I ),
			  get_param_fix16( PARAM_PID_ROLL_RATE_D ),
			  _state_estimator.p, 0, 0, -_fc_100, _fc_100 );
			  // XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init( &_pid_pitch_rate,
			  get_param_fix16( PARAM_PID_PITCH_RATE_P ),
			  get_param_fix16( PARAM_PID_PITCH_RATE_I ),
			  get_param_fix16( PARAM_PID_PITCH_RATE_D ),
			  _state_estimator.q, 0, 0, -_fc_100, _fc_100 );
			  // XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init( &_pid_yaw_rate,
			  get_param_fix16( PARAM_PID_YAW_RATE_P ),
			  get_param_fix16( PARAM_PID_YAW_RATE_I ),
			  get_param_fix16( PARAM_PID_YAW_RATE_D ),
			  _state_estimator.r, 0, 0, -_fc_100, _fc_100 );
			  // XXX: Mixer input is normalized from 100/-100 to 1/-1
}

void controller_att_pid_step( v3d* tau, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt ) {
	//==-- Attitude Control
	// If we should listen to attitude input
	//XXX: Currently this method takes ~700ms just for the atittude controller
	if ( !( input->input_mask & CMD_IN_IGNORE_ATTITUDE ) ) {
		fix16_t yaw_w = get_param_fix16( PARAM_MC_ANGLE_YAW_W );

		// If we are going to override the calculated yaw rate, just ignore it
		if ( !( input->input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
			yaw_w = 0;
		}

		v3d angle_error = {0,0,0};
		qf16 qe = {_fc_1,0,0,0}; //No angle reference, so no angle error (R==R_sp==Identity)
		rot_error_from_attitude( &angle_error,
								 &qe,
								 &(input->q),
								 &(state->attitude),
								 yaw_w );

		v3d_mul_s(&rates_ref, &angle_error, get_param_fix16( PARAM_MC_ANGLE_P ));
	}

	//==-- Rate Control PIDs
	// Roll Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_ROLL_RATE ) ) {
		// Use the commanded roll rate goal
		rates_ref->x = input.r;
	}

	// Pitch Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_PITCH_RATE ) ) {
		// Use the commanded pitch rate goal
		rates_ref->y = input.p;
	}

	// Yaw Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
		// Use the commanded yaw rate goal
		rates_ref->z = input.y;
	} else {
		// If we're in offboard mode, and we aren't going to override yaw rate, and
		// we want to fuse
		// XXX: Ideally this would be handled as an additional case using the IGNORE
		// flags, somehow...
		if ( ( _system_status.control_mode == MAIN_MODE_OFFBOARD ) &&
			 get_param_uint( PARAM_CONTROL_OB_FUSE_YAW_RATE ) ) {
			// Add in the additional yaw rate input
			// TODO: This is more of a hack. Needs to be conerted from euler rate to body rates (should effect goal_y, goal_p, and goal_r)
			rates_ref->z = fix16_add( rates_ref->z, input.y );
		}
	}

	// Constrain rates to set params
	rates_ref->x = fix16_constrain( rates_ref->x, -get_param_fix16( PARAM_MAX_ROLL_RATE ),
							  get_param_fix16( PARAM_MAX_ROLL_RATE ) );
	rates_ref->y = fix16_constrain( rates_ref->y, -get_param_fix16( PARAM_MAX_PITCH_RATE ),
							  get_param_fix16( PARAM_MAX_PITCH_RATE ) );
	rates_ref->z= fix16_constrain( rates_ref->z, -get_param_fix16( PARAM_MAX_YAW_RATE ),
							  get_param_fix16( PARAM_MAX_YAW_RATE ) );


	// Rate PID Controllers
	v3d u;
	u.x = pid_step( &_pid_roll_rate, dt, rates_ref->x, state->p );
	u.y = pid_step( &_pid_pitch_rate, dt, rates_ref->y, state->q );
	u.z = pid_step( &_pid_yaw_rate, dt, rates_ref->z, state->r );

	// XXX:
	//"Post-Scale/Normalize" the commands to act within
	// a range that is approriate for the motors. This
	// allows us to have higher PID gains for the rates
	//(by a factor of 100), and avoids complications
	// of getting close to the fixed-point step size
	tau.r = fix16_div( u.x, _fc_100 );
	tau.p = fix16_div( u.y, _fc_100 );
	tau.y = fix16_div( u.z, _fc_100 );
}
