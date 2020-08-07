#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixextra.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "safety.h"	//XXX: This is included to do the hack for better yaw tracking in stab mode
#include "controllers/control_lib.h"
#include "controllers/control_lib_pid.h"
#include "controllers/controller_att_pid.h"

pid_controller_t _pid_roll_rate;
pid_controller_t _pid_pitch_rate;
pid_controller_t _pid_yaw_rate;

void controller_att_pid_reset( void ) {
	pid_reset( &_pid_roll_rate, _state_estimator.p );
	pid_reset( &_pid_pitch_rate, _state_estimator.q );
	pid_reset( &_pid_yaw_rate, _state_estimator.r );
}

void controller_att_pid_init( void ) {
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

void controller_att_pid_step( v3d* u, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt ) {
	//==-- Attitude Control
	v3d rates = {0,0,0};
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
		control_lib_q_att_error( &angle_error,
								 &qe,
								 &(input->q),
								 &(state->attitude),
								 yaw_w );

		v3d_mul_s(rates_ref, &angle_error, get_param_fix16( PARAM_MC_ANGLE_P ));
	}

	//==-- Rate Control PIDs
	// Roll Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_ROLL_RATE ) ) {
		// Use the commanded roll rate goal
		rates.x = input->r;
	}

	// Pitch Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_PITCH_RATE ) ) {
		// Use the commanded pitch rate goal
		rates.y = input->p;
	}

	// Yaw Rate
	if ( !( input->input_mask & CMD_IN_IGNORE_YAW_RATE ) ) {
		// Use the commanded yaw rate goal
		rates.z = input->y;
	} else {
		// XXX: Better yaw tracking in stab mode
		// If we're in offboard mode, and we aren't going to override yaw rate, and
		// we want to fuse
		// XXX: Ideally this would be handled as an additional case using the IGNORE
		// flags, somehow...
		if ( ( _system_status.control_mode == MAIN_MODE_OFFBOARD ) &&
			 get_param_uint( PARAM_CONTROL_OB_FUSE_YAW_RATE ) ) {
			// Add in the additional yaw rate input
			// TODO: This is more of a hack. Needs to be conerted from euler rate to body rates (should effect goal_y, goal_p, and goal_r)
			rates.z = fix16_add( rates.z, input->y );
		}
	}

	// Constrain rates to set params
	rates.x = fix16_constrain( rates.x, -get_param_fix16( PARAM_MAX_ROLL_RATE ),
							  get_param_fix16( PARAM_MAX_ROLL_RATE ) );
	rates.y = fix16_constrain( rates.y, -get_param_fix16( PARAM_MAX_PITCH_RATE ),
							  get_param_fix16( PARAM_MAX_PITCH_RATE ) );
	rates.z= fix16_constrain( rates.z, -get_param_fix16( PARAM_MAX_YAW_RATE ),
							  get_param_fix16( PARAM_MAX_YAW_RATE ) );

	//Save our rates reference
	*rates_ref = rates;

	// Rate PID Controllers
	v3d u_s; //scaled control input
	u_s.x = pid_step( &_pid_roll_rate, dt, rates.x, state->p );
	u_s.y = pid_step( &_pid_pitch_rate, dt, rates.y, state->q );
	u_s.z = pid_step( &_pid_yaw_rate, dt, rates.z, state->r );

	// XXX:
	//"Post-Scale/Normalize" the commands to act within
	// a range that is approriate for the motors. This
	// allows us to have higher PID gains for the rates
	//(by a factor of 100), and avoids complications
	// of getting close to the fixed-point step size
	u->x = fix16_div( u_s.x, _fc_100 );
	u->y = fix16_div( u_s.y, _fc_100 );
	u->z = fix16_div( u_s.z, _fc_100 );
}
