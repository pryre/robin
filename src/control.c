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

#include "estimator.h"
#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "drivers/drv_system.h"
#include "drivers/drv_status_io.h"

#include "control.h"
#include "controllers/controller_att_nac.h"
#include "controllers/controller_att_pid.h"
#include "controllers/control_lib.h"

//system_status_t _system_status;
//sensor_readings_t _sensors;
//state_t _state_estimator;

control_timing_t _control_timing;
command_input_t _cmd_ob_input;
command_input_t _control_input;

static void control_reset( void ) {
	controller_att_pid_reset();
	controller_att_nac_reset();
}

//XXX: c is the input signal: c = [tx; ty; tz; Tz]
static void control_step( mf16* c, const uint32_t time_now ) {
	// Variables that store the computed attitude goal rates
	command_input_t input;
	control_lib_set_input_zero( &input ); // Failsafe input to be safe

	// Handle controller input
	//_system_status.mode |= MAV_MODE_FLAG_MANUAL_ENABLED

	if((c->rows == 4) && (c->columns = 1)) {
		control_lib_set_input_from_mode( &input );

		// Do control!
		fix16_t dt = fix16_from_float( 1e-6 * (float)( time_now - _control_timing.time_last ) ); // Delta time in seconds

		if ( ( dt == 0 ) || ( time_now - _control_timing.time_last > _control_timing.period_stale ) ) {
			dt = 0;
			control_reset();
		}

		// Save the control input mask used
		_control_input.input_mask = input.input_mask;

		//==-- Throttle Control
		fix16_t goal_throttle = 0;

		// Trottle
		if ( !( input.input_mask & CMD_IN_IGNORE_THROTTLE ) ) {
			// Use the commanded throttle
			goal_throttle = input.T;
		}

		// Save intermittent goals
		_control_input.T = goal_throttle;

		// Don't allow for derivative or integral calculations if
		// there is a very low throttle
		if ( goal_throttle < _fc_0_05 ) {
			dt = 0;
		}

		//Attitude Control
		//Normalize this before continuing (and save for recording)
		qf16_normalize_to_unit( &_control_input.q, &input.q );
		input.q = _control_input.q;

		v3d c_att = V3D_ZERO;
		v3d rates_ref = V3D_ZERO;
		if( get_param_uint(PARAM_MC_USE_NAC) ) {
			controller_att_nac_step(&c_att, &rates_ref, &input, &_state_estimator, dt);
		} else {
			controller_att_pid_step(&c_att, &rates_ref, &input, &_state_estimator, dt);
		}

		// Save intermittent goals and calculate rate error
		_control_input.r = rates_ref.x;
		_control_input.p = rates_ref.y;
		_control_input.y = rates_ref.z;

		// Set our control references
		c->data[0][0] = c_att.x;
		c->data[1][0] = c_att.y;
		c->data[2][0] = c_att.z;
		c->data[3][0] = _control_input.T;
	} else {
		safety_request_state(MAV_STATE_EMERGENCY);
		mavlink_queue_broadcast_error( "Control: Invalid control signal var(c)" );
		status_buzzer_failure();
	}
}

void control_init( void ) {
	_system_status.control_mode = 0;
	_control_timing.time_last = 0;
	_control_timing.period_update = 1000 * fix16_to_int( fix16_div( _fc_1000, get_param_fix16( PARAM_RATE_CONTROL ) ) );
	_control_timing.period_stale = 4 * _control_timing.period_update; // Allow for some variance in update rate
	_control_timing.average_update = 0;

	controller_att_pid_init();
	controller_att_nac_init();

	control_lib_set_input_zero(&_control_input);
	control_lib_set_input_zero(&_cmd_ob_input);

	control_reset();
}

void control_run( uint32_t now ) {
	// Run the control loop at a slower frequency so it is more resilient against noise
	// Ideally this will be locked to some update rate that is both slower and in-sync
	// with the estimator
	if( ( _state_estimator.time_updated - _control_timing.time_last ) > _control_timing.period_update ) {
		//XXX: c is the input signal: c = [tx; ty; tz; Tz]
		mf16 c = { .rows = 4, .columns = 1, .errors = 0 }; //Data is 0 by default

		//Before we run control, the following requirements should be met:
		//	- The system is armed
		//	- We are running in an non-emergency state
		//	- The estimator has finished its initialization
		if( ( safety_is_armed() ) &&
			( _system_status.state != MAV_STATE_EMERGENCY ) &&
			( _system_status.state != MAV_STATE_CRITICAL ) &&
			( _state_estimator.time_updated > get_param_uint( PARAM_EST_INIT_TIME ) ) ) {
			//==-- Update Controller
			// Apply the current commands and update the PID controllers
			// but keep things running in step with the estimator
			control_step( &c, _state_estimator.time_updated );
		} else {
			//

			//==-- Reset Controller
			//We're in pretty bad shape, so zero everything for "complete failsafe"
			control_reset(); // Reset the PIDs and output flat 0s for control
		}

		//XXX:	Calculate mixer output here, as we only need to update the
		//		output when we update the control values.
		mixer_set_primary_using_map( &c );

		// Calculate timings for feedback
		_control_timing.average_update = fix16_div(
			_fc_1000, fix16_from_int( ( now - _control_timing.time_last ) / 1000 ) );

		_control_timing.time_last = now;
	}
}

#ifdef __cplusplus
}
#endif
