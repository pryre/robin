#ifdef __cplusplus
extern "C" {
#endif

#include "safety.h"
#include "controller.h"
#include "drivers/drv_sensors.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"
#include "mavlink_system.h"
#include "mixer.h"
#include "params.h"
#include "sensors.h"

#include "fixextra.h"

system_status_t _system_status;
sensor_readings_t _sensors;
command_input_t _cmd_ob_input;
command_input_t _control_input;
control_output_t _control_output;

static const char mav_state_names[MAV_STATE_NUM_STATES][MAV_STATE_NAME_LEN] = {
	"UNINIT",
	"BOOT",
	"CALIBRATE",
	"STANDBY",
	"ACTIVE",
	"CRITICAL",
	"EMERGENCY",
	"POWEROFF",
	"TERMINATE"
};

static const char mav_mode_names[MAV_MODE_NUM_MODES][MAV_MODE_NAME_LEN] = {
	"UNSET",
	"MANUAL",
	"ALTCTL",
	"POSCTL",
	"AUTO",
	"ACRO",
	"OFFBOARD",
	"STABILIZED",
	"RATTITUDE"
};

static uint32_t _time_safety_arm_throttle_timeout;	//Used to auto-disarm if no input detected after arm
static uint32_t _time_safety_critical_timeout;	//Used to "upgrade" to emergency status if in critical state is not recovered from
static uint32_t _time_safety_button_pressed;
static bool _new_safety_button_press;

static void init_sensor_state( timeout_status_t* sensor, const char* name,
							   const param_id_t param_stream_count,
							   const param_id_t param_timeout ) {
	sensor->health = SYSTEM_HEALTH_UNKNOWN;
	sensor->last_read = 0;
	sensor->count = 0;
	sensor->param_stream_count = param_stream_count;
	sensor->param_timeout = param_timeout;
	strncpy( sensor->name, name, SAFETY_SENSOR_NAME_LEN );
}

void safety_init() {
	init_sensor_state( &_system_status.sensors.hil, "HIL",
					   PARAM_SENSOR_HIL_STRM_COUNT, PARAM_SENSOR_HIL_TIMEOUT );
	init_sensor_state( &_system_status.sensors.imu, "IMU",
					   PARAM_SENSOR_IMU_STRM_COUNT, PARAM_SENSOR_IMU_TIMEOUT );
	init_sensor_state( &_system_status.sensors.mag, "Magnetometer",
					   PARAM_SENSOR_MAG_STRM_COUNT, PARAM_SENSOR_MAG_TIMEOUT );
	init_sensor_state( &_system_status.sensors.baro, "Barometer",
					   PARAM_SENSOR_BARO_STRM_COUNT, PARAM_SENSOR_BARO_TIMEOUT );
	init_sensor_state( &_system_status.sensors.sonar, "Sonar",
					   PARAM_SENSOR_SONAR_STRM_COUNT, PARAM_SENSOR_SONAR_TIMEOUT );
	init_sensor_state( &_system_status.sensors.ext_pose, "External Pose",
					   PARAM_SENSOR_EXT_POSE_STRM_COUNT,
					   PARAM_SENSOR_EXT_POSE_TIMEOUT );
	init_sensor_state( &_system_status.sensors.rc_input, "RC Input",
					   PARAM_SENSOR_RC_INPUT_STRM_COUNT,
					   PARAM_SENSOR_RC_INPUT_TIMEOUT );
	init_sensor_state( &_system_status.sensors.offboard_heartbeat,
					   "Offboard Heartbeat", PARAM_SENSOR_OFFB_HRBT_STRM_COUNT,
					   PARAM_SENSOR_OFFB_HRBT_TIMEOUT );
	init_sensor_state( &_system_status.sensors.offboard_control,
					   "Offboard Control", PARAM_SENSOR_OFFB_CTRL_STRM_COUNT,
					   PARAM_SENSOR_OFFB_CTRL_TIMEOUT );
	init_sensor_state( &_system_status.sensors.offboard_mixer_g0_control,
					   "Group 0 Actuator", PARAM_SENSOR_OFFB_G0_STRM_COUNT,
					   PARAM_SENSOR_OFFB_G0_TIMEOUT );
	init_sensor_state( &_system_status.sensors.offboard_mixer_g1_control,
					   "Group 1 Actuator", PARAM_SENSOR_OFFB_G1_STRM_COUNT,
					   PARAM_SENSOR_OFFB_G1_TIMEOUT );
	// init_sensor_state(&_system_status.sensors.pwm_control, "PWM Control",
	// PARAM_SENSOR_PWM_CTRL_STRM_COUNT, PARAM_SENSOR_PWM_CTRL_TIMEOUT);

	_time_safety_arm_throttle_timeout = 0;
	_time_safety_critical_timeout = 0;

	_new_safety_button_press = false;
	_time_safety_button_pressed = 0;
}

bool safety_is_armed( void ) {
	return _system_status.arm_status;
}

bool safety_switch_engaged( void ) {
	return ( !_system_status.safety_button_status ) && ( _sensors.safety_button.status.present );
}

bool safety_request_state( uint8_t req_state ) {
	bool change_state = false;

	// XXX: State request to same state, just say OK as we don't want to taint timeouts
	if ( _system_status.state == req_state ) {
		change_state = true;
	} else {
		switch ( req_state ) {
		case MAV_STATE_BOOT: {
			if ( _system_status.state == MAV_STATE_UNINIT )
				change_state = true;

			break;
		}
		case MAV_STATE_CALIBRATING: {
			if ( _system_status.state == MAV_STATE_STANDBY )
				change_state = true;

			break;
		}
		case MAV_STATE_STANDBY: {
			if ( !safety_is_armed() && ( ( _system_status.state == MAV_STATE_ACTIVE ) || ( _system_status.state == MAV_STATE_CRITICAL ) || ( _system_status.state == MAV_STATE_EMERGENCY ) ) ) {
				change_state = true;
			} else if ( _system_status.state == MAV_STATE_CALIBRATING ) {
				change_state = true;
			} else if ( _system_status.state == MAV_STATE_BOOT ) {
				change_state = true;
			}

			break;
		}
		case MAV_STATE_ACTIVE: {
			if ( _system_status.state == MAV_STATE_STANDBY )
				change_state = true;

			break;
		}
		case MAV_STATE_CRITICAL: {
			if ( _system_status.state == MAV_STATE_ACTIVE ) {

				if( get_param_uint( PARAM_CRITICAL_TIMEOUT ) )
					_time_safety_critical_timeout = system_micros();

				change_state = true;
			}

			break;
		}
		case MAV_STATE_EMERGENCY: { // Allow any request to put the mav into

			change_state = true;

			break;
		}
		case MAV_STATE_POWEROFF: { // Allows the mav to check it is safe to
			// poweroff/reboot
			if ( ( _system_status.state == MAV_STATE_UNINIT ) || ( _system_status.state == MAV_STATE_BOOT ) || ( _system_status.state == MAV_STATE_STANDBY ) )
				change_state = true;

			break;
		}
		default: { // Other states cannot be requested
			break;
		}
		}

		if ( change_state )
			_system_status.state = req_state;
	}

	return change_state;
}

static bool check_control_mode_inputs( uint8_t req_ctrl_mode ) {
	bool control_mode_ok = false;

	switch ( req_ctrl_mode ) {
	case MAIN_MODE_STABILIZED: {
		if ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK )
			control_mode_ok = true;

		break;
	}
	case MAIN_MODE_ACRO: {
		if ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK )
			control_mode_ok = true;

		break;
	}
	case MAIN_MODE_OFFBOARD: {
		if ( ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) && ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) )
			control_mode_ok = true;

		break;
	}
	default: { // Other control modes cannot be requested
		break;
	}
	}

	return control_mode_ok;
}

bool safety_request_control_mode( uint8_t req_ctrl_mode ) {
	bool change_ctrl_mode = false;

	if ( _system_status.control_mode == req_ctrl_mode ) { // XXX: State request to same state, just say OK
		change_ctrl_mode = true;
	} else {
		change_ctrl_mode = check_control_mode_inputs( req_ctrl_mode );
	}

	if ( change_ctrl_mode ) {
		_system_status.control_mode = req_ctrl_mode;

		char text_success[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] Control changed: ";

		strncat( text_success, mav_mode_names[_system_status.control_mode],
				 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 ); // XXX: Stops string overflow warnings

		mavlink_queue_broadcast_notice( text_success );

		status_buzzer_success();
	} else {
		char text_error[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] Control change denied: ";

		strncat( text_error, mav_mode_names[req_ctrl_mode],
				 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 ); // XXX: Stops string overflow warnings

		mavlink_queue_broadcast_error( text_error );

		status_buzzer_failure();
	}

	return change_ctrl_mode;
}

static void do_safety_arm( void ) {
	_system_status.arm_status = true; // ARM!
	mavlink_queue_broadcast_notice( "[SAFETY] Mav armed!" );

	status_buzzer_success();
}

bool safety_request_arm( void ) {
	bool result = false;
	bool mode_check = false;
	bool control_check = false;
	bool throttle_check = false;

	char text_error[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] Arming denied: ";
	char text_reason[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

	// Skip all this if we're already armed
	if ( _system_status.arm_status ) {
		do_safety_arm();
		result = true;
	} else {
		switch ( _system_status.control_mode ) {
		case MAIN_MODE_STABILIZED: {
			mode_check = true;

			if ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) {
				control_check = true;

				if ( _sensors.rc_input.c_T < _fc_0_05 )
					throttle_check = true;
			}

			break;
		}
		case MAIN_MODE_ACRO: {
			mode_check = true;

			if ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) {
				control_check = true;

				if ( _sensors.rc_input.c_T < _fc_0_05 )
					throttle_check = true;
			}

			break;
		}
		case MAIN_MODE_OFFBOARD: {
			mode_check = true;

			if ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) {
				control_check = true;

				if ( ( _cmd_ob_input.T == 0 ) || ( _cmd_ob_input.input_mask & CMD_IN_IGNORE_THROTTLE ) ) {
					throttle_check = true;
				}
			}

			break;
		}
		default: { break; }
		}

		// Make sure low throttle is being output
		throttle_check &= ( _control_output.T == 0 );

		if ( !safety_switch_engaged() && ( _system_status.health == SYSTEM_HEALTH_OK ) && ( control_check ) && ( throttle_check ) && ( mode_check ) ) {
			if ( safety_request_state( MAV_STATE_ACTIVE ) ) {
				do_safety_arm();
				result = true;

				// Record down the arm time for throttle timeout if enabled
				if ( get_param_uint( PARAM_THROTTLE_TIMEOUT ) > 0 )
					_time_safety_arm_throttle_timeout = system_micros();
			} else {
				strncpy( text_reason, "state change from ",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );

				strncat( text_reason, mav_state_names[_system_status.state],
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 ); // XXX: Stops string overflow warnings

				strncat( text_error, text_reason,
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 ); // XXX: Stops string overflow errors
				mavlink_queue_broadcast_error( text_error );
			}
		} else {
			if ( safety_switch_engaged() ) {
				strncpy( text_reason, "safety engaged",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
			} else if ( _system_status.health != SYSTEM_HEALTH_OK ) {
				strncpy( text_reason, "sensor error ",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );

				if ( _sensors.imu.status.present && ( _system_status.sensors.imu.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(IMU)", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( _sensors.mag.status.present && ( _system_status.sensors.mag.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(mag)", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( _sensors.baro.status.present && ( _system_status.sensors.baro.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(baro)", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( _sensors.sonar.status.present && ( _system_status.sensors.sonar.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(sonar)",
							 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( _sensors.ext_pose.status.present && ( _system_status.sensors.ext_pose.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(ext_pose)",
							 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( _sensors.hil.status.present && ( _system_status.sensors.hil.health != SYSTEM_HEALTH_OK ) ) {
					strncpy( text_reason, "(HIL)", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				} else if ( !control_check ) {
					if ( ( _system_status.control_mode == MAIN_MODE_STABILIZED ) || ( _system_status.control_mode == MAIN_MODE_ACRO ) ) {
						strncpy( text_reason, "(rc_input)",
								 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
					} else if ( _system_status.control_mode == MAIN_MODE_OFFBOARD ) {
						if ( _system_status.sensors.offboard_heartbeat.health != SYSTEM_HEALTH_OK ) {
							strncpy( text_reason, "(offb_hrbt)",
									 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
						} else if ( _system_status.sensors.offboard_control.health != SYSTEM_HEALTH_OK ) {
							strncpy( text_reason, "(offb_ctrl)",
									 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
						}
					} else if ( _system_status.control_mode == MAIN_MODE_UNSET ) {
						strncpy( text_reason, "(no_mode)",
								 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
					} else {
						strncpy( text_reason, "(no_ctrl)",
								 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
					}
				} else {
					strncpy( text_reason, "(unkown)",
							 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
				}
			} else if ( !mode_check ) {
				strncpy( text_reason, "flight mode not set",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
			} else if ( !control_check ) {
				strncpy( text_reason, "no control input",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
			} else if ( !throttle_check ) {
				strncpy( text_reason, "high throttle",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
			} else {
				// XXX: Should never get here, but hust in case
				strncpy( text_reason, "undefined arming failure",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
			}

			strncat( text_error, text_reason,
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 ); // XXX: Stops string overflow errors
			mavlink_queue_broadcast_error( text_error );

			status_buzzer_failure();
		}
	}

	return result;
}

bool safety_request_disarm( void ) {
	bool result = false;

	_system_status.arm_status = false; // DISARM!

	_time_safety_arm_throttle_timeout = 0;

	mavlink_queue_broadcast_notice( "[SAFETY] Mav disarmed!" );

	if ( safety_request_state( MAV_STATE_STANDBY ) ) {
		mavlink_queue_broadcast_info( "[SAFETY] Mav returned to standby state" );
	} else {
		mavlink_queue_broadcast_error(
			"[SAFETY] Unable to return to standby state!" );
	}

	status_buzzer_success();

	result = true; // XXX: Never fail a disarm request

	return result;
}

static void safety_switch_update( uint32_t time_now ) {
	if ( _sensors.safety_button.status.new_data ) {
		if ( !_sensors.safety_button.state ) { // Inversed as a pull-up resistor is
			// used for button detect
			_time_safety_button_pressed = time_now;
			_new_safety_button_press = true;
		} else {
			_new_safety_button_press = false;
		}

		_sensors.safety_button.status.new_data = false;
	}

	if ( _new_safety_button_press && ( time_now - _time_safety_button_pressed ) > 1000000 ) {
		_system_status.safety_button_status = !_system_status.safety_button_status; // Toggle arming safety
		_new_safety_button_press = false;

		// If the mav was armed and safety was switched off, disarm it
		if ( !_system_status.safety_button_status && safety_is_armed() ) {
			safety_request_disarm();
		}

		status_buzzer_success();
	}
}

void safety_update_sensor( timeout_status_t* sensor ) {
	sensor->last_read = system_micros();
	sensor->count++;

	if ( ( sensor->health == SYSTEM_HEALTH_TIMEOUT ) && ( sensor->count > get_param_uint( sensor->param_stream_count ) ) ) {
		sensor->health = SYSTEM_HEALTH_OK;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Connected: ";
		strncat( text, sensor->name,
				 ( MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 20 ) ); // Magic number is size of the message start
		// mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_info( &text[0] );
	} else if ( sensor->health == SYSTEM_HEALTH_UNKNOWN ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
	}
}

static void safety_check_sensor( timeout_status_t* sensor, uint32_t time_now ) {
	if ( ( sensor->health == SYSTEM_HEALTH_OK ) && ( ( time_now - sensor->last_read ) > get_param_uint( sensor->param_timeout ) ) ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
		sensor->count = 0;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Timeout: ";
		strncat( text, sensor->name,
				 ( MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 18 ) ); // Magic number is size of the message start
		// mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	}
}

static void safety_arm_throttle_timeout( uint32_t time_now ) {
	if ( _time_safety_arm_throttle_timeout ) { // If the timeout is active
		if ( _control_input.T > get_param_fix16( PARAM_THROTTLE_TIMEOUT_VALUE ) ) {
			_time_safety_arm_throttle_timeout = 0; // We have recieved throttle input, disable timeout
		} else if ( ( time_now - _time_safety_arm_throttle_timeout ) > get_param_uint( PARAM_THROTTLE_TIMEOUT ) ) {
			mavlink_queue_broadcast_error( "[SAFETY] Throttle timeout, disarming!" );
			safety_request_disarm();
		}
	}
}

static void safety_check_failsafe( uint32_t time_now ) {
	// Auto throttle timeout
	safety_arm_throttle_timeout( time_now );

	// Handle system health issues
	if( _system_status.health != SYSTEM_HEALTH_OK ) {
		// If flight-critical systems are down
		// Make sure the system is not in HIL mode, as flight-critical sensors missing
		// Normally, we only really need to worry about IMU (XXX: more?)
		if( ( !_sensors.hil.status.present ) &&
			( _system_status.sensors.imu.health != SYSTEM_HEALTH_OK ) ) { // Flight-critical issue, call emergency state
			safety_request_state( MAV_STATE_EMERGENCY );
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] EMERGENCY! Unhealthy, hard failsafe!";
			mavlink_queue_broadcast_error( &text[0] );
		} else { // Non-flight-critical, call critical system state
			safety_request_state( MAV_STATE_CRITICAL );
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] CRITICAL! Unhealthy, soft failsafe!";
			mavlink_queue_broadcast_error( &text[0] );
		}
	}
}

static void safety_critical_timeout( uint32_t time_now ) {
	// If the timeout is active
	if ( _time_safety_critical_timeout ) {
		if ( ( time_now - _time_safety_critical_timeout ) > get_param_uint( PARAM_CRITICAL_TIMEOUT ) ) {
			mavlink_queue_broadcast_error( "[SAFETY] EMERGENCY! Critical state unrecoverable!" );
			safety_request_state( MAV_STATE_EMERGENCY );
		}
	}
}

static void system_state_update( uint32_t time_now ) {
	//==-- System State Machine
	switch ( _system_status.state ) {
		case MAV_STATE_UNINIT: {
			break;
		}
		case MAV_STATE_BOOT: {
			break;
		}
		case MAV_STATE_CALIBRATING: {
			break;
		}
		case MAV_STATE_STANDBY: {
			break;
		}
		case MAV_STATE_ACTIVE: {
			// Check if system for failsafe tasks
			safety_check_failsafe( time_now );

			break;
		}
		case MAV_STATE_CRITICAL: {
			// Check to see if we need to elevate to emergency state
			safety_critical_timeout( time_now );

			break;
		}
		case MAV_STATE_EMERGENCY: {
			safety_request_disarm();

			break;
		}
		default: {
			safety_request_state( MAV_STATE_EMERGENCY );

			mavlink_queue_broadcast_error(
				"[SAFETY] Emergency! Mav entered unknown state!" );

			break;
		}
	}

	//==-- System Mode Reporting
	_system_status.mode = MAV_MODE_PREFLIGHT;

	// Report if armed
	if ( safety_is_armed() )
		_system_status.mode |= MAV_MODE_FLAG_SAFETY_ARMED;

	// Report if mav is controlling itself
	//if ( _system_status.state == MAV_STATE_CRITICAL )
	//	_system_status.mode |= MAV_MODE_FLAG_AUTO_ENABLED;

	// Report offboard input status
	bool ctrl_mode_ok = false;

	if ( ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) && ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) ) {
		_system_status.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;

		ctrl_mode_ok = true;
	}

	if ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) {
		_system_status.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

		ctrl_mode_ok = true;
	}

	if ( !ctrl_mode_ok ) {
		_system_status.control_mode = 0;
	}
}

static void safety_health_update( uint32_t time_now ) {
	bool sensors_ok = ( !_sensors.imu.status.present || ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) ) && ( !_sensors.mag.status.present || ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) ) && ( !_sensors.baro.status.present || ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) ) && ( !_sensors.sonar.status.present || ( _system_status.sensors.sonar.health == SYSTEM_HEALTH_OK ) ) && ( !_sensors.ext_pose.status.present || ( _system_status.sensors.ext_pose.health == SYSTEM_HEALTH_OK ) ) && ( !_sensors.hil.status.present || ( _system_status.sensors.hil.health == SYSTEM_HEALTH_OK ) );

	bool control_ok = check_control_mode_inputs( _system_status.control_mode );

	if ( sensors_ok && control_ok ) {
		_system_status.health = SYSTEM_HEALTH_OK;
	} else {
		_system_status.health = SYSTEM_HEALTH_ERROR;
	}
}

uint32_t compat_encode_px4_main_mode( uint8_t main_mode ) {
	return ( uint32_t )( main_mode << 16 );
}

uint8_t compat_decode_px4_main_mode( uint32_t mode ) {
	return ( uint8_t )( mode >> 16 );
}

void safety_run( uint32_t time_now ) {
	// Check sensors to ensure they are operating correctly
	safety_check_sensor( &_system_status.sensors.imu, time_now );
	safety_check_sensor( &_system_status.sensors.mag, time_now );
	safety_check_sensor( &_system_status.sensors.baro, time_now );
	safety_check_sensor( &_system_status.sensors.sonar, time_now );
	safety_check_sensor( &_system_status.sensors.ext_pose, time_now );
	safety_check_sensor( &_system_status.sensors.rc_input, time_now );
	safety_check_sensor( &_system_status.sensors.offboard_heartbeat, time_now );
	safety_check_sensor( &_system_status.sensors.offboard_control, time_now );
	safety_check_sensor( &_system_status.sensors.offboard_mixer_g0_control,
						 time_now );
	safety_check_sensor( &_system_status.sensors.offboard_mixer_g1_control,
						 time_now );
	// safety_check_sensor( &_system_status.sensors.pwm_control, time_now );

	if ( _sensors.hil.status.present )
		safety_check_sensor( &_system_status.sensors.hil, time_now );

	// Update the overall system health based on the individual sensors
	safety_health_update( time_now );

	// Check the safety switch to see if the user has pushed or released it
	safety_switch_update( time_now );

	// Make sure current system state and mode are valid, and handle changes
	system_state_update( time_now );
}

void safety_prepare_graceful_shutdown( void ) {
	// Make sure outputs for PWMs are set to off
	mixer_clear_outputs();

	// Make sure there are no i2c jobs still running
	drv_sensors_i2c_clear();

	// Give a few final moments for the comms to send and PWM to update
	system_pause_ms( 500 );
}

#ifdef __cplusplus
}
#endif
