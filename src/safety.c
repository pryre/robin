#ifdef __cplusplus
extern "C" {
#endif

#include "breezystm32.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "controller.h"

#include <stdio.h>

system_status_t _system_status;
sensor_readings_t _sensors;
command_input_t _command_input;
control_output_t _control_output;
char mav_state_names[MAV_STATE_NUM_STATES][MAV_STATE_NAME_LEN];
char mav_mode_names[MAV_MODE_NUM_MODES][MAV_MODE_NAME_LEN];

static status_led_t _status_led_green;
static status_led_t _status_led_red;

static uint32_t _time_safety_button_pressed;
static bool _new_safety_button_press;

static void status_led_init() {
	_status_led_green.gpio_p = LED0_GPIO;
	_status_led_green.pin = LED0_PIN;
	_status_led_green.period_us = 1000000;
	_status_led_green.length_us = 250000;
	_status_led_green.last_pulse = 0;

	_status_led_red.gpio_p = LED1_GPIO;
	_status_led_red.pin = LED1_PIN;
	_status_led_red.period_us = 500000;
	_status_led_red.length_us = 250000;
	_status_led_red.last_pulse = 0;
}

void safety_init() {
	_system_status.sensors.imu.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.imu.last_read = 0;
	_system_status.sensors.imu.count = 0;
	strncpy(_system_status.sensors.imu.name, "IMU", 24);

	_system_status.sensors.mag.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.mag.last_read = 0;
	_system_status.sensors.mag.count = 0;
	strncpy(_system_status.sensors.mag.name, "Magnatometer", 24);

	_system_status.sensors.sonar.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.sonar.last_read = 0;
	_system_status.sensors.sonar.count = 0;
	strncpy(_system_status.sensors.sonar.name, "Sonar", 24);

	_system_status.sensors.offboard_heartbeat.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.offboard_heartbeat.last_read = 0;
	_system_status.sensors.offboard_heartbeat.count = 0;
	strncpy(_system_status.sensors.offboard_heartbeat.name, "Offboard Heartbeat", 24);

	_system_status.sensors.offboard_control.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.offboard_control.last_read = 0;
	_system_status.sensors.offboard_control.count = 0;
	strncpy(_system_status.sensors.offboard_control.name, "Offboard Control", 24);

	_new_safety_button_press = false;
	_time_safety_button_pressed = 0;

	status_led_init();

	strncpy( mav_mode_names[0], "CUSTOM", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[1], "TEST", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[2], "AUTO", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[3], "GUIDED", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[4], "STABILIZE", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[5], "HIL", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[6], "MANUAL", MAV_MODE_NAME_LEN);
	strncpy( mav_mode_names[7], "ERROR!", MAV_MODE_NAME_LEN);

	strncpy( mav_state_names[0], "UNINIT", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[1], "BOOT", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[2], "CALIBRATE", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[3], "STANDBY", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[4], "ACTIVE", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[5], "CRITICAL", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[6], "EMERGENCY", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[7], "POWEROFF", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[8], "ERROR!", MAV_STATE_NAME_LEN);

	//TODO:
		//Barometer
		//Diff Pressure
}

bool safety_is_armed(void) {
	return _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY;
}

bool safety_request_state(uint8_t req_state) {
	bool success = false;
	bool change_state = false;

	switch( req_state ) {
		case MAV_STATE_BOOT: {
			if(_system_status.state == MAV_STATE_UNINIT)
				change_state = true;

			break;
		}
		case MAV_STATE_CALIBRATING: {
			if(_system_status.state == MAV_STATE_STANDBY)
				change_state = true;

			break;
		}
		case MAV_STATE_STANDBY: {
			if( !safety_is_armed() &&
			  ( ( _system_status.state == MAV_STATE_ACTIVE ) ||
			    ( _system_status.state == MAV_STATE_CRITICAL ) ||
			    ( _system_status.state == MAV_STATE_EMERGENCY ) ) ) {
				change_state = true;
			} else if ( _system_status.state == MAV_STATE_CALIBRATING ) {
				change_state = true;
			} else if ( _system_status.state == MAV_STATE_BOOT ) {
				change_state = true;
			}

			break;
		}
		case MAV_STATE_ACTIVE: {
			if(_system_status.state == MAV_STATE_STANDBY)
				change_state = true;

			break;
		}
		case MAV_STATE_CRITICAL: {
			if(_system_status.state == MAV_STATE_ACTIVE)
				change_state = true;

			break;
		}
		case MAV_STATE_EMERGENCY: {
			if( ( _system_status.state == MAV_STATE_ACTIVE ) ||
			  ( _system_status.state == MAV_STATE_CRITICAL ) )
				change_state = true;

			break;
		}
		default: {//MAV_STATE_UNINIT, MAV_STATE_BOOT, or MAV_STATE_POWEROFF should always hold state
			break;
		}
	}

	if( change_state ) {
		_system_status.state = req_state;
		success = true;
	}

	return success;
}

bool safety_request_arm(void) {
	bool result = false;
	bool throttle_check = false;
	bool mode_check = false;

	//Make sure the system is in the correct mode
	//TODO: May need more cases here to
	//TODO: Use the system_state_check() functions here!
	if( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED )
		mode_check = true;

	//Make sure low throttle is being provided
	if( ( ( _command_input.T == 0 ) ||
	  ( _command_input.input_mask & CMD_IN_IGNORE_THROTTLE) ) &&
	  ( _control_output.T == 0 ) )
		throttle_check = true;

	if(	( throttle_check ) &&
	  ( mode_check ) &&
	  ( _system_status.safety_button_status ) &&
	  ( _system_status.health == SYSTEM_HEALTH_OK ) &&
	  ( safety_request_state( MAV_STATE_ACTIVE ) ) ) {
		_system_status.mode |= MAV_MODE_FLAG_SAFETY_ARMED;	//ARM!

		mavlink_queue_broadcast_notice("[SAFETY] Mav armed!");

		result = true;

		//TODO: Make success beep here

	} else {
		char text_error[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] Arming denied: ";
		char text_reason[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

		if( !_system_status.safety_button_status) {
			strncpy(text_reason,
					 "safety engaged",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if( _system_status.health != SYSTEM_HEALTH_OK ) {
			strncpy(text_reason,
					 "sensor error",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			//TODO: Feedback on specific sensor
		} else if( _system_status.state != MAV_STATE_STANDBY ) {
			strncpy(text_reason,
					 "mav cannot arm in current state: ",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

			strncat(text_reason,
					mav_state_names[_system_status.state],
					MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if( !mode_check) {	//TODO: Check this error message works correctly
			strncpy(text_reason,
					 "cannot arm in mode: ",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

			int i = 0;
			for(i = 0; i < MAV_MODE_NUM_MODES; i++) {
				if( _system_status.mode & ( 1 << i) )
					break;
			}

			strncat(text_reason,
					mav_mode_names[i],
					MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if( !throttle_check ) {
			strncpy(text_reason, "high throttle", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		}

		strncat(text_error, text_reason, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		mavlink_queue_broadcast_error(text_error);

		//TODO: Make failure beep here
	}

	return result;
}

bool safety_request_disarm(void) {
	bool result = false;

	_system_status.mode &= ( 0xff ^ MAV_MODE_FLAG_SAFETY_ARMED );	//DISARM!

	mavlink_queue_broadcast_notice("[SAFETY] Mav disarmed!");

	if( safety_request_state( MAV_STATE_STANDBY ) ) {
		mavlink_queue_broadcast_notice("[SAFETY] Mav returned to standby state");
	} else {
		mavlink_queue_broadcast_error("[SAFETY] Unable to return to standby state!");
	}

	//TODO: Make success beep here

	result = true;

	return result;
}

static void safety_switch_update(uint32_t time_now) {
	if(_sensors.safety_button.status.new_data) {
		if(!_sensors.safety_button.state) {	//Inversed as a pull-up resistor is used for button detect
			_time_safety_button_pressed = time_now;
			_new_safety_button_press = true;
		} else {
			_new_safety_button_press = false;
		}

		//if( ( time_safety_button_released - time_safety_button_pressed ) > 1000000 )
		//	_system_status.safety_button_status = !_system_status.safety_button_status;	//Toggle arming safety

		_sensors.safety_button.status.new_data = false;
	}

	if(_new_safety_button_press && ( time_now - _time_safety_button_pressed ) > 1000000 ) {
		_system_status.safety_button_status = !_system_status.safety_button_status;	//Toggle arming safety
		_new_safety_button_press = false;

		//If the mav was armed and safety was switched off, disarm it
		if( !_system_status.safety_button_status && safety_is_armed() ) {
			safety_request_disarm();
		}
	}
}

static void status_led_do_pulse(status_led_t* led) {
	if( ( micros() - led->last_pulse ) > led->period_us )
		led->last_pulse = micros();

	if( ( micros() - led->last_pulse ) < led->length_us ) {
		digitalLo(led->gpio_p, led->pin);	//On
	} else {
		digitalHi(led->gpio_p, led->pin);	//Off
	}
}

static void status_led_update(void) {
	// Safety LED
	if( safety_is_armed() ) {
		digitalLo(_status_led_red.gpio_p, _status_led_red.pin);	//On
	} else if (_system_status.safety_button_status) {
		status_led_do_pulse(&_status_led_red);
	} else {
		digitalHi(_status_led_red.gpio_p, _status_led_red.pin);	//Off
	}

	//System heartbeat LED
	status_led_do_pulse(&_status_led_green);
}

void safety_update_sensor( timeout_status_t *sensor, uint32_t stream_count ) {
	sensor->last_read = micros();
	sensor->count++;

	if( ( sensor->health == SYSTEM_HEALTH_TIMEOUT ) && ( sensor->count > stream_count ) ) {
		sensor->health = SYSTEM_HEALTH_OK;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Connected: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 20) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	} else if( sensor->health == SYSTEM_HEALTH_UNKNOWN ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
	}
}

static void safety_check_sensor( timeout_status_t *sensor, uint32_t time_now, uint32_t timeout ) {
	if( ( sensor->health == SYSTEM_HEALTH_OK ) && ( ( time_now - sensor->last_read ) > timeout ) ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
		sensor->count = 0;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Timeout: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 18) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	}
}

bool safety_mode_set(uint8_t req_mode) {
	bool success = false;

	//Quick check to make sure the request is a valid supported mode
	if( ( req_mode == MAV_MODE_PREFLIGHT ) ||
	  ( req_mode == MAV_MODE_FLAG_STABILIZE_ENABLED ) ||
	  ( req_mode == MAV_MODE_FLAG_GUIDED_ENABLED ) ||
	  ( req_mode == MAV_MODE_FLAG_AUTO_ENABLED ) ) {
		_system_status.mode = req_mode | ( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY );
		success = true;
	}
	return success;
}

void safety_check_failsafe(void) {
	if( _system_status.health != SYSTEM_HEALTH_OK ) {
		//If flight-critical systems are down
		if( ( _system_status.sensors.imu.health != SYSTEM_HEALTH_OK ) ) {	//Emergency //TODO: More?
			safety_request_state( MAV_STATE_EMERGENCY );
			safety_request_disarm();
		} else {	//Critical
			safety_request_state( MAV_STATE_CRITICAL );
		}

		//Put the system into failsafe mode
		safety_mode_set(MAV_MODE_FLAG_AUTO_ENABLED);
	}
}

//TODO: Document state machine
static void system_mode_update(void) {
	//==-- System Mode State Machine
	if( _system_status.mode == MAV_MODE_PREFLIGHT ) {
		if ( _system_status.health == SYSTEM_HEALTH_OK )
			safety_mode_set( MAV_MODE_FLAG_GUIDED_ENABLED );
	} else if( _system_status.mode == MAV_MODE_GUIDED_DISARMED ) {
		if( _system_status.health != SYSTEM_HEALTH_OK )
			safety_mode_set( MAV_MODE_PREFLIGHT );
	} else if( _system_status.mode == MAV_MODE_GUIDED_ARMED ) {
			safety_check_failsafe();	//Check if system should failsafe
	} else if( _system_status.mode == MAV_MODE_AUTO_DISARMED ) {
		safety_mode_set( MAV_MODE_PREFLIGHT );	//Failsafe mode has been resolved
	} else if( _system_status.mode == MAV_MODE_AUTO_ARMED ) {
		//TODO: Something? Panic?
	}
}

static void safety_health_update(uint32_t time_now) {
	if( ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) &&
	  ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) &&
	  ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) ) {
		_system_status.health = SYSTEM_HEALTH_OK;
	} else {
		_system_status.health = SYSTEM_HEALTH_ERROR;
	}
}

void safety_run( uint32_t time_now ) {
	//Check sensors to ensure they are operating correctly
	safety_check_sensor( &_system_status.sensors.imu, time_now, get_param_uint(PARAM_SENSOR_IMU_TIMEOUT) );
	safety_check_sensor( &_system_status.sensors.offboard_heartbeat, time_now, get_param_uint(PARAM_SENSOR_OFFB_HRBT_TIMEOUT) );
	safety_check_sensor( &_system_status.sensors.offboard_control, time_now, get_param_uint(PARAM_SENSOR_OFFB_CTRL_TIMEOUT) );
	//TODO: Check timeouts for:
		//Compass
		//Sonar
		//Anything else?

	//Update the overall system health based on the individual sensors
	safety_health_update(time_now);

	//Check the safety switch to see if the user has pushed or released it
	safety_switch_update(time_now);

	//Make sure current system state and mode are valid, and handle changes
	system_mode_update();

	//Update LED flashes to match system state
	status_led_update();
}

#ifdef __cplusplus
}
#endif
