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

#define NUM_STATUS_BEEP_STEPS 4	//Number of beeps x2

system_status_t _system_status;
sensor_readings_t _sensors;
command_input_t _command_input;
control_output_t _control_output;
char mav_state_names[MAV_STATE_NUM_STATES][MAV_STATE_NAME_LEN];
char mav_mode_names[MAV_MODE_NUM_MODES][MAV_MODE_NAME_LEN];

static status_led_t _status_led_green;
static status_led_t _status_led_red;
static status_buzzer_t _status_buzzer;

static uint32_t _time_safety_arm_throttle_timeout;
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

static void status_buzzer_init() {
	//Buzzer
	_status_buzzer.gpio_p = GPIOA;
	_status_buzzer.pin = Pin_12;

	gpio_config_t safety_buzzer_cfg;
    safety_buzzer_cfg.pin = _status_buzzer.pin;
    safety_buzzer_cfg.mode = Mode_Out_PP;
    safety_buzzer_cfg.speed = Speed_2MHz;
    gpioInit(_status_buzzer.gpio_p, &safety_buzzer_cfg);

	_status_buzzer.mode = STATUS_BUZZER_QUIET;
	_status_buzzer.state = false;
	_status_buzzer.beep_steps = 0;			//Number of beeps to make
	_status_buzzer.period_high_us = 1912;	//C Major
	_status_buzzer.period_low_us = 3830;	//C Minor
	_status_buzzer.length_us = 200000;		//200ms beep length
	_status_buzzer.last_pulse = 0;			//Time last pulse started
	_status_buzzer.last_beep = 0;			//Time last pulse started

    digitalLo( _status_buzzer.gpio_p, _status_buzzer.pin );
}

void safety_init() {
	_system_status.sensors.imu.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.imu.last_read = 0;
	_system_status.sensors.imu.count = 0;
	_system_status.sensors.imu.param_stream_count = PARAM_SENSOR_IMU_STRM_COUNT;
	_system_status.sensors.imu.param_timeout = PARAM_SENSOR_IMU_TIMEOUT;
	strncpy(_system_status.sensors.imu.name, "IMU", 24);

	_system_status.sensors.mag.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.mag.last_read = 0;
	_system_status.sensors.mag.count = 0;
	_system_status.sensors.mag.param_stream_count = PARAM_SENSOR_MAG_STRM_COUNT;
	_system_status.sensors.mag.param_timeout = PARAM_SENSOR_MAG_TIMEOUT;
	strncpy(_system_status.sensors.mag.name, "Magnatometer", 24);

	_system_status.sensors.baro.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.baro.last_read = 0;
	_system_status.sensors.baro.count = 0;
	_system_status.sensors.baro.param_stream_count = PARAM_SENSOR_BARO_STRM_COUNT;
	_system_status.sensors.baro.param_timeout = PARAM_SENSOR_BARO_TIMEOUT;
	strncpy(_system_status.sensors.baro.name, "Barometer", 24);

	_system_status.sensors.sonar.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.sonar.last_read = 0;
	_system_status.sensors.sonar.count = 0;
	_system_status.sensors.sonar.param_stream_count = PARAM_SENSOR_SONAR_STRM_COUNT;
	_system_status.sensors.sonar.param_timeout = PARAM_SENSOR_SONAR_TIMEOUT;
	strncpy(_system_status.sensors.sonar.name, "Sonar", 24);

	_system_status.sensors.ext_pose.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.ext_pose.last_read = 0;
	_system_status.sensors.ext_pose.count = 0;
	_system_status.sensors.ext_pose.param_stream_count = PARAM_SENSOR_EXT_POSE_STRM_COUNT;
	_system_status.sensors.ext_pose.param_timeout = PARAM_SENSOR_EXT_POSE_TIMEOUT;
	strncpy(_system_status.sensors.ext_pose.name, "External Pose", 24);

	_system_status.sensors.offboard_heartbeat.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.offboard_heartbeat.last_read = 0;
	_system_status.sensors.offboard_heartbeat.count = 0;
	_system_status.sensors.offboard_heartbeat.param_stream_count = PARAM_SENSOR_OFFB_HRBT_STRM_COUNT;
	_system_status.sensors.offboard_heartbeat.param_timeout = PARAM_SENSOR_OFFB_HRBT_TIMEOUT;
	strncpy(_system_status.sensors.offboard_heartbeat.name, "Offboard Heartbeat", 24);

	_system_status.sensors.offboard_control.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.sensors.offboard_control.last_read = 0;
	_system_status.sensors.offboard_control.count = 0;
	_system_status.sensors.offboard_control.param_stream_count = PARAM_SENSOR_OFFB_CTRL_STRM_COUNT;
	_system_status.sensors.offboard_control.param_timeout = PARAM_SENSOR_OFFB_CTRL_TIMEOUT;
	strncpy(_system_status.sensors.offboard_control.name, "Offboard Control", 24);

	_time_safety_arm_throttle_timeout = 0;

	_new_safety_button_press = false;
	_time_safety_button_pressed = 0;

	strncpy( mav_state_names[0], "UNINIT", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[1], "BOOT", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[2], "CALIBRATE", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[3], "STANDBY", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[4], "ACTIVE", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[5], "CRITICAL", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[6], "EMERGENCY", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[7], "POWEROFF", MAV_STATE_NAME_LEN);
	strncpy( mav_state_names[8], "ERROR!", MAV_STATE_NAME_LEN);

	status_led_init();

	status_buzzer_init();
}

void status_buzzer_success(void) {
	_status_buzzer.mode = STATUS_BUZZER_SUCCESS;
	_status_buzzer.beep_steps = NUM_STATUS_BEEP_STEPS;
}

void status_buzzer_failure(void) {
	_status_buzzer.mode = STATUS_BUZZER_FAILURE;
	_status_buzzer.beep_steps = NUM_STATUS_BEEP_STEPS;
}

static void status_buzzer_update(void) {
	//If the system is in a failsafe mode, and the buzzer isn't set to make a different beep
	if( (_system_status.state == MAV_STATE_CRITICAL ) ||
	  (_system_status.state == MAV_STATE_EMERGENCY ) ) {
		//Not waiting for the delay makes a funky beep pattern, but it is unique
		if( _status_buzzer.mode == STATUS_BUZZER_QUIET ) {
			_status_buzzer.mode = STATUS_BUZZER_FAILSAFE;
			_status_buzzer.beep_steps = NUM_STATUS_BEEP_STEPS;
			_status_buzzer.last_beep = 0;
			_status_buzzer.last_pulse = 0;
		}
	}

	if( _status_buzzer.last_beep == 0 ) {
		//This is a fresh beep
		_status_buzzer.last_beep = micros();
	} else if( ( micros() - _status_buzzer.last_beep ) > _status_buzzer.length_us ) {
		_status_buzzer.beep_steps--;
		_status_buzzer.last_beep = micros();
	}

	if(_status_buzzer.beep_steps == 0) {
		_status_buzzer.mode = STATUS_BUZZER_QUIET;	//No need to reset state, as it only makes noise when it changes, not when high
		_status_buzzer.last_beep = 0;
		_status_buzzer.last_pulse = 0;
	}

	uint32_t beep_step_us = 0;

	switch(_status_buzzer.mode) {
		case STATUS_BUZZER_SUCCESS: {
			if(_status_buzzer.beep_steps == 4) {
				beep_step_us = _status_buzzer.period_low_us;
			} else if(_status_buzzer.beep_steps == 2) {
				beep_step_us = _status_buzzer.period_high_us;
			}

			break;
		}
		case STATUS_BUZZER_FAILURE: {
			if(_status_buzzer.beep_steps == 4) {
				beep_step_us = _status_buzzer.period_high_us;
			} else if(_status_buzzer.beep_steps == 2) {
				beep_step_us = _status_buzzer.period_low_us;
			}

			break;
		}
		case STATUS_BUZZER_FAILSAFE: {
			if(_status_buzzer.beep_steps % 2)
				beep_step_us = _status_buzzer.period_low_us;

			break;
		}
		default: {
			break;
		}
	}

	if( beep_step_us > 0 ) {
		if( ( micros() - _status_buzzer.last_pulse ) > beep_step_us ) {
			_status_buzzer.state = !_status_buzzer.state;
			_status_buzzer.last_pulse = micros();
		}

		if(_status_buzzer.state) {
			digitalHi(_status_buzzer.gpio_p, _status_buzzer.pin);
		} else {
			digitalLo(_status_buzzer.gpio_p, _status_buzzer.pin);
		}
	}
}

bool safety_is_armed(void) {
	return _system_status.arm_status;
}

bool safety_request_state(uint8_t req_state) {
	bool change_state = false;

	if(_system_status.state == req_state) {	//XXX: State request to same state, just say OK
		change_state = true;
	} else {
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
			case MAV_STATE_EMERGENCY: {	//Allow any request to put the mav into emergency mode
				//if( ( _system_status.state == MAV_STATE_ACTIVE ) ||
				//  ( _system_status.state == MAV_STATE_CRITICAL ) )
				change_state = true;

				break;
			}
			case MAV_STATE_POWEROFF: {	//Allows the mav to check it is safe to poweroff/reboot
				if( (_system_status.state == MAV_STATE_UNINIT) ||
					(_system_status.state == MAV_STATE_BOOT) ||
					(_system_status.state == MAV_STATE_STANDBY) )
					change_state = true;

				break;
			}
			default: {	// Other states cannot be requested
				break;
			}
		}

		if( change_state )
			_system_status.state = req_state;
	}

	return change_state;
}

bool safety_request_arm(void) {
	bool result = false;
	bool throttle_check = false;

	//Make sure low throttle is being provided
	if( ( _control_output.T == 0 ) &&
	  ( ( _command_input.T == 0 ) ||
	    ( _command_input.input_mask & CMD_IN_IGNORE_THROTTLE) ) )
		throttle_check = true;

	if(	( ( _system_status.safety_button_status ) || !_sensors.safety_button.status.present ) &&
	  ( _system_status.health == SYSTEM_HEALTH_OK ) &&
	  ( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED ) &&
	  ( throttle_check ) &&
	  ( safety_request_state( MAV_STATE_ACTIVE ) ) ) {
		_system_status.arm_status = true;	//ARM!

		_time_safety_arm_throttle_timeout = micros();	//Record down the arm time for throttle timeout

		mavlink_queue_broadcast_notice("[SAFETY] Mav armed!");

		result = true;

		status_buzzer_success();
	} else {
		char text_error[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SAFETY] Arming denied: ";
		char text_reason[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

		if( ( !_system_status.safety_button_status ) && ( _sensors.safety_button.status.present ) ) {
			strncpy(text_reason,
					 "safety engaged",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if( _system_status.health != SYSTEM_HEALTH_OK ) {
			strncpy(text_reason,
					 "sensor error ",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

			if(_sensors.imu.status.present && (_system_status.sensors.imu.health != SYSTEM_HEALTH_OK) ){
				strncpy(text_reason,
						 "(IMU)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else if(_sensors.mag.status.present && (_system_status.sensors.mag.health != SYSTEM_HEALTH_OK) ) {
				strncpy(text_reason,
						 "(mag)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else if(_sensors.baro.status.present && (_system_status.sensors.baro.health != SYSTEM_HEALTH_OK) ) {
				strncpy(text_reason,
						 "(baro)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else if(_sensors.sonar.status.present && (_system_status.sensors.sonar.health != SYSTEM_HEALTH_OK) ) {
				strncpy(text_reason,
						 "(sonar)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else if(_system_status.sensors.offboard_heartbeat.health != SYSTEM_HEALTH_OK) {
				strncpy(text_reason,
						 "(offb_hrbt)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else if(_system_status.sensors.offboard_control.health != SYSTEM_HEALTH_OK) {
				strncpy(text_reason,
						 "(offb_ctrl)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			} else {
				strncpy(text_reason,
						 "(unkown)",
						 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
			}
		} else if( _system_status.state != MAV_STATE_STANDBY ) {
			strncpy(text_reason,
					 "state: ",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

			strncat(text_reason,
					mav_state_names[_system_status.state],
					MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1);	//XXX: Stops string overflow warnings
		} else if( !( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED ) ) {	//TODO: Check this error message works correctly
			strncpy(text_reason,
					 "no guidance input",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if( !throttle_check ) {
			strncpy(text_reason, "high throttle", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		}

		strncat(text_error, text_reason, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1);	//XXX: Stops string overflow errors
		mavlink_queue_broadcast_error(text_error);

		status_buzzer_failure();
	}

	return result;
}

bool safety_request_disarm(void) {
	bool result = false;

	_system_status.arm_status = false;	//DISARM!

	_time_safety_arm_throttle_timeout = 0;

	mavlink_queue_broadcast_notice("[SAFETY] Mav disarmed!");

	if( safety_request_state( MAV_STATE_STANDBY ) ) {
		mavlink_queue_broadcast_notice("[SAFETY] Mav returned to standby state");
	} else {
		mavlink_queue_broadcast_error("[SAFETY] Unable to return to standby state!");
	}

	status_buzzer_success();

	result = true;	//XXX: Never fail a disarm request

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

		_sensors.safety_button.status.new_data = false;
	}

	if(_new_safety_button_press && ( time_now - _time_safety_button_pressed ) > 1000000 ) {
		_system_status.safety_button_status = !_system_status.safety_button_status;	//Toggle arming safety
		_new_safety_button_press = false;

		//If the mav was armed and safety was switched off, disarm it
		if( !_system_status.safety_button_status && safety_is_armed() ) {
			safety_request_disarm();
		}

		status_buzzer_success();
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

void safety_update_sensor( timeout_status_t *sensor ) {
	sensor->last_read = micros();
	sensor->count++;

	if( ( sensor->health == SYSTEM_HEALTH_TIMEOUT ) && ( sensor->count > get_param_uint(sensor->param_stream_count) ) ) {
		sensor->health = SYSTEM_HEALTH_OK;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Connected: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 20) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	} else if( sensor->health == SYSTEM_HEALTH_UNKNOWN ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
	}
}

static void safety_check_sensor( timeout_status_t *sensor, uint32_t time_now ) {
	if( ( sensor->health == SYSTEM_HEALTH_OK ) && ( ( time_now - sensor->last_read ) > get_param_uint(sensor->param_timeout) ) ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
		sensor->count = 0;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Timeout: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 18) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	}
}

void safety_check_failsafe(void) {
	if( _system_status.health != SYSTEM_HEALTH_OK ) {
		//If flight-critical systems are down
		if( ( _system_status.sensors.imu.health != SYSTEM_HEALTH_OK ) ) {	//Emergency //TODO: More?
			safety_request_state( MAV_STATE_EMERGENCY );
		} else {	//Critical
			safety_request_state( MAV_STATE_CRITICAL );
		}
	}
}

static void system_state_update(void) {
	//==-- System State Machine
	switch( _system_status.state ) {
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
			safety_check_failsafe();	//Check if system should failsafe

			break;
		}
		case MAV_STATE_CRITICAL: {
			break;
		}
		case MAV_STATE_EMERGENCY: {
			safety_request_disarm();

			break;
		}
		default: {
			safety_request_state(MAV_STATE_EMERGENCY);

			mavlink_queue_broadcast_error("[SAFETY] Emergency! Mav entered unknown state!");

			break;
		}
	}

	//==-- System Mode Reporting
	_system_status.mode = MAV_MODE_PREFLIGHT;

	//Report if armed
	if( safety_is_armed() )
		_system_status.mode |= MAV_MODE_FLAG_SAFETY_ARMED;

	//Report if mav is controlling itself
	if( _system_status.state == MAV_STATE_CRITICAL )
		_system_status.mode |= MAV_MODE_FLAG_AUTO_ENABLED;

	//Report offboard input status
	if( ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) &&
	  ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) )
		_system_status.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
}

static void safety_health_update(uint32_t time_now) {
	if( ( !_sensors.imu.status.present || ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) ) &&
	  ( !_sensors.mag.status.present || ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) ) &&
	  ( !_sensors.baro.status.present || ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) ) &&
	  ( !_sensors.sonar.status.present || ( _system_status.sensors.sonar.health == SYSTEM_HEALTH_OK ) ) &&
	  ( !_sensors.ext_pose.status.present || ( _system_status.sensors.ext_pose.health == SYSTEM_HEALTH_OK ) ) &&
	  ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) &&
	  ( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) ) {
		_system_status.health = SYSTEM_HEALTH_OK;
	} else {
		_system_status.health = SYSTEM_HEALTH_ERROR;
	}
}

static void safety_arm_throttle_timeout( uint32_t time_now ) {
	if( _time_safety_arm_throttle_timeout ) {	//If the timeout is active
		if( _command_input.T > 0 ) {
			_time_safety_arm_throttle_timeout = 0;	//We have recieved throttle input, disable timeout
		} else if( ( time_now - _time_safety_arm_throttle_timeout) > get_param_uint(PARAM_THROTTLE_TIMEOUT) ) {
			mavlink_queue_broadcast_error("[SAFETY] Throttle timeout, disarming!");
			safety_request_disarm();
		}
	}
}

void safety_run( uint32_t time_now ) {
	//Check sensors to ensure they are operating correctly
	safety_check_sensor( &_system_status.sensors.imu, time_now );
	safety_check_sensor( &_system_status.sensors.mag, time_now );
	safety_check_sensor( &_system_status.sensors.baro, time_now );
	safety_check_sensor( &_system_status.sensors.sonar, time_now );
	safety_check_sensor( &_system_status.sensors.ext_pose, time_now );
	safety_check_sensor( &_system_status.sensors.offboard_heartbeat, time_now );
	safety_check_sensor( &_system_status.sensors.offboard_control, time_now );

	//Update the overall system health based on the individual sensors
	safety_health_update(time_now);

	//Check the safety switch to see if the user has pushed or released it
	safety_switch_update(time_now);

	//Make sure current system state and mode are valid, and handle changes
	system_state_update();

	//Update LED flashes to match system state
	status_led_update();

	//Update buzzer to see if it should be making any noise
	status_buzzer_update();

	//Auto throttle timeout
	safety_arm_throttle_timeout(time_now);
}

#ifdef __cplusplus
}
#endif
