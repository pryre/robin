#ifdef __cplusplus
extern "C" {
#endif

#include "breezystm32.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "controller.h"

system_status_t _system_status;
sensor_readings_t _sensors;
command_input_t _command_input;
control_output_t _control_output;

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
	_system_status.mavlink.offboard_heartbeat.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.mavlink.offboard_heartbeat.last_read = 0;
	_system_status.mavlink.offboard_heartbeat.count = 0;
	strncpy(_system_status.mavlink.offboard_heartbeat.name, "Offboard Heartbeat", 24);

	_system_status.mavlink.offboard_control.health = SYSTEM_HEALTH_UNKNOWN;
	_system_status.mavlink.offboard_control.last_read = 0;
	_system_status.mavlink.offboard_control.count = 0;
	strncpy(_system_status.mavlink.offboard_control.name, "Offboard Control", 24);

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

	_new_safety_button_press = false;
	_time_safety_button_pressed = 0;

	status_led_init();

	//TODO:
		//Barometer
		//Diff Pressure
}

bool safety_request_arm(void) {
	bool result = false;
	bool base_systems_health = false;
	bool throttle_check = false;

	if( ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) &&
		( _system_status.mavlink.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) &&
		( _system_status.mavlink.offboard_control.health == SYSTEM_HEALTH_OK ) )
		base_systems_health = true;
		//TODO: Should there be any other checks here?
		//	Check to see if all inputs are good to go

	if( ( _control_output.T == 0 ) &&
		( ( _command_input.T == 0 ) ||
		  _command_input.input_mask & CMD_IN_IGNORE_THROTTLE) )
		throttle_check = true;

	if(	throttle_check &&
		base_systems_health &&
	    _system_status.safety_button_status &&
	    ( _system_status.state == MAV_STATE_STANDBY ) ) {

		_system_status.mode |= MAV_MODE_FLAG_SAFETY_ARMED;	//ARM!
		_system_status.state = MAV_STATE_ACTIVE;

		mavlink_queue_broadcast_notice("[SAFETY] Mav armed!");

		//TODO: Make success beep here

		result = true;
	} else {
		//TODO: Make failure beep here

		if( _system_status.state != MAV_STATE_STANDBY ) {
			mavlink_queue_broadcast_error("[SAFETY] Arming denied: mav not in standby state");
		} else if( !_system_status.safety_button_status) {
			mavlink_queue_broadcast_error("[SAFETY] Arming denied: safety engaged");
		} else if( !base_systems_health ) {
			mavlink_queue_broadcast_error("[SAFETY] Arming denied: sensor error");
		} else if( !throttle_check ) {
			mavlink_queue_broadcast_error("[SAFETY] Arming denied: high throttle");
		}
	}

	return result;
}

bool safety_request_disarm(void) {
	bool result = false;

	_system_status.mode &= !MAV_MODE_FLAG_SAFETY_ARMED;	//DISARM!
	_system_status.state = MAV_STATE_STANDBY;

	mavlink_queue_broadcast_notice("[SAFETY] Mav disarmed!");

	//TODO: Make system beep here as well

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
		if( !_system_status.safety_button_status &&
			(_system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY) ) {
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
	if( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY ) {
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

static void safety_check( timeout_status_t *sensor, uint32_t time_now, uint32_t timeout ) {
	if( ( sensor->health == SYSTEM_HEALTH_OK ) && ( ( time_now - sensor->last_read ) > timeout ) ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
		sensor->count = 0;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Timeout: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 18) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_broadcast_notice( &text[0] );
	}
}

void safety_run( uint32_t time_now ) {
	safety_check( &_system_status.sensors.imu, time_now, get_param_uint(PARAM_SENSOR_IMU_TIMEOUT) );

	safety_check( &_system_status.mavlink.offboard_heartbeat, time_now, get_param_uint(PARAM_SENSOR_OFFB_HRBT_TIMEOUT) );
	safety_check( &_system_status.mavlink.offboard_control, time_now, get_param_uint(PARAM_SENSOR_OFFB_CTRL_TIMEOUT) );

	safety_switch_update(time_now);

	status_led_update();

	//TODO: Need to do more checks here!
}

#ifdef __cplusplus
}
#endif
