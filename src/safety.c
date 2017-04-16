#ifdef __cplusplus
extern "C" {
#endif

#include "breezystm32.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"

system_status_t _system_status;
status_led_t _status_led;

static void status_led_init() {
	_status_led.is_on = false;
	_status_led.num_pulses = 0;
	_status_led.num_pulses_done = 0;
	_status_led.last_start = 0;
	_status_led.last_pulse = 0;
	_status_led.pulse_length = 200000;	//0.2s
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

	status_led_init();

	//TODO:
		//Barometer
		//Diff Pressure
}

//TODO: Should not be hard coded?
static void status_led_update() {
	//LED blinker
	// Single blink for unarmed, double blink for armed
	if( _system_status.mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY ) {
		_status_led.num_pulses = 2;
	} else {
		_status_led.num_pulses = 1;
	}

	if( ( micros() - _status_led.last_start ) > 1000000) {
		_status_led.num_pulses = 0;
		_status_led.last_start = micros();
	}

	if( _status_led.num_pulses_done < _status_led.num_pulses ) {
		if( ( micros() - _status_led.last_pulse ) > _status_led.pulse_length ) {
			if( _status_led.is_on ) {
				LED0_OFF;
			}

			if( ( micros() - _status_led.last_pulse ) > ( 2* _status_led.pulse_length ) ) {
				_status_led.last_pulse = micros();
				_status_led.num_pulses_done++;
			}
		} else {
			if( !_status_led.is_on )
				LED0_ON;
		}
	}
}

void safety_update_sensor( timeout_status_t *sensor, uint32_t stream_count ) {
	sensor->last_read = micros();
	sensor->count++;

	if( ( sensor->health == SYSTEM_HEALTH_TIMEOUT ) && ( sensor->count > stream_count ) ) {
		sensor->health = SYSTEM_HEALTH_OK;

		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Connected: ";
		strncat(text, sensor->name, (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 20) );	//Magic number is size of the message start
		//mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
		mavlink_queue_notice( &text[0] );
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
		mavlink_queue_notice( &text[0] );
	}
}

void safety_run( uint32_t time_now ) {
	safety_check( &_system_status.mavlink.offboard_control, time_now, get_param_int(PARAM_SENSOR_OFFB_CTRL_TIMEOUT) );

	status_led_update();

	//TODO: Need to do more checks here!
}

#ifdef __cplusplus
}
#endif
