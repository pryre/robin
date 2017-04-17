#ifdef __cplusplus
extern "C" {
#endif

#include "breezystm32.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"

system_status_t _system_status;

static status_led_t _status_led_green;
static status_led_t _status_led_red;

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

	status_led_init();

	//TODO:
		//Barometer
		//Diff Pressure
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

static void status_led_update() {
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
		mavlink_queue_notice_broadcast( &text[0] );
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
		mavlink_queue_notice_broadcast( &text[0] );
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
