#ifdef __cplusplus
extern "C" {
#endif

#include "breezystm32.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"

system_status_t _system_status;

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
}

void safety_update( timeout_status_t *sensor, uint32_t stream_count ) {
	sensor->last_read = micros();
	sensor->count++;

	if( ( sensor->health == SYSTEM_HEALTH_TIMEOUT ) && ( sensor->count > stream_count ) ) {
		sensor->health = SYSTEM_HEALTH_OK;

		//TODO: Needs to be buffered
		char text[50] = "[SENSOR] Connected: ";
		strncat(text, sensor->name, 29);
		mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
	} else if( sensor->health == SYSTEM_HEALTH_UNKNOWN ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
	}
}

static void safety_check( timeout_status_t *sensor, uint32_t time_now, uint32_t timeout ) {
	if( ( sensor->health == SYSTEM_HEALTH_OK ) && ( ( time_now - sensor->last_read ) > timeout ) ) {
		sensor->health = SYSTEM_HEALTH_TIMEOUT;
		sensor->count = 0;

		//TODO: Needs to be buffered
		char text[50] = "[SENSOR] Timeout: ";
		strncat(text, sensor->name, 31);
		mavlink_send_statustext_notice(MAV_SEVERITY_NOTICE, &text[0]);
	}
}

void safety_run( uint32_t time_now ) {
	safety_check(&_system_status.mavlink.offboard_control, time_now, 100000);	//TODO: Use params here
}

#ifdef __cplusplus
}
#endif
