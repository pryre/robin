#include <fix16.h>
#include <stdbool.h>

#include "drivers/drv_pwm.h"

bool drv_sensors_i2c_init( void ) {
	return false;
}

bool drv_sensors_imu_init( fix16_t* scale_accel, fix16_t* scale_gyro ) {
	return false;
}

bool drv_sensors_mag_init( void ) {
	return false;
}

bool drv_sensors_baro_init( void ) {
	return false;
}

bool drv_sensors_sonar_init( void ) {
	return false;
}

bool drv_sensors_rc_input_init( void ) {
	return false;
}

bool drv_sensors_safety_button_init( void ) {
	return false;
}

bool drv_sensors_battery_monitor_init( void ) {
	return false;
}

bool drv_sensors_i2c_job_queued( void ) {
	return false;
}

void drv_sensors_i2c_clear( void ) {
}

bool drv_sensors_i2c_read( uint32_t time_us ) {
	return false;
}

void drv_sensors_rc_input_read( uint16_t* readings ) {
	for ( int i = 0; i < MAX_INPUTS; i++ ) {
		readings[i] = pwmRead( i );
	}
}

bool drv_sensors_safety_button_read( void ) {
	return false;
}

uint16_t drv_sensors_battery_monitor_read( void ) {
	return 0;
}
