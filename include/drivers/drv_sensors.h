#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <fix16.h>
#include <stdbool.h>

//extern volatile uint32_t _imu_time_ready;

bool drv_sensors_i2c_init( void );
bool drv_sensors_imu_init( uint32_t i2c, fix16_t* scale_accel, fix16_t* scale_gyro );
void drv_sensors_imu_configure_int(void ( *functionPtr )( void ));
bool drv_sensors_mag_init( uint32_t i2c );
bool drv_sensors_baro_init( uint32_t i2c );
bool drv_sensors_sonar_init( void );
bool drv_sensors_rc_input_init( void );
bool drv_sensors_safety_button_init( void );
bool drv_sensors_battery_monitor_init( void );

bool drv_sensors_i2c_job_queued( void );
void drv_sensors_i2c_clear( void );
bool drv_sensors_i2c_read( uint32_t time_us );

bool drv_sensors_rc_input_read( uint16_t* readings );
bool drv_sensors_rc_rssi_read( uint16_t* reading );
bool drv_sensors_safety_button_read( void );
uint16_t drv_sensors_battery_monitor_read( void );

#ifdef __cplusplus
}
#endif
