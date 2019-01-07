#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"
#include "fixquat.h"
#include "fixvector3d.h"
//#include "breezystm32.h"

#include "drivers/drv_pwm.h"
#include "safety.h"

#define SENSOR_RC_MIDSTICK 1500
#define SENSOR_VMON_DIVIDER_NAZE32 \
	_fc_11 // XXX: Naze32's use 10K:1k which gives a value of 11.0

typedef struct {
	bool present;		// If the sensor is plugged in or not
	bool new_data;		// If there is new data ready
	uint32_t time_read; // Time measured
} sensor_status_t;

typedef struct {
	bool present;

	uint32_t start; // Loop start time
	uint32_t end;   // Loop end time
	uint32_t dt;	// Time sepent this loop

	uint32_t counter;	  // Times the data from this sensor has been collated
	uint32_t average_time; // Sum of dt
	uint32_t max;		   // Maximum dt so far
	uint32_t min;		   // Minimum dt so far

	uint32_t imu_time_read;

	int64_t rt_offset_ns;
	float rt_drift;
	uint64_t rt_ts_last;
	uint64_t rt_tc_last;
	uint32_t rt_sync_last;
} sensor_readings_clock_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} vector_int32_t;

// Hardware In The Loop State Input
typedef struct {
	sensor_status_t status;

	v3d accel; // Body-frame accelerations
	v3d gyro;  // Body-frame rotation rates
	v3d mag;   // Magnetometer reading vector

	fix16_t pressure_abs;  // Absolute preassure (mbar)
	fix16_t pressure_diff; // Differential preassure (mbar)
	fix16_t pressure_alt;  // Calculated altitude from pressure (m)

	fix16_t temperature; // Sensor temperature reading
} sensor_readings_hil_t;

// IMU
typedef struct {
	sensor_status_t status;

	vector_int32_t accel_raw;
	vector_int32_t gyro_raw;
	int16_t temp_raw;

	v3d accel;			 // Vector of accel data
	v3d gyro;			 // Vector of gyro data
	fix16_t temperature; // Sensor temperature reading

	fix16_t accel_scale; // Scale to correct raw accel data
	fix16_t gyro_scale;  // Scale to correct raw gyro data
} sensor_readings_imu_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update; // update rate in ms

	vector_int32_t raw;
	v3d scaled;
	qf16 q;
} sensor_readings_magnometer_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update; // update rate in ms

	int32_t raw_press; // Barometer reading
	int32_t raw_temp;  // Sensor temperature reading
} sensor_readings_barometer_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update; // update rate in ms

	int16_t range; // Measured range
} sensor_readings_sonar_t;

typedef struct {
	sensor_status_t status;

	v3d p;  // External position estimate
	qf16 q; // External attitude estimate
} sensor_readings_ext_pose_t;

typedef struct {
	sensor_status_t status;

	uint16_t raw[MAX_INPUTS];

	uint16_t p_r;
	uint16_t p_p;
	uint16_t p_y;
	uint16_t p_T;
	uint16_t p_m;

	fix16_t c_r;
	fix16_t c_p;
	fix16_t c_y;
	fix16_t c_T;

	compat_px4_main_mode_t c_m;
} sensor_readings_rc_input_t;

typedef struct {
	sensor_status_t status;

	bool arm_req_made;		 // Measured state
	uint32_t timer_start_us; // Measured range
} sensor_readings_rc_safety_toggle_t;

typedef struct {
	sensor_status_t status;

	bool state;			// Measured state
	uint32_t period_us; // Measured range

	uint32_t time_db_read; // Initial dounce read time
	uint32_t period_db_us; // Debounce period
	bool state_db;		   // State at debounce read
} sensor_readings_safety_button_t;

typedef struct {
	sensor_status_t status;

	uint16_t state_raw;		// Measured state raw
	fix16_t state_calc;		// Measured state
	fix16_t state_filtered; // Measured state filtered
	fix16_t precentage;		// Measured percentage

} sensor_readings_voltage_monitor_t;

typedef struct {
	sensor_readings_clock_t clock;
	sensor_readings_hil_t hil;
	sensor_readings_imu_t imu;
	sensor_readings_magnometer_t mag;
	sensor_readings_barometer_t baro;
	sensor_readings_sonar_t sonar;
	sensor_readings_ext_pose_t ext_pose;
	sensor_readings_rc_input_t rc_input;
	sensor_readings_rc_safety_toggle_t rc_safety_toggle;
	sensor_readings_safety_button_t safety_button;
	sensor_readings_voltage_monitor_t voltage_monitor;
} sensor_readings_t;

extern sensor_readings_t _sensors;

// function declarations
void sensors_init( void );
bool sensors_read( uint32_t time_us );

// void sensors_poll(void);
uint32_t sensors_clock_ls_get( void );		   // Get time at loop start
void sensors_clock_ls_set( uint32_t time_us ); // Set time at loop start
void sensors_clock_update( uint32_t time_us ); // Update the timing variable (used
// at the end of the loop)

int64_t sensors_clock_smooth_time_skew( int64_t tc, int64_t tn );
float sensors_clock_smooth_time_drift( float tc, float tn );
uint64_t sensors_clock_rt_get(
	void ); // Get the current time syncronized real time (good for logging)

uint32_t
	sensors_clock_imu_int_get( void ); // Get the time of the latest imu interrupt

void sensors_update_rc_cal( void );

// void sensors_poll(uint32_t time_us);

// bool sensors_update(uint32_t time_us);

#ifdef __cplusplus
}
#endif
