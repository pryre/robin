#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "fixvector3d.h"
#include "fixquat.h"
#include "fix16.h"
#include "breezystm32.h"

#include "safety.h"

//RC CAL DEFINES
#define SENSOR_RC_CAL_MIN 0
#define SENSOR_RC_CAL_MID 1
#define SENSOR_RC_CAL_MAX 2

#define SENSOR_RC_MIDSTICK 1500

//XXX: Defined in MAV_CMD_PREFLIGHT_CALIBRATION
//Param1
#define SENSOR_CAL_CMD_GYRO 1
#define SENSOR_CAL_CMD_GYRO_TEMP 3
//Param2
#define SENSOR_CAL_CMD_MAG 1
//Param3
#define SENSOR_CAL_CMD_PRESSURE_GND 1
//Param4
#define SENSOR_CAL_CMD_RC 1
#define SENSOR_CAL_CMD_RC_TRIM 2
//Param5
#define SENSOR_CAL_CMD_ACCEL 1
#define SENSOR_CAL_CMD_ACCEL_LEVEL 2
#define SENSOR_CAL_CMD_ACCEL_TEMP 3
#define SENSOR_CAL_CMD_ACCEL_SIMPLE 4
//Param6
#define SENSOR_CAL_CMD_COMPASS_MOTOR 1
#define SENSOR_CAL_CMD_AIRPSEED 2
//Param7
#define SENSOR_CAL_CMD_ESC 1
#define SENSOR_CAL_CMD_BAROMETER 3

#define SENSOR_VMON_DIVIDER_NAZE32 _fc_11	//XXX: Naze32's use 10K:1k which gives a value of 11.0

typedef struct {
	bool present;		//If the sensor is plugged in or not
	bool new_data;	//If there is new data ready
	uint32_t time_read;	//Time measured
} sensor_status_t;

typedef struct {
	bool present;

	uint32_t start;			//Loop start time
	uint32_t end;			//Loop end time
	uint32_t dt;			//Time sepent this loop

	uint32_t counter;		//Times the data from this sensor has been collated
	uint32_t average_time;	//Sum of dt
	uint32_t max;			//Maximum dt so far
	uint32_t min;			//Minimum dt so far

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

typedef struct {
	sensor_status_t status;

    vector_int32_t accel_raw;
    vector_int32_t gyro_raw;
    int16_t temp_raw;

	v3d accel;		//Vector of accel data
	v3d gyro;		//Vector of gyro data
	fix16_t temperature;	//Sensor temperature reading

	fix16_t accel_scale;	//Scale to correct raw accel data
	fix16_t gyro_scale;	//Scale to correct raw gyro data
} sensor_readings_imu_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update;	//update rate in ms

	vector_int32_t raw;
	v3d scaled;
	qf16 q;
} sensor_readings_magnometer_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update;	//update rate in ms

	int32_t raw_press;		//Barometer reading
	int32_t raw_temp;	//Sensor temperature reading
} sensor_readings_barometer_t;

typedef struct {
	sensor_status_t status;
	uint32_t period_update;	//update rate in ms

	int16_t range;	//Measured range
} sensor_readings_sonar_t;

typedef struct {
	sensor_status_t status;

	v3d p;	//External position estimate
	qf16 q;	//External attitude estimate
} sensor_readings_ext_pose_t;

typedef struct {
	sensor_status_t status;

	v3d torque;	//External torque input
} sensor_readings_torque_comp_t;

typedef struct {
	sensor_status_t status;

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

	bool arm_req_made;			//Measured state
	uint32_t timer_start_us;	//Measured range
} sensor_readings_rc_safety_toggle_t;

typedef struct {
	sensor_status_t status;

	GPIO_TypeDef *gpio_p;
	uint16_t pin;

	bool state;			//Measured state
	uint32_t period_us;	//Measured range

	uint32_t time_db_read;	//Initial dounce read time
	uint32_t period_db_us;	//Debounce period
	bool state_db;			//State at debounce read
} sensor_readings_safety_button_t;

typedef struct {
	sensor_status_t status;

	GPIO_TypeDef *gpio_p;
	uint16_t pin;

	uint16_t state_raw;		//Measured state raw
	fix16_t state_calc;		//Measured state
	fix16_t state_filtered;	//Measured state filtered
	fix16_t precentage;		//Measured percentage

} sensor_readings_voltage_monitor_t;

typedef struct {
	sensor_readings_clock_t clock;
	sensor_readings_imu_t imu;
	sensor_readings_magnometer_t mag;
	sensor_readings_barometer_t baro;
	sensor_readings_sonar_t sonar;
	sensor_readings_ext_pose_t ext_pose;
	sensor_readings_torque_comp_t torque_comp;
	sensor_readings_rc_input_t rc_input;
	sensor_readings_rc_safety_toggle_t rc_safety_toggle;
	sensor_readings_safety_button_t safety_button;
	sensor_readings_voltage_monitor_t voltage_monitor;
} sensor_readings_t;

typedef enum {
	SENSOR_CAL_NONE = 0,
	SENSOR_CAL_GYRO,
	SENSOR_CAL_MAG,
	SENSOR_CAL_GND_PRESSURE,
	SENSOR_CAL_RC,
	SENSOR_CAL_ACCEL,
	SENSOR_CAL_LEVEL_HORIZON,
	SENSOR_CAL_INTER,
	SENSOR_CAL_BARO,
	SENSOR_CAL_INVALID
} sensor_calibration_request_t;

typedef enum {
	SENSOR_CAL_RC_RANGE_INIT = 0,
	SENSOR_CAL_RC_RANGE_MIDDOWN,
	SENSOR_CAL_RC_RANGE_CORNERS,
	SENSOR_CAL_RC_RANGE_EXTREMES,
	SENSOR_CAL_RC_RANGE_DONE
} sensor_calibration_rc_range_t;

typedef struct {
	bool waiting;
	sensor_calibration_rc_range_t step;
	uint16_t rc_ranges[8][4];
	bool rc_rev[8];
} sensor_calibration_rc_range_data_t;

typedef struct {
	uint16_t count;
	fix16_t sum_x;
	fix16_t sum_y;
	fix16_t sum_z;
} sensor_calibration_gyro_data_t;

typedef enum {
	SENSOR_CAL_ACCEL_INIT = 0,
	SENSOR_CAL_ACCEL_Z_DOWN,
	SENSOR_CAL_ACCEL_Z_UP,
	SENSOR_CAL_ACCEL_Y_DOWN,
	SENSOR_CAL_ACCEL_Y_UP,
	SENSOR_CAL_ACCEL_X_DOWN,
	SENSOR_CAL_ACCEL_X_UP,
	SENSOR_CAL_ACCEL_DONE
} sensor_calibration_accel_rotations_t;

typedef struct {
	uint16_t count;

	int32_t t_sum;
	int32_t x_sum;
	int32_t y_sum;
	int32_t z_sum;

	int32_t t_av_sum;

	int32_t x_flat_av_sum;
	int32_t x_up_av;
	int32_t x_down_av;

	int32_t y_flat_av_sum;
	int32_t y_up_av;
	int32_t y_down_av;

	int32_t z_flat_av_sum;
	int32_t z_up_av;
	int32_t z_down_av;

} sensor_calibration_gravity_data_t;

typedef struct {
	sensor_calibration_accel_rotations_t accel_cal_step;
	sensor_calibration_gravity_data_t data;

	bool waiting;

	uint16_t acc1G;
	fix16_t temp_scale;
	fix16_t temp_shift;
} sensor_calibration_accel_data_t;

typedef struct {
	sensor_calibration_gyro_data_t gyro;
	sensor_calibration_rc_range_data_t rc;
	sensor_calibration_accel_data_t accel;
} sensor_calibration_data_t;

typedef struct {
	uint8_t type;
	sensor_calibration_data_t data;

	uint8_t req_sysid;
	uint8_t req_compid;
} sensor_calibration_t;

extern sensor_readings_t _sensors;
extern sensor_calibration_t _sensor_calibration;

// function declarations
void sensors_init_imu(void);
void sensors_deinit_imu(void);
void sensors_init_internal(void);
void sensors_init_external(void);
void sensors_cal_init(void);
bool sensors_request_cal(sensor_calibration_request_t req);
bool sensors_read(void);
bool i2c_job_queued(void);

//void sensors_poll(void);
uint32_t sensors_clock_ls_get(void);	//Get time at loop start
void sensors_clock_ls_set(uint32_t time_us);	//Set time at loop start
void sensors_clock_update(uint32_t time_us);	//Update the timing variable (used at the end of the loop)

int64_t sensors_clock_smooth_time_skew(int64_t tc, int64_t tn);
float sensors_clock_smooth_time_drift(float tc, float tn);
uint64_t sensors_clock_rt_get(void);	//Get the current time syncronized real time (good for logging)

uint32_t sensors_clock_imu_int_get(void);	//Get the time of the latest imu interrupt

void sensors_update_rc_cal(void);

void sensors_poll(uint32_t time_us);

bool sensors_update(uint32_t time_us);
