#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "fixvector3d.h"

//TODO: Common header file for defines like this
//#define BOARD_REV 2

// global variable declarations
//extern vector_t _accel;
//extern vector_t _gyro;
//extern float _imu_temperature;
//extern uint32_t _imu_time;
//extern bool _imu_ready;

//extern bool _diff_pressure_present;
//extern int16_t _diff_pressure;
//extern int16_t _temperature;

//extern bool _baro_present;
//extern int16_t _baro_pressure;
//extern int16_t _baro_temperature;

//extern bool _sonar_present;
//extern int16_t _sonar_range;
//extern uint32_t _sonar_time;

//TODO: Sensors should have a "ready" state for when the are calculated and ready to use

typedef struct {
	bool present;
	uint32_t start;			//Loop start time
	uint32_t end;			//Loop end time
	uint32_t dt;			//Time sepent this loop

	uint32_t counter;		//Times the data from this sensor has been collated
	uint32_t average_time;	//Sum of dt
	uint32_t max;			//Maximum dt so far
	uint32_t min;			//Minimum dt so far
} sensor_readings_time_t;

typedef struct {
	bool present;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

	v3d accel;		//Vector of accel data
	v3d gyro;		//Vector of gyro data
	fix16_t temperature;	//Sensor temperature reading

	uint32_t time;		//Time measured
	fix16_t accel_scale;	//Scale to correct raw accel data
	fix16_t gyro_scale;	//Scale to correct raw gyro data
} sensor_readings_imu_t;

typedef struct {
	bool present;
	int16_t pressure;		//Barometer reading
	int16_t temperature;	//Sensor temperature reading
} sensor_readings_barometer_t;

typedef struct {
	bool present;
	int16_t range;	//Measured range
	uint32_t time;	//Ping taken time
} sensor_readings_sonar_t;

typedef struct {
	sensor_readings_time_t time;
	sensor_readings_imu_t imu;
	sensor_readings_barometer_t baro;
	sensor_readings_sonar_t sonar;
} sensor_readings_t;

typedef enum {
	SENSOR_CAL_NONE = 0,
	SENSOR_CAL_GYRO = 1,
	SENSOR_CAL_MAG = 2,
	SENSOR_CAL_BARO = 4,
	SENSOR_CAL_RC = 8,		//Calibrate ESCs?
	SENSOR_CAL_ACCEL = 16,
	SENSOR_CAL_INTER = 32,	//TODO: Implement this
	SENSOR_CAL_ALL = 128
} sensor_calibration_request_t;

typedef struct {
	uint16_t count;
	fix16_t sum_x;
	fix16_t sum_y;
	fix16_t sum_z;
} sensor_calibration_gyro_data_t;

typedef struct {
	uint16_t count;
	int32_t z_up_sum;
	int32_t z_down_sum;
	int16_t z_up_av;
	int16_t z_down_av;
} sensor_calibration_gravity_data_t;

typedef struct {
	uint16_t count;
	int32_t sum_x;
	int32_t sum_y;
	int32_t sum_z;
	int32_t sum_t;

	uint16_t acc1G;
	fix16_t temp_scale;
	fix16_t temp_shift;
//	fix16_t sum_x;
//	fix16_t sum_y;
//	fix16_t sum_z;
//	fix16_t sum_t;
} sensor_calibration_accel_data_t;

typedef struct {
	sensor_calibration_gyro_data_t gyro;
	sensor_calibration_accel_data_t accel;
} sensor_calibration_data_t;

extern volatile bool imu_interrupt;
extern uint8_t _sensor_calibration;
extern sensor_readings_t _sensors;
extern sensor_calibration_data_t _sensor_cal_data;

// function declarations
void sensors_init(void);
bool sensors_read(void);
//void sensors_poll(void);
bool sensors_update(uint32_t time_us);
void sensors_calibrate(void);

//bool calibrate_acc(void);
//bool calibrate_gyro(void);
