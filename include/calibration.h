#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "fixvector3d.h"
#include "fixquat.h"
#include "fix16.h"
#include "breezystm32.h"

//RC CAL DEFINES
#define SENSOR_RC_CAL_MIN 0
#define SENSOR_RC_CAL_MID 1
#define SENSOR_RC_CAL_MAX 2

//XXX: Defined in MAV_CMD_PREFLIGHT_CALIBRATION
//Param1
#define CAL_CMD_GYRO 1
#define CAL_CMD_GYRO_TEMP 3
//Param2
#define CAL_CMD_MAG 1
//Param3
#define CAL_CMD_PRESSURE_GND 1
//Param4
#define CAL_CMD_RC 1
#define CAL_CMD_RC_TRIM 2
//Param5
#define CAL_CMD_ACCEL 1
#define CAL_CMD_ACCEL_LEVEL 2
#define CAL_CMD_ACCEL_TEMP 3
#define CAL_CMD_ACCEL_SIMPLE 4
//Param6
#define CAL_CMD_COMPASS_MOTOR 1
#define CAL_CMD_AIRPSEED 2
//Param7
#define CAL_CMD_ESC 1
#define CAL_CMD_BAROMETER 3

#define CAL_GYRO_HIGH_BIAS 200

typedef enum {
	CAL_NONE = 0,
	CAL_GYRO,
	CAL_MAG,
	CAL_GND_PRESSURE,
	CAL_RC,
	CAL_ACCEL,
	CAL_LEVEL_HORIZON,
	CAL_INTER,
	CAL_BARO,
	CAL_INVALID
} calibration_request_t;

typedef enum {
	CAL_RC_RANGE_INIT = 0,
	CAL_RC_RANGE_MIDDOWN,
	CAL_RC_RANGE_CORNERS,
	CAL_RC_RANGE_EXTREMES,
	CAL_RC_RANGE_DONE
} calibration_rc_range_t;

typedef struct {
	bool waiting;
	calibration_rc_range_t step;
	uint16_t ranges[8][4];
	bool rev[8];
} calibration_rc_range_data_t;

typedef struct {
	uint16_t count;
	fix16_t sum_x;
	fix16_t sum_y;
	fix16_t sum_z;
} calibration_gyro_data_t;

typedef enum {
	CAL_ACCEL_INIT = 0,
	CAL_ACCEL_Z_DOWN,
	CAL_ACCEL_Z_UP,
	CAL_ACCEL_Y_DOWN,
	CAL_ACCEL_Y_UP,
	CAL_ACCEL_X_DOWN,
	CAL_ACCEL_X_UP,
	CAL_ACCEL_DONE
} calibration_accel_rotations_t;

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

} calibration_gravity_data_t;

typedef struct {
	calibration_accel_rotations_t accel_cal_step;
	calibration_gravity_data_t data;

	bool waiting;

	uint16_t acc1G;
	fix16_t temp_scale;
	fix16_t temp_shift;
} calibration_accel_data_t;

typedef struct {
	calibration_gyro_data_t gyro;
	calibration_rc_range_data_t rc;
	calibration_accel_data_t accel;
} calibration_raw_data_t;

typedef struct {
	uint8_t type;
	calibration_raw_data_t data;

	uint8_t req_sysid;
	uint8_t req_compid;
} calibration_data_t;

extern calibration_data_t _calibration_data;

void calibration_init(void);
bool calibration_request(calibration_request_t req);
void calibration_run(void);
void calibration_done(void);

bool calibrate_accel(void);
bool calibrate_barometer(void);
bool calibrate_ground_pressure(void);
bool calibrate_gyro(void);
bool calibrate_interference(void);
bool calibrate_level_horizon(void);
bool calibrate_magnetometer(void);
bool calibrate_rc(void);

