#pragma once

#include <mavlink_system.h>
#include <mavlink/mavlink_types.h>
#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"

//#include "mavlink.h"

//This needs to be 1 less, as we need to allow space for '\n'
//MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1
#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

typedef enum {
	//==-- System
	PARAM_BOARD_REVISION,
	PARAM_VERSION_FIRMWARE,
	PARAM_VERSION_SOFTWARE,
	PARAM_BAUD_RATE_0,
	//PARAM_BAUD_RATE_1,

	PARAM_TIMESYNC_ALPHA,

	//==-- Mavlink
	PARAM_SYSTEM_ID,
	PARAM_COMPONENT_ID,

	PARAM_STREAM_RATE_HEARTBEAT_0,
	PARAM_STREAM_RATE_SYS_STATUS_0,
	PARAM_STREAM_RATE_HIGHRES_IMU_0,
	PARAM_STREAM_RATE_ATTITUDE_0,
	PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0,
	PARAM_STREAM_RATE_ATTITUDE_TARGET_0,
	PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0,
	PARAM_STREAM_RATE_TIMESYNC_0,
	PARAM_STREAM_RATE_LOW_PRIORITY_0,
	/*
	PARAM_STREAM_RATE_HEARTBEAT_1,
	PARAM_STREAM_RATE_SYS_STATUS_1,
	PARAM_STREAM_RATE_HIGHRES_IMU_1,
	PARAM_STREAM_RATE_ATTITUDE_1,
	PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1,
	PARAM_STREAM_RATE_ATTITUDE_TARGET_1,
	PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1,
	PARAM_STREAM_RATE_TIMESYNC_1,
	PARAM_STREAM_RATE_LOW_PRIORITY_1,
	*/
	//==-- Sensors
	PARAM_SENSOR_IMU_CBRK,
	PARAM_SENSOR_MAG_CBRK,
	PARAM_SENSOR_BARO_CBRK,
	PARAM_SENSOR_SONAR_CBRK,
	PARAM_SENSOR_SAFETY_CBRK,

	PARAM_SENSOR_BARO_UPDATE,
	PARAM_SENSOR_SONAR_UPDATE,
	PARAM_SENSOR_MAG_UPDATE,

	PARAM_SENSOR_IMU_STRM_COUNT,
	PARAM_SENSOR_BARO_STRM_COUNT,
	PARAM_SENSOR_SONAR_STRM_COUNT,
	PARAM_SENSOR_MAG_STRM_COUNT,
	PARAM_SENSOR_OFFB_HRBT_STRM_COUNT,
	PARAM_SENSOR_OFFB_CTRL_STRM_COUNT,

	PARAM_SENSOR_IMU_TIMEOUT,
	PARAM_SENSOR_BARO_TIMEOUT,
	PARAM_SENSOR_SONAR_TIMEOUT,
	PARAM_SENSOR_MAG_TIMEOUT,
	PARAM_SENSOR_OFFB_HRBT_TIMEOUT,
	PARAM_SENSOR_OFFB_CTRL_TIMEOUT,

	//==-- Estimator
	PARAM_INIT_TIME,
	PARAM_EST_USE_ACC_COR,
	PARAM_EST_USE_MAT_EXP,
	PARAM_EST_USE_QUAD_INT,
	PARAM_FILTER_KP,
	PARAM_FILTER_KI,
	PARAM_GYRO_ALPHA,
	PARAM_ACC_ALPHA,
	PARAM_GYRO_X_BIAS,
	PARAM_GYRO_Y_BIAS,
	PARAM_GYRO_Z_BIAS,
	PARAM_ACC_X_BIAS,
	PARAM_ACC_Y_BIAS,
	PARAM_ACC_Z_BIAS,
	PARAM_ACC_X_SCALE_POS,
	PARAM_ACC_X_SCALE_NEG,
	PARAM_ACC_Y_SCALE_POS,
	PARAM_ACC_Y_SCALE_NEG,
	PARAM_ACC_Z_SCALE_POS,
	PARAM_ACC_Z_SCALE_NEG,
	PARAM_ACC_X_TEMP_COMP,
	PARAM_ACC_Y_TEMP_COMP,
	PARAM_ACC_Z_TEMP_COMP,

	//==-- Control
	PARAM_PID_ROLL_RATE_P,
	PARAM_PID_ROLL_RATE_I,
	PARAM_PID_ROLL_RATE_D,
	PARAM_MAX_ROLL_RATE,

	PARAM_PID_PITCH_RATE_P,
	PARAM_PID_PITCH_RATE_I,
	PARAM_PID_PITCH_RATE_D,
	PARAM_MAX_PITCH_RATE,

	PARAM_PID_YAW_RATE_P,
	PARAM_PID_YAW_RATE_I,
	PARAM_PID_YAW_RATE_D,
	PARAM_MAX_YAW_RATE,

	PARAM_PID_ROLL_ANGLE_P,
	PARAM_MAX_ROLL_ANGLE,

	PARAM_PID_PITCH_ANGLE_P,
	PARAM_MAX_PITCH_ANGLE,

	PARAM_PID_YAW_ANGLE_P,

	PARAM_MAX_COMMAND,
	PARAM_PID_TAU,

	//==-- Output
	PARAM_MOTOR_PWM_SEND_RATE,
	PARAM_MOTOR_PWM_IDLE,
	PARAM_MOTOR_PWM_MIN,
	PARAM_MOTOR_PWM_MAX,

	PARAM_DO_ESC_CAL,
	PARAM_FAILSAFE_THROTTLE,
	PARAM_THROTTLE_TIMEOUT,

	PARAM_MIXER,

	//==-- Number of Parameters
	PARAMS_COUNT
} param_id_t;

// type definitions
typedef struct {
	uint8_t version;
	uint16_t size;
	uint8_t magic_be;                       // magic number, should be 0xBE

	uint32_t values[PARAMS_COUNT];
	char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
	mavlink_message_type_t types[PARAMS_COUNT];

	uint8_t magic_ef;                       // magic number, should be 0xEF
	uint8_t chk;                            // XOR checksum
} params_t;

// global variable declarations
extern params_t _params;

// function declarations
void init_params(void);
void set_param_defaults(void);

bool read_params(void);
bool write_params(void);
void param_change_callback(param_id_t id);

mavlink_message_type_t get_param_type(param_id_t id);
void get_param_name(param_id_t id, char *name);
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

uint32_t get_param_uint(param_id_t id);
int32_t get_param_int(param_id_t id);
fix16_t get_param_fix16(param_id_t id);

bool set_param_uint(param_id_t id, uint32_t value);
bool set_param_int(param_id_t id, int32_t value);
bool set_param_fix16(param_id_t id, fix16_t value);

bool set_param_by_name_uint(const char name[PARAMS_NAME_LENGTH], uint32_t value);
bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value);
bool set_param_by_name_fix16(const char name[PARAMS_NAME_LENGTH], fix16_t value);
