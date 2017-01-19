#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "params.h"
#include "mixer.h"
/*
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_stream.h"
*/

// global variable definitions
params_t _params;

// local function definitions
static void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	_params.types[id] = PARAM_TYPE_INT32;
}

static void init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value) {
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);

	union {
		float f;
		uint32_t i;
	} u;

	u.f = value;

	_params.values[id] = u.i;
	_params.types[id] = PARAM_TYPE_FLOAT;
}

// function definitions
void init_params(void) {
	initEEPROM();
	if (!read_params())	{
		set_param_defaults();
		write_params();
	}

	/* TODO:
	for (uint16_t id = 0; id < PARAMS_COUNT; id++)
		param_change_callback((param_id_t) id);
	*/
}

void set_param_defaults(void) {
	//==-- System
	init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 5);
	init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600);

	//==-- Mavlink
	init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1);
	init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1);

	init_param_int(PARAM_STREAM_ATTITUDE_RATE, "STRM_ATTITUDE", 100);
	init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 500);
	init_param_int(PARAM_STREAM_MAG_RATE, "STRM_MAG", 0);
	init_param_int(PARAM_STREAM_BARO_RATE, "STRM_BARO", 0);
	init_param_int(PARAM_STREAM_SONAR_RATE, "STRM_SONAR", 0);
	init_param_int(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE, "STRM_SERVO", 50);

	//==-- Sensors
	init_param_int(PARAM_DIFF_PRESS_UPDATE, "DIFF_PRESS_UP", 0); // us
	init_param_int(PARAM_BARO_UPDATE, "BARO_UPDATE", 0);
	init_param_int(PARAM_SONAR_UPDATE, "SONAR_UPDATE", 0);
	init_param_int(PARAM_MAG_UPDATE, "MAG_UPDATE", 20000);

	//==-- Estimator
	init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // ms
	init_param_float(PARAM_FILTER_KP, "FILTER_KP", 1.0f);
	init_param_float(PARAM_FILTER_KI, "FILTER_KI", 0.1f);
	init_param_float(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", 0.6f);
	init_param_float(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", 0.6f);
	init_param_int(PARAM_STREAM_ADJUSTED_GYRO, "STRM_ADJUST_GYRO", 0);
	init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f);
	init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f);
	init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f);
	init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0.0f);
	init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0.0f);
	init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0.0f);
	init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0.0f);
	init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0.0f);
	init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0.0f);

	//==-- Control
	init_param_float(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 0.070f);
	init_param_float(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", 0.00001f);
	init_param_float(PARAM_MAX_ROLL_RATE, "MAX_ROLL_RATE", 3.14159f);

	init_param_float(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 0.070f);
	init_param_float(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", 0.00001f);
	init_param_float(PARAM_MAX_PITCH_RATE, "MAX_PITCH_RATE", 3.14159f);

	init_param_float(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0.025f);
	init_param_float(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0.0f);
	init_param_float(PARAM_MAX_YAW_RATE, "MAX_YAW_RATE", 6.283f);

	init_param_float(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 0.15f);
	init_param_float(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0.0f);
	init_param_float(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0.07f);
	init_param_float(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", 0.786f);

	init_param_float(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 0.15f);
	init_param_float(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0.0f);
	init_param_float(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0.07f);
	init_param_float(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", 0.786);

	init_param_float(PARAM_PID_TAU, "PID_TAU", 0.05f);
	init_param_int(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000);

	//==-- Output
	init_param_int(PARAM_MIXER, "MIXER", QUADCOPTER_PLUS);

	init_param_int(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_UPDATE", 400);
	init_param_int(PARAM_MOTOR_IDLE_PWM, "MOTOR_IDLE_PWM", 1150);
	init_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true);
}

bool read_params(void) {
	return readEEPROM();
}

bool write_params(void) {
	return writeEEPROM(true);
}

/* TODO:
void param_change_callback(param_id_t id)
{
  switch (id)
  {
  case PARAM_SYSTEM_ID:
    mavlink_system.sysid = get_param_int(PARAM_SYSTEM_ID);
    break;
  case PARAM_STREAM_HEARTBEAT_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_HEARTBEAT, get_param_int(PARAM_STREAM_HEARTBEAT_RATE));
    break;

  case PARAM_STREAM_ATTITUDE_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_ATTITUDE, get_param_int(PARAM_STREAM_ATTITUDE_RATE));
    break;

  case PARAM_STREAM_IMU_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_IMU, get_param_int(PARAM_STREAM_IMU_RATE));
    break;
  case PARAM_STREAM_AIRSPEED_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_DIFF_PRESSURE, get_param_int(PARAM_STREAM_AIRSPEED_RATE));
    break;
  case PARAM_STREAM_SONAR_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_SONAR, get_param_int(PARAM_STREAM_SONAR_RATE));
    break;
  case  PARAM_STREAM_BARO_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_BARO, get_param_int(PARAM_STREAM_BARO_RATE));
    break;

  case PARAM_STREAM_SERVO_OUTPUT_RAW_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW, get_param_int(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE));
    break;
  case PARAM_STREAM_RC_RAW_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_RC_RAW, get_param_int(PARAM_STREAM_RC_RAW_RATE));
    break;
  default:
    // no action needed for this parameter
    break;
  }
}
*/
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]) {
	for (uint16_t id = 0; id < PARAMS_COUNT; id++) {
		bool match = true;

		for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++) {
			// compare each character
			if (name[i] != _params.names[id][i])
			{
				match = false;
				break;
			}

			// stop comparing if end of string is reached
			if (_params.names[id][i] == '\0')
				break;
		}

		if (match)
			return (param_id_t) id;
	}

	return PARAMS_COUNT;
}

int get_param_int(param_id_t id) {
	return _params.values[id];
}

float get_param_float(param_id_t id) {
	union {
		float f;
		uint32_t i;
	} u;

	u.i = _params.values[id];

	return u.f;
}

char *get_param_name(param_id_t id) {
	return _params.names[id];
}

param_type_t get_param_type(param_id_t id) {
	return _params.types[id];
}

bool set_param_int(param_id_t id, int32_t value) {
	if (id < PARAMS_COUNT && value != _params.values[id]) {
		_params.values[id] = value;
		param_change_callback(id);
		//TODO: mavlink_send_param(id);
		return true;
	}
	return false;
}

bool set_param_float(param_id_t id, float value) {
	union {
		float f;
		uint32_t i;
	} u;

	u.f = value;

  return set_param_int(id, u.i);
}

bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value) {
	param_id_t id = lookup_param_id(name);
	return set_param_int(id, value);
}

bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value) {
	union {
		float f;
		uint32_t i;
	} u;

	u.f = value;
	return set_param_by_name_int(name, u.i);
}
