#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "params.h"
#include "mixer.h"
#include "fix16.h"

// global variable definitions
params_t _params;

// local function definitions
static void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	_params.types[id] = PARAM_TYPE_INT32;
}

static void init_param_fix16(param_id_t id, char name[PARAMS_NAME_LENGTH], fix16_t value) {
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);

	union {
		fix16_t f;
		uint32_t i;
	} u;

	u.f = value;

	_params.values[id] = u.i;
	_params.types[id] = PARAM_TYPE_FIX16;
}

// function definitions
void init_params(void) {
	initEEPROM();

	if (!read_params())	{
		set_param_defaults();
		write_params();
	}

	/* TODO: More to do with the live params
	for (uint16_t id = 0; id < PARAMS_COUNT; id++)
		param_change_callback((param_id_t) id);
	*/
}

void set_param_defaults(void) {
	//==-- System
	init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 5);
	init_param_int(PARAM_VERSION_FIRMWARE, "FW_VERSION", 1);
	init_param_int(PARAM_VERSION_SOFTWARE, "SW_VERSION", 1);
	init_param_int(PARAM_BAUD_RATE_0, "BAUD_RATE_0", 921600);	//Set baud rate to 0 to disable a comm port
	init_param_int(PARAM_BAUD_RATE_1, "BAUD_RATE_1", 0);	//TODO: Check if this works
	init_param_int(PARAM_TIMESYNC_ALPHA, "TIMESYNC_ALPHA", fix16_from_float(0.8f));

	//==-- Mavlink
	init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1);
	init_param_int(PARAM_COMPONENT_ID, "COMP_ID", 1);

	init_param_int(PARAM_STREAM_RATE_HEARTBEAT_0, "STRM_HRTBT_0", 1000000);
	init_param_int(PARAM_STREAM_RATE_SYS_STATUS_0, "STRM_SYS_STAT_0", 5000000);
	init_param_int(PARAM_STREAM_RATE_HIGHRES_IMU_0, "STRM_HR_IMU_0", 10000);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_0, "STRM_ATT_0", 0);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0, "STRM_ATT_Q_0", 20000);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_TARGET_0, "STRM_ATT_T_0", 20000);
	init_param_int(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0, "STRM_SRV_OUT_0", 100000);
	init_param_int(PARAM_STREAM_RATE_TIMESYNC_0, "STRM_TIMESYNC_0", 100000);
	init_param_int(PARAM_STREAM_RATE_LOW_PRIORITY_0, "STRM_LPQ_0", 10000);

	init_param_int(PARAM_STREAM_RATE_HEARTBEAT_1, "STRM_HRTBT_1", 1000000);
	init_param_int(PARAM_STREAM_RATE_SYS_STATUS_1, "STRM_SYS_STAT_1", 5000000);
	init_param_int(PARAM_STREAM_RATE_HIGHRES_IMU_1, "STRM_HR_IMU_1", 10000);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_1, "STRM_ATT_1", 0);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1, "STRM_ATT_Q_1", 20000);
	init_param_int(PARAM_STREAM_RATE_ATTITUDE_TARGET_1, "STRM_ATT_T_1", 20000);
	init_param_int(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1, "STRM_SRV_OUT_1", 100000);
	init_param_int(PARAM_STREAM_RATE_TIMESYNC_1, "STRM_TIMESYNC_1", 100000);
	init_param_int(PARAM_STREAM_RATE_LOW_PRIORITY_1, "STRM_LPQ_1", 10000);

	//==-- Sensors
	//All params here in us
	//TODO: Double check which is used here
	init_param_int(PARAM_SENSOR_DIFF_PRESS_UPDATE, "UPDATE_DIFF_P", 0);
	init_param_int(PARAM_SENSOR_BARO_UPDATE, "UPDATE_BARO", 0);
	init_param_int(PARAM_SENSOR_SONAR_UPDATE, "UPDATE_SONAR", 0);
	init_param_int(PARAM_SENSOR_MAG_UPDATE, "UPDATE_MAG", 0);
	init_param_int(PARAM_SENSOR_OFFB_CTRL_UPDATE, "UPDATE_OFFB_CTRL", 0);

	init_param_int(PARAM_SENSOR_DIFF_PRESS_TIMEOUT, "TIMEOUT_DIFF_P", 20000);
	init_param_int(PARAM_SENSOR_BARO_TIMEOUT, "TIMEOUT_BARO", 20000);
	init_param_int(PARAM_SENSOR_SONAR_TIMEOUT, "TIMEOUT_SONAR", 20000);
	init_param_int(PARAM_SENSOR_MAG_TIMEOUT, "TIMEOUT_MAG", 20000);
	init_param_int(PARAM_SENSOR_OFFB_CTRL_TIMEOUT, "TIMEOUT_OFFB_CTRL", 50000);

	//==-- Estimator
	init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // ms
	init_param_int(PARAM_EST_USE_ACC_COR, "EST_ACC_COR", 1); // 1=true; 0=false
	init_param_int(PARAM_EST_USE_MAT_EXP, "EST_MAT_EXP", 1); // 1=true; 0=false
	init_param_int(PARAM_EST_USE_QUAD_INT, "EST_QUAD_INT", 1); // 1=true; 0=false
	init_param_fix16(PARAM_FILTER_KP, "FILTER_KP", fix16_from_float(1.0f));
	init_param_fix16(PARAM_FILTER_KI, "FILTER_KI", fix16_from_float(0.05f));
	init_param_fix16(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", fix16_from_float(0.6f));
	init_param_fix16(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", fix16_from_float(0.6f));
	init_param_int(PARAM_STREAM_ADJUSTED_GYRO, "STRM_ADJST_GYRO", 0);
	init_param_int(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0);
	init_param_int(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0);
	init_param_int(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0);
	init_param_int(PARAM_ACC_X_BIAS, "ACC_X_BIAS", 0);
	init_param_int(PARAM_ACC_Y_BIAS, "ACC_Y_BIAS", 0);
	init_param_int(PARAM_ACC_Z_BIAS, "ACC_Z_BIAS", 0);
	init_param_fix16(PARAM_ACC_X_TEMP_COMP, "ACC_X_TEMP_COMP", fix16_from_float(0.0f));
	init_param_fix16(PARAM_ACC_Y_TEMP_COMP, "ACC_Y_TEMP_COMP", fix16_from_float(0.0f));
	init_param_fix16(PARAM_ACC_Z_TEMP_COMP, "ACC_Z_TEMP_COMP", fix16_from_float(0.0f));

	//==-- Control
	init_param_fix16(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", fix16_from_float(0.15f));
	init_param_fix16(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", fix16_from_float(0.05f));
	init_param_fix16(PARAM_PID_ROLL_RATE_D, "PID_ROLL_RATE_D", fix16_from_float(0.003f));
	init_param_fix16(PARAM_MAX_ROLL_RATE, "MAX_ROLL_RATE", fix16_from_float(3.14159f));

	init_param_fix16(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", fix16_from_float(0.15f));
	init_param_fix16(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", fix16_from_float(0.05f));
	init_param_fix16(PARAM_PID_PITCH_RATE_D, "PID_PITCH_RATE_D", fix16_from_float(0.003f));
	init_param_fix16(PARAM_MAX_PITCH_RATE, "MAX_PITCH_RATE", fix16_from_float(3.14159f));

	init_param_fix16(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", fix16_from_float(0.2f));
	init_param_fix16(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", fix16_from_float(0.1f));
	init_param_fix16(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", fix16_from_float(0.0f));
	init_param_fix16(PARAM_MAX_YAW_RATE, "MAX_YAW_RATE", fix16_from_float(3.14159f/2.0f));

	init_param_fix16(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", fix16_from_float(6.5f));
	init_param_fix16(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", fix16_from_float(0.786f));

	init_param_fix16(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", fix16_from_float(6.5f));
	init_param_fix16(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", fix16_from_float(0.786f));

	init_param_fix16(PARAM_PID_YAW_ANGLE_P, "PID_YAW_ANG_P", fix16_from_float(2.8f));

	init_param_fix16(PARAM_PID_TAU, "PID_TAU", fix16_from_float(0.05f));
	init_param_int(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000);

	//==-- Output
	init_param_int(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_RATE", 400);
	init_param_int(PARAM_MOTOR_PWM_IDLE, "MOTOR_PWM_IDLE", 1150);
	init_param_int(PARAM_MOTOR_PWM_MIN, "MOTOR_PWM_MIN", 1000);
	init_param_int(PARAM_MOTOR_PWM_MAX, "MOTOR_PWM_MAX", 2000);

	init_param_int(PARAM_MIXER, "MIXER", QUADCOPTER_PLUS);
}

bool read_params(void) {
	return readEEPROM();
}

bool write_params(void) {
	return writeEEPROM();
}


void param_change_callback(param_id_t id)
{
	/* //TODO: Live update parameters
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
 case PARAM_STREAM_BARO_RATE:
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
*/
}

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

int32_t get_param_int(param_id_t id) {
	return _params.values[id];
}

fix16_t get_param_fix16(param_id_t id) {
	union {
		fix16_t f;
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

		return true;
	}

	return false;
}

bool set_param_fix16(param_id_t id, fix16_t value) {
	union {
		fix16_t f;
		uint32_t i;
	} u;

	u.f = value;

 return set_param_int(id, u.i);
}

bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value) {
	param_id_t id = lookup_param_id(name);
	return set_param_int(id, value);
}

bool set_param_by_name_fix16(const char name[PARAMS_NAME_LENGTH], fix16_t value) {
	union {
		fix16_t f;
		uint32_t i;
	} u;

	u.f = value;
	return set_param_by_name_int(name, u.i);
}
