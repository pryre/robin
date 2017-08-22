#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "params.h"
#include "mixer.h"
#include "fix16.h"

#include "controller.h"
#include "pid_controller.h"

#include "safety.h"
#include "mavlink_system.h"


// global variable definitions
params_t _params;

// local function definitions
static void init_param_uint(param_id_t id, const char name[PARAMS_NAME_LENGTH], const uint32_t value) {
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	_params.types[id] = MAVLINK_TYPE_UINT32_T;
}

static void init_param_int(param_id_t id, const char name[PARAMS_NAME_LENGTH], const int32_t value) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	_params.types[id] = MAVLINK_TYPE_INT32_T;
}

static void init_param_fix16(param_id_t id, const char name[PARAMS_NAME_LENGTH], const fix16_t value) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	_params.types[id] = MAVLINK_TYPE_FLOAT;
}

// function definitions
void params_init(void) {
	initEEPROM();

	if ( !read_params() ) {
		bool flasher = true;

		for(uint32_t i=0; i<5; i++) {
			if(flasher) {
				digitalLo(LED0_GPIO, LED0_PIN);
				digitalLo(LED1_GPIO, LED1_PIN);
			} else {
				digitalHi(LED0_GPIO, LED0_PIN);
				digitalHi(LED1_GPIO, LED1_PIN);
			}

			flasher = !flasher;
			delay(100);
		}

		digitalHi(LED0_GPIO, LED0_PIN);
		digitalHi(LED1_GPIO, LED1_PIN);

		set_param_defaults();

		write_params();

		systemReset();
	}
}

//TODO: Still need to clean up int and uint parameters
//		Should make an automated script for checking correct usages!
void set_param_defaults(void) {
	//==-- System
	init_param_uint(PARAM_BOARD_REVISION, "BOARD_REV", 5);
	init_param_uint(PARAM_VERSION_FIRMWARE, "FW_VERSION", strtoll(GIT_VERSION_FLIGHT_STR, NULL, 16));
	init_param_uint(PARAM_VERSION_SOFTWARE, "SW_VERSION", strtoll(GIT_VERSION_OS_STR, NULL, 16));
	init_param_uint(PARAM_BAUD_RATE_0, "BAUD_RATE_0", 921600);	//Set baud rate to 0 to disable a comm port
	init_param_uint(PARAM_BAUD_RATE_1, "BAUD_RATE_1", 0);	//Set baud rate to 0 to disable a comm port
	init_param_fix16(PARAM_TIMESYNC_ALPHA, "TIMESYNC_ALPHA", fix16_from_float(0.8f));

	//==-- Mavlink
	init_param_uint(PARAM_SYSTEM_ID, "SYS_ID", 1);
	init_param_uint(PARAM_COMPONENT_ID, "COMP_ID", 1);

	init_param_uint(PARAM_STREAM_RATE_HEARTBEAT_0, "STRM0_HRTBT", 1000000);
	init_param_uint(PARAM_STREAM_RATE_SYS_STATUS_0, "STRM0_SYS_STAT", 5000000);
	init_param_uint(PARAM_STREAM_RATE_HIGHRES_IMU_0, "STRM0_HR_IMU", 10000);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_0, "STRM0_ATT", 0);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0, "STRM0_ATT_Q", 20000);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_TARGET_0, "STRM0_ATT_T", 20000);
	init_param_uint(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0, "STRM0_SRV_OUT", 100000);
	init_param_uint(PARAM_STREAM_RATE_TIMESYNC_0, "STRM0_TIMESYNC", 100000);
	init_param_uint(PARAM_STREAM_RATE_LOW_PRIORITY_0, "STRM0_LPQ", 10000);

	init_param_uint(PARAM_STREAM_RATE_HEARTBEAT_1, "STRM1_HRTBT", 1000000);
	init_param_uint(PARAM_STREAM_RATE_SYS_STATUS_1, "STRM1_SYS_STAT", 5000000);
	init_param_uint(PARAM_STREAM_RATE_HIGHRES_IMU_1, "STRM1_HR_IMU", 10000);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_1, "STRM1_ATT", 0);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1, "STRM1_ATT_Q", 20000);
	init_param_uint(PARAM_STREAM_RATE_ATTITUDE_TARGET_1, "STRM1_ATT_T", 20000);
	init_param_uint(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1, "STRM1_SRV_OUT", 100000);
	init_param_uint(PARAM_STREAM_RATE_TIMESYNC_1, "STRM1_TIMESYNC", 100000);
	init_param_uint(PARAM_STREAM_RATE_LOW_PRIORITY_1, "STRM1_LPQ", 10000);

	//==-- Sensors
	//All params here in us
	init_param_uint(PARAM_SENSOR_IMU_CBRK, "CBRK_IMU", 1);
	init_param_uint(PARAM_SENSOR_MAG_CBRK, "CBRK_MAG", 0);
	init_param_uint(PARAM_SENSOR_BARO_CBRK, "CBRK_BARO", 0);
	init_param_uint(PARAM_SENSOR_SONAR_CBRK, "CBRK_SONAR", 0);
	init_param_uint(PARAM_SENSOR_SAFETY_CBRK, "CBRK_SAFETY", 1);

	//TODO: Double check which is used here
	init_param_uint(PARAM_SENSOR_BARO_UPDATE, "UPDATE_BARO", 0);
	init_param_uint(PARAM_SENSOR_SONAR_UPDATE, "UPDATE_SONAR", 0);
	init_param_uint(PARAM_SENSOR_MAG_UPDATE, "UPDATE_MAG", 0);

	init_param_uint(PARAM_SENSOR_IMU_STRM_COUNT, "STRM_NUM_IMU", 1000);
	init_param_uint(PARAM_SENSOR_BARO_STRM_COUNT, "STRM_NUM_BARO", 50);
	init_param_uint(PARAM_SENSOR_SONAR_STRM_COUNT, "STRM_NUM_SONAR", 50);
	init_param_uint(PARAM_SENSOR_MAG_STRM_COUNT, "STRM_NUM_MAG", 50);
	init_param_uint(PARAM_SENSOR_OFFB_HRBT_STRM_COUNT, "STRM_NUM_OB_H", 2);
	init_param_uint(PARAM_SENSOR_OFFB_CTRL_STRM_COUNT, "STRM_NUM_OB_C", 100);

	init_param_uint(PARAM_SENSOR_IMU_TIMEOUT, "TIMEOUT_IMU", 2000);
	init_param_uint(PARAM_SENSOR_BARO_TIMEOUT, "TIMEOUT_BARO", 20000);
	init_param_uint(PARAM_SENSOR_SONAR_TIMEOUT, "TIMEOUT_SONAR", 20000);
	init_param_uint(PARAM_SENSOR_MAG_TIMEOUT, "TIMEOUT_MAG", 20000);
	init_param_uint(PARAM_SENSOR_OFFB_HRBT_TIMEOUT, "UPDATE_OB_HRBT", 5000000);
	init_param_uint(PARAM_SENSOR_OFFB_CTRL_TIMEOUT, "TIMEOUT_OB_CTRL", 200000);

	//==-- Estimator
	init_param_uint(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // ms
	init_param_uint(PARAM_EST_USE_ACC_COR, "EST_ACC_COR", 1); // 1=true; 0=false
	init_param_uint(PARAM_EST_USE_MAT_EXP, "EST_MAT_EXP", 1); // 1=true; 0=false
	init_param_uint(PARAM_EST_USE_QUAD_INT, "EST_QUAD_INT", 1); // 1=true; 0=false
	init_param_uint(PARAM_EST_USE_ADPT_BIAS, "EST_ADPT_BIAS", 0); // 1=true; 0=false
	init_param_fix16(PARAM_FILTER_KP, "FILTER_KP", fix16_from_float(1.0f));
	init_param_fix16(PARAM_FILTER_KI, "FILTER_KI", fix16_from_float(0.05f));
	init_param_fix16(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", fix16_from_float(0.6f));
	init_param_fix16(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", fix16_from_float(0.6f));
	init_param_int(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0);
	init_param_int(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0);
	init_param_int(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0);
	init_param_int(PARAM_ACC_X_BIAS, "ACC_X_BIAS", 0);
	init_param_int(PARAM_ACC_Y_BIAS, "ACC_Y_BIAS", 0);
	init_param_int(PARAM_ACC_Z_BIAS, "ACC_Z_BIAS", 0);
	init_param_fix16(PARAM_ACC_X_SCALE_POS, "ACC_X_S_POS", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_X_SCALE_NEG, "ACC_X_S_NEG", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_Y_SCALE_POS, "ACC_Y_S_POS", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_Y_SCALE_NEG, "ACC_Y_S_NEG", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_Z_SCALE_POS, "ACC_Z_S_POS", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_Z_SCALE_NEG, "ACC_Z_S_NEG", fix16_from_float(1.0f));
	init_param_fix16(PARAM_ACC_X_TEMP_COMP, "ACC_X_TEMP_COMP", fix16_from_float(0.0f));
	init_param_fix16(PARAM_ACC_Y_TEMP_COMP, "ACC_Y_TEMP_COMP", fix16_from_float(0.0f));
	init_param_fix16(PARAM_ACC_Z_TEMP_COMP, "ACC_Z_TEMP_COMP", fix16_from_float(0.0f));

	//==-- Control
	init_param_fix16(PARAM_PID_ROLL_RATE_P, "PID_ROLL_R_P", fix16_from_float(0.05f));
	init_param_fix16(PARAM_PID_ROLL_RATE_I, "PID_ROLL_R_I", fix16_from_float(0.0f));
	init_param_fix16(PARAM_PID_ROLL_RATE_D, "PID_ROLL_R_D", fix16_from_float(0.005f));
	init_param_fix16(PARAM_MAX_ROLL_RATE, "MAX_ROLL_R", fix16_from_float(3.14159f));

	init_param_fix16(PARAM_PID_PITCH_RATE_P, "PID_PITCH_R_P", fix16_from_float(0.05f));
	init_param_fix16(PARAM_PID_PITCH_RATE_I, "PID_PITCH_R_I", fix16_from_float(0.00f));
	init_param_fix16(PARAM_PID_PITCH_RATE_D, "PID_PITCH_R_D", fix16_from_float(0.005f));
	init_param_fix16(PARAM_MAX_PITCH_RATE, "MAX_PITCH_R", fix16_from_float(3.14159f));

	init_param_fix16(PARAM_PID_YAW_RATE_P, "PID_YAW_R_P", fix16_from_float(0.2f));
	init_param_fix16(PARAM_PID_YAW_RATE_I, "PID_YAW_R_I", fix16_from_float(0.1f));
	init_param_fix16(PARAM_PID_YAW_RATE_D, "PID_YAW_R_D", fix16_from_float(0.0f));
	init_param_fix16(PARAM_MAX_YAW_RATE, "MAX_YAW_R", fix16_from_float(3.14159f/2.0f));

	init_param_fix16(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", fix16_from_float(6.5f));
	init_param_fix16(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", fix16_from_float(0.786f));

	init_param_fix16(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", fix16_from_float(6.5f));
	init_param_fix16(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", fix16_from_float(0.786f));

	init_param_fix16(PARAM_PID_YAW_ANGLE_P, "PID_YAW_ANG_P", fix16_from_float(2.8f));

	init_param_fix16(PARAM_PID_TAU, "PID_TAU", fix16_from_float(0.05f));
	init_param_int(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000);

	//==-- Output
	init_param_uint(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_RATE", 400);
	init_param_uint(PARAM_MOTOR_PWM_IDLE, "MOTOR_PWM_IDLE", 1150);
	init_param_uint(PARAM_MOTOR_PWM_MIN, "MOTOR_PWM_MIN", 1000);
	init_param_uint(PARAM_MOTOR_PWM_MAX, "MOTOR_PWM_MAX", 2000);

	init_param_uint(PARAM_DO_ESC_CAL, "DO_ESC_CAL", 0); // 1=true; 0=false
	init_param_fix16(PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THRTL", fix16_from_float(0.25f));
	init_param_uint(PARAM_THROTTLE_TIMEOUT, "TIMEOUT_THRTL", 10000000);

	init_param_uint(PARAM_MIXER, "MIXER", MIXER_QUADCOPTER_X);
}

bool read_params(void) {
	return readEEPROM();
}

bool write_params(void) {
	bool success = false;

	//XXX: System will freeze after eeprom write (not 100% sure why, but something to do with interrupts), so reboot here
	if( safety_request_state( MAV_STATE_POWEROFF ) ) {
		bool flasher = true;
		GPIO_TypeDef *gpio_led_p;
		uint16_t led_pin;

		if( writeEEPROM() ) {
			//Write sucess
			mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[PARAM] EEPROM written, mav will now reboot");

			gpio_led_p = LED0_GPIO;
			led_pin = LED0_PIN;

		} else {
			//Write failed
			mavlink_send_broadcast_statustext(MAV_SEVERITY_ERROR, "[PARAM] EEPROM write failed, mav will now reboot");

			gpio_led_p = LED1_GPIO;
			led_pin = LED1_PIN;
		}

		for(uint32_t i=0; i<5; i++) {
			if(flasher) {
				digitalLo(gpio_led_p, led_pin);
			} else {
				digitalHi(gpio_led_p, led_pin);
			}

			flasher = !flasher;
			delay(100);
		}

		digitalHi(LED0_GPIO, LED0_PIN);
		digitalHi(LED1_GPIO, LED1_PIN);

		systemReset();
	} else {
		mavlink_queue_broadcast_error("[SAFETY] Unable to poweroff, can't write params!");
	}

	return success;
}

//TODO: Any more params need a callback to update?
void param_change_callback(param_id_t id) {
	switch(id) {
		case PARAM_PID_ROLL_RATE_P:
			pid_set_gain_p(&_pid_roll_rate, get_param_fix16(PARAM_PID_ROLL_RATE_P));
			break;
		case PARAM_PID_ROLL_RATE_I:
			pid_set_gain_i(&_pid_roll_rate, get_param_fix16(PARAM_PID_ROLL_RATE_I));
			break;
		case PARAM_PID_ROLL_RATE_D:
			pid_set_gain_d(&_pid_roll_rate, get_param_fix16(PARAM_PID_ROLL_RATE_D));
			break;
		case PARAM_PID_PITCH_RATE_P:
			pid_set_gain_p(&_pid_pitch_rate, get_param_fix16(PARAM_PID_PITCH_RATE_P));
			break;
		case PARAM_PID_PITCH_RATE_I:
			pid_set_gain_i(&_pid_pitch_rate, get_param_fix16(PARAM_PID_PITCH_RATE_I));
			break;
		case PARAM_PID_PITCH_RATE_D:
			pid_set_gain_d(&_pid_pitch_rate, get_param_fix16(PARAM_PID_PITCH_RATE_D));
			break;
		case PARAM_PID_YAW_RATE_P:
			pid_set_gain_p(&_pid_yaw_rate, get_param_fix16(PARAM_PID_YAW_RATE_P));
			break;
		case PARAM_PID_YAW_RATE_I:
			pid_set_gain_i(&_pid_yaw_rate, get_param_fix16(PARAM_PID_YAW_RATE_I));
			break;
		case PARAM_PID_YAW_RATE_D:
			pid_set_gain_d(&_pid_yaw_rate, get_param_fix16(PARAM_PID_YAW_RATE_D));
			break;
		case PARAM_PID_TAU:
				pid_set_gain_tau(&_pid_roll_rate, get_param_fix16(PARAM_PID_TAU));
				pid_set_gain_tau(&_pid_pitch_rate, get_param_fix16(PARAM_PID_TAU));
				pid_set_gain_tau(&_pid_yaw_rate, get_param_fix16(PARAM_PID_TAU));
			break;
		case PARAM_MAX_ROLL_RATE:
			pid_set_min_max(&_pid_roll_rate,
							-get_param_fix16(PARAM_MAX_ROLL_RATE),
							get_param_fix16(PARAM_MAX_ROLL_RATE));
			break;
		case PARAM_MAX_PITCH_RATE:
			pid_set_min_max(&_pid_pitch_rate,
							-get_param_fix16(PARAM_MAX_PITCH_RATE),
							get_param_fix16(PARAM_MAX_PITCH_RATE));
			break;
		case PARAM_MAX_YAW_RATE:
			pid_set_min_max(&_pid_yaw_rate,
							-get_param_fix16(PARAM_MAX_YAW_RATE),
							get_param_fix16(PARAM_MAX_YAW_RATE));
			break;
		default:
			// no action needed for this parameter
			break;
	}

	mavlink_message_t msg_out;
	mavlink_prepare_param_value( &msg_out, id );
	lpq_queue_broadcast_msg( &msg_out );
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

uint32_t get_param_uint(param_id_t id) {
	return _params.values[id];
}

int32_t get_param_int(param_id_t id) {
	union {
		uint32_t u;
		int32_t i;
	} u;

	u.u = _params.values[id];

	return u.i;
}

fix16_t get_param_fix16(param_id_t id) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.u = _params.values[id];

	return u.f;
}

void get_param_name(param_id_t id, char name[PARAMS_NAME_LENGTH]) {
	memcpy(name, _params.names[id], PARAMS_NAME_LENGTH);
}

mavlink_message_type_t get_param_type(param_id_t id) {
	return _params.types[id];
}

bool set_param_uint(param_id_t id, uint32_t value) {
	if (id < PARAMS_COUNT && value != _params.values[id]) {
		_params.values[id] = value;
		param_change_callback(id);

		return true;
	}

	return false;
}

bool set_param_int(param_id_t id, int32_t value) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	return set_param_uint(id, u.u);
}

bool set_param_fix16(param_id_t id, fix16_t value) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	return set_param_uint(id, u.u);
}

bool set_param_by_name_uint(const char name[PARAMS_NAME_LENGTH], uint32_t value) {
	param_id_t id = lookup_param_id(name);

	return set_param_uint(id, value);
}

bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	return set_param_by_name_uint(name, u.u);
}

bool set_param_by_name_fix16(const char name[PARAMS_NAME_LENGTH], fix16_t value) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	return set_param_by_name_uint(name, u.u);
}
