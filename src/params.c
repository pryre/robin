#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "params.h"
#include "mixer.h"
#include "fix16.h"

#include "param_generator/param_gen.h"

#include "controller.h"
#include "pid_controller.h"

#include "safety.h"
#include "mavlink_system.h"
#include "mavlink_transmit.h"


// global variable definitions
params_t _params;

// local function definitions
void init_param_uint(param_id_t id, const char name[PARAMS_NAME_LENGTH], const uint32_t value) {
	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	_params.types[id] = MAVLINK_TYPE_UINT32_T;
}

void init_param_int(param_id_t id, const char name[PARAMS_NAME_LENGTH], const int32_t value) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	_params.types[id] = MAVLINK_TYPE_INT32_T;
}

void init_param_fix16(param_id_t id, const char name[PARAMS_NAME_LENGTH], const fix16_t value) {
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

		//systemReset();
	}
	
	//for(param_id_t i=0; i < PARAMS_COUNT; i++)
	//	param_change_callback(i);
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
//TODO: Should probably check return values for bad param set
void param_change_callback(param_id_t id) {
	switch(id) {
		case PARAM_STREAM_RATE_HEARTBEAT_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_HEARTBEAT);
			break;
		case PARAM_STREAM_RATE_SYS_STATUS_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_SYS_STATUS);
			break;
		case PARAM_STREAM_RATE_HIGHRES_IMU_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_HIGHRES_IMU);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_ATTITUDE);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_ATTITUDE_QUATERNION);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_TARGET_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_ATTITUDE_TARGET);
			break;
		case PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW);
			break;
		case PARAM_STREAM_RATE_TIMESYNC_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_TIMESYNC);
			break;
		case PARAM_STREAM_RATE_LOW_PRIORITY_0:
			communication_calc_period_update(COMM_CH_0, MAVLINK_STREAM_ID_LOW_PRIORITY);
			break;
		case PARAM_STREAM_RATE_HEARTBEAT_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_HEARTBEAT);
			break;
		case PARAM_STREAM_RATE_SYS_STATUS_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_SYS_STATUS);
			break;
		case PARAM_STREAM_RATE_HIGHRES_IMU_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_HIGHRES_IMU);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_ATTITUDE);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_ATTITUDE_QUATERNION);
			break;
		case PARAM_STREAM_RATE_ATTITUDE_TARGET_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_ATTITUDE_TARGET);
			break;
		case PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW);
			break;
		case PARAM_STREAM_RATE_TIMESYNC_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_TIMESYNC);
			break;
		case PARAM_STREAM_RATE_LOW_PRIORITY_1:
			communication_calc_period_update(COMM_CH_1, MAVLINK_STREAM_ID_LOW_PRIORITY);
			break;
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
