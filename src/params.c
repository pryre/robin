#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "params.h"
#include "mixer.h"
#include "fix16.h"

#include "param_generator/param_gen.h"

//#include "controller.h"
//#include "pid_controller.h"

#include "safety.h"
#include "mavlink_system.h"
//#include "mavlink_transmit.h"


// global variable definitions
params_t _params;
const char _param_names[PARAMS_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];

// local function definitions
void init_param_uint(param_id_t id, const uint32_t value) {
	//memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	_params.types[id] = MAVLINK_TYPE_UINT32_T;
}

void init_param_int(param_id_t id, const int32_t value) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	//memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	_params.types[id] = MAVLINK_TYPE_INT32_T;
}

void init_param_fix16(param_id_t id, const fix16_t value) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	//memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
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
	}

	//Check if we should reset the by request
	if(get_param_uint(PARAM_RESET_PARAMS)) {
		set_param_defaults();

		write_params();
	}

	//for(param_id_t i=0; i < PARAMS_COUNT; i++)
	//	param_change_callback(i);
}

bool read_params(void) {
	return readEEPROM();
}

bool write_params(void) {
	bool success = false;

	if( safety_request_state( MAV_STATE_STANDBY ) ) {
		if( writeEEPROM() ) {
			mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[PARAM] EEPROM written");
			success = true;
		} else {
			mavlink_send_broadcast_statustext(MAV_SEVERITY_ERROR, "[PARAM] EEPROM write failed");
		}
	} else {
		mavlink_queue_broadcast_error("[SAFETY] Unable to enter standby, can't write params!");
	}

	return success;
}

param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]) {
	for (uint16_t id = 0; id < PARAMS_COUNT; id++) {
		bool match = true;

		for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++) {
			// compare each character
			if (name[i] != _param_names[id][i])
			{
				match = false;
				break;
			}

			// stop comparing if end of string is reached
			if (_param_names[id][i] == '\0')
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
	memcpy(name, _param_names[id], PARAMS_NAME_LENGTH);
}

MAV_PARAM_TYPE get_param_type(param_id_t id) {
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
