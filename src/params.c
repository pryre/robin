#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/drv_flash.h"
#include "drivers/drv_sensors.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"
#include "fix16.h"
#include "mixer.h"
#include "params.h"
#include "sensors.h"

#include "param_gen.h"

#include "mavlink_system.h"
#include "safety.h"

// global variable definitions
params_t _params;
//const char _param_names[PARAMS_COUNT]
//					   [MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
//const MAV_PARAM_TYPE _param_types[PARAMS_COUNT];

// local function definitions
void init_param_uint( param_id_t id, const uint32_t value ) {
	// memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = value;
	//_params.types[id] = MAVLINK_TYPE_UINT32_T;
}

void init_param_int( param_id_t id, const int32_t value ) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	// memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	//_params.types[id] = MAVLINK_TYPE_INT32_T;
}

void init_param_fix16( param_id_t id, const fix16_t value ) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	// memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
	_params.values[id] = u.u;
	//_params.types[id] = MAVLINK_TYPE_FLOAT;
}

// function definitions
void params_init( void ) {
	drv_flash_init();

	if ( !read_params() ) {
		bool flasher = true;

		for ( uint32_t i = 0; i < 5; i++ ) {
			if ( flasher ) {
				status_led_arm_set( true );
				status_led_heart_set( true );
			} else {
				status_led_arm_set( false );
				status_led_heart_set( false );
			}

			flasher = !flasher;
			system_pause_ms( 100 );
		}

		status_led_arm_set( false );
		status_led_heart_set( false );

		set_param_defaults();

		write_params();
	}

	// Check if we should reset the by request
	if ( get_param_uint( PARAM_RESET_PARAMS ) ) {
		set_param_defaults();

		write_params();
	}

	// for(param_id_t i=0; i < PARAMS_COUNT; i++)
	//	param_change_callback(i);
}

bool read_params( void ) {
	return drv_flash_read();
}

bool write_params( void ) {
	bool success = false;
	bool state_ok = false;

	// Deinit imu as writing EEPROM messes up the callback
	if ( ( _system_status.state != MAV_STATE_UNINIT ) && ( _system_status.state != MAV_STATE_BOOT ) ) {

		drv_sensors_i2c_clear();

		state_ok = safety_request_state( MAV_STATE_STANDBY );
	} else {
		state_ok = true; // We are booting, so no need to check for state
	}

	if ( state_ok ) {
		if ( drv_flash_write() ) {
			mavlink_send_broadcast_statustext( MAV_SEVERITY_INFO,
											   "[PARAM] EEPROM written" );
			success = true;
		} else {
			mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR,
											   "[PARAM] EEPROM write failed" );
		}
	} else {
		mavlink_queue_broadcast_error(
			"[SAFETY] Unable to enter standby, can't write params!" );
	}

	return success;
}

param_id_t lookup_param_id( const char name[PARAMS_NAME_LENGTH] ) {
	for ( uint16_t id = 0; id < PARAMS_COUNT; id++ ) {
		bool match = true;

		for ( uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++ ) {
			// compare each character
			if ( name[i] != _param_names[id][i] ) {
				match = false;
				break;
			}

			// stop comparing if end of string is reached
			if ( _param_names[id][i] == '\0' )
				break;
		}

		if ( match )
			return (param_id_t)id;
	}

	return PARAMS_COUNT;
}

uint32_t get_param_uint( param_id_t id ) {
	return _params.values[id];
}

int32_t get_param_int( param_id_t id ) {
	union {
		uint32_t u;
		int32_t i;
	} u;

	u.u = _params.values[id];

	return u.i;
}

fix16_t get_param_fix16( param_id_t id ) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.u = _params.values[id];

	return u.f;
}

void get_param_name( param_id_t id, char name[PARAMS_NAME_LENGTH] ) {
	memcpy( name, _param_names[id], PARAMS_NAME_LENGTH );
}

MAV_PARAM_TYPE get_param_type( param_id_t id ) {
	return _param_types[id];
}

bool set_param_uint( param_id_t id, uint32_t value ) {
	// XXX: Don't care if we override same value: if(... && value !=
	// _params.values[id]) {
	if ( id < PARAMS_COUNT ) {
		_params.values[id] = value;
		param_change_callback( id );

		return true;
	}

	return false;
}

bool set_param_int( param_id_t id, int32_t value ) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	return set_param_uint( id, u.u );
}

bool set_param_fix16( param_id_t id, fix16_t value ) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	return set_param_uint( id, u.u );
}

bool set_param_by_name_uint( const char name[PARAMS_NAME_LENGTH],
							 uint32_t value ) {
	param_id_t id = lookup_param_id( name );

	return set_param_uint( id, value );
}

bool set_param_by_name_int( const char name[PARAMS_NAME_LENGTH], int32_t value ) {
	union {
		int32_t i;
		uint32_t u;
	} u;

	u.i = value;

	return set_param_by_name_uint( name, u.u );
}

bool set_param_by_name_fix16( const char name[PARAMS_NAME_LENGTH],
							  fix16_t value ) {
	union {
		fix16_t f;
		uint32_t u;
	} u;

	u.f = value;

	return set_param_by_name_uint( name, u.u );
}
