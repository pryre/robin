#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "calibration.h"
#include "mavlink_system.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"

#include "drivers/drv_status_io.h"

#include "fix16.h"
#include "fixextra.h"
#include "fixquat.h"
#include "fixvector3d.h"

calibration_data_t _calibrations;
//sensor_readings_t _sensors;

void calibration_init( void ) {
	_calibrations.type = CAL_NONE;

	_calibrations.data.gyro.count = 0;
	_calibrations.data.gyro.sum_x = 0;
	_calibrations.data.gyro.sum_y = 0;
	_calibrations.data.gyro.sum_z = 0;

	_calibrations.data.accel.accel_cal_step = CAL_ACCEL_INIT;
	_calibrations.data.accel.waiting = false;
	_calibrations.data.accel.data.count = 0;

	_calibrations.data.accel.data.t_sum = 0;
	_calibrations.data.accel.data.x_sum = 0;
	_calibrations.data.accel.data.y_sum = 0;
	_calibrations.data.accel.data.z_sum = 0;

	_calibrations.data.accel.data.t_av_sum = 0;
	_calibrations.data.accel.data.x_flat_av_sum = 0;
	_calibrations.data.accel.data.y_flat_av_sum = 0;
	_calibrations.data.accel.data.z_flat_av_sum = 0;
	_calibrations.data.accel.data.x_up_av = 0;
	_calibrations.data.accel.data.x_down_av = 0;
	_calibrations.data.accel.data.y_up_av = 0;
	_calibrations.data.accel.data.y_down_av = 0;
	_calibrations.data.accel.data.z_up_av = 0;
	_calibrations.data.accel.data.z_down_av = 0;

	_calibrations.data.rc.waiting = false;
	_calibrations.data.rc.step = CAL_RC_RANGE_INIT;
	_calibrations.data.rc.is_stick = 0;
	_calibrations.data.rc.is_switch = 0;
	for ( int i = 0; i < 8; i++ ) {
		// XXX: Init all to "true stick centre"
		_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] = 1500;
		_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MID] = 1500;
		_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] = 1500;
		_calibrations.data.rc.rev[i] = false;
	}

	_calibrations.data.accel.temp_scale = fix16_from_float( 340.0f );
	_calibrations.data.accel.temp_shift = fix16_from_float( 36.53f );
}

static bool calibration_request_state( void ) {
	bool success = safety_request_state( MAV_STATE_CALIBRATING );

	if ( !success ) {
		mavlink_queue_broadcast_error(
			"[SENSOR] Cannot enter calibration in this mode!" );
	}

	return success;
}

bool calibration_request( calibration_request_t req ) {
	bool success = false;
	char text_reason[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

	switch ( req ) {
	case CAL_GYRO: {
		if ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "gyro", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_MAG: {
		if ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "mag", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_GND_PRESSURE: {
		if ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "gnd press", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_RC: {
		if( ( _system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK ) &&
			( _sensors.rc_input.mapping_set ) ) {

			success = calibration_request_state();
		} else {

			if ( !_sensors.rc_input.mapping_set ) {
				mavlink_queue_broadcast_error( "[SENSOR] RC mapping params no set" );
			}

			strncpy( text_reason, "rc", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_ACCEL: {
		if ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "accel", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_LEVEL_HORIZON: {
		if ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "level", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	case CAL_INTER: {
		// TODO: Need to check sensors required are present
		success = calibration_request_state();

		break;
	}
	case CAL_BARO: {
		if ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
			success = calibration_request_state();
		} else {
			strncpy( text_reason, "baro", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN );
		}

		break;
	}
	default: { break; }
	}

	if ( success ) {
		_calibrations.type = req;
	} else {
		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Cannot cal ";
		strncat( text, text_reason, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		strncat( text, " no input detected!",
				 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		mavlink_queue_broadcast_error( text );
	}

	return success;
}

void calibration_done( void ) {
	safety_request_state( MAV_STATE_STANDBY );

	calibration_init(); // Reset calibration data
}

// TODO: This does not take into account temperature
// Returns true if all calibrations are complete
void calibration_run( void ) {
	bool cal_mode_error = false;

	// If we actually need to do a calibration
	if ( _calibrations.type != CAL_NONE ) {
		switch ( _calibrations.type ) {
		case CAL_GYRO: {
			cal_mode_error = !calibrate_gyro();

			break;
		}
		case CAL_MAG: {
			cal_mode_error = !calibrate_magnetometer();

			break;
		}
		case CAL_GND_PRESSURE: {
			cal_mode_error = !calibrate_ground_pressure();

			break;
		}
		case CAL_RC: {
			cal_mode_error = !calibrate_rc();

			break;
		}
		case CAL_ACCEL: {
			cal_mode_error = !calibrate_accel();

			break;
		}
		case CAL_LEVEL_HORIZON: {
			cal_mode_error = !calibrate_level_horizon();

			break;
		}
		case CAL_INTER: {
			cal_mode_error = !calibrate_interference();

			break;
		}
		case CAL_BARO: {
			cal_mode_error = !calibrate_barometer();

			break;
		}
		default: {
			calibration_done();
			cal_mode_error = true;
			mavlink_queue_broadcast_error(
				"[SENSOR] Invalid calibration mode, clearing" );

			break;
		}
		}

		// If there was a calibration running, but it finished this pass
		if ( _calibrations.type == CAL_NONE ) {
			mavlink_message_t msg;

			if ( _system_status.state == MAV_STATE_CALIBRATING ) {
				calibration_done();
				mavlink_queue_broadcast_error(
					"[SENSOR] Invalid calibration state, clearing" );
			}

			if ( cal_mode_error ) {
				// Send a message saying that it has failed
				mavlink_prepare_command_ack( &msg, MAV_CMD_PREFLIGHT_CALIBRATION,
											 MAV_RESULT_FAILED, _calibrations.req_sysid,
											 _calibrations.req_compid, 0xFF );
				status_buzzer_failure();
			} else {
				// Send a message saying that it has completed 100%
				mavlink_prepare_command_ack(
					&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_IN_PROGRESS,
					_calibrations.req_sysid, _calibrations.req_compid, 100 );
				status_buzzer_success();
			}

			lpq_queue_broadcast_msg( &msg );
		}
	}
}

#ifdef __cplusplus
}
#endif
