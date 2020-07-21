#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "calibration.h"
#include "safety.h"

// calibration_data_t _calibrations;
// system_status_t _system_status;

MAV_RESULT mavlink_handle_command_long_preflight_calibration(
	mavlink_channel_t chan, float* params, uint8_t sysid, uint8_t compid ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	//XXX: Only allow one calibration request at a time
	if ( _system_status.state == MAV_STATE_CALIBRATING ) {
		if ( ( (int)params[4] == CAL_CMD_ACCEL ) && ( _calibrations.type == CAL_ACCEL ) ) {
			_calibrations.data.accel.waiting = false;
			command_result = MAV_RESULT_ACCEPTED;
		} else if ( ( (int)params[3] == CAL_CMD_RC ) && ( _calibrations.type == CAL_RC ) ) {
			if ( _calibrations.data.rc.waiting ) {
				_calibrations.data.rc.waiting = false;
			}
			/* else if ( _calibrations.data.rc.step == CAL_RC_RANGE_EXTREMES ) {
				_calibrations.data.rc.step = CAL_RC_RANGE_DONE;
			}
			*/
			command_result = MAV_RESULT_ACCEPTED;
		} else {
			mavlink_queue_broadcast_error(
				"[SENSOR] Automatic calibration in progress" );
			command_result = MAV_RESULT_DENIED;
		}
	} else if ( _calibrations.type == CAL_NONE ) {
		_calibrations.req_sysid = sysid;
		_calibrations.req_compid = compid;

		command_result = MAV_RESULT_DENIED;

		if ( (int)params[0] == CAL_CMD_GYRO ) {
			command_result = calibration_request( CAL_GYRO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
			// XXX: } else if( (int)params[0] == CAL_CMD_GYRO_TEMP ) {
			// XXX: TODO: GYRO TEMP
		} else if ( (int)params[1] == CAL_CMD_MAG ) {
			command_result = calibration_request( CAL_MAG ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if ( (int)params[2] == CAL_CMD_PRESSURE_GND ) {
			command_result = calibration_request( CAL_GND_PRESSURE ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if ( (int)params[3] == CAL_CMD_RC ) {
			command_result = calibration_request( CAL_RC ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
			//: } else if( (int)params[3] == CAL_CMD_RC_TRIM ) {
			// XXX: RC is done during normal RC cal, maybe it shouldn't?
		} else if ( (int)params[4] == CAL_CMD_ACCEL ) {
			command_result = calibration_request( CAL_ACCEL ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if ( (int)params[4] == CAL_CMD_ACCEL_LEVEL ) {
			command_result = calibration_request( CAL_LEVEL_HORIZON ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
			// XXX: } else if( (int)params[4] == CAL_CMD_ACCEL_TEMP ) {
			// XXX: TODO: ACCEL TEMP
		} else if ( (int)params[5] == CAL_CMD_COMPASS_MOTOR ) {
			command_result = calibration_request( CAL_INTER ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
			// XXX:} else if( (int)params[5] == CAL_CMD_AIRPSEED ) {
			// XXX: TODO: Airpspeed?
		} else if ( (int)params[6] == CAL_CMD_ESC ) {
			// Manually reset calibration mode
			if ( set_param_uint( PARAM_DO_ESC_CAL, 1 ) ) {
				write_params();

				command_result = MAV_RESULT_ACCEPTED;
				mavlink_queue_broadcast_notice(
					"[SENSOR] ESC cal will be run next reboot" );
			} else {
				mavlink_queue_broadcast_error( "[SENSOR] Failed to configure ESC cal!" );
			}
		} else if ( (int)params[6] == CAL_CMD_BAROMETER ) {
			command_result = calibration_request( CAL_BARO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		}
	} else {
		mavlink_queue_broadcast_error( "[SENSOR] Calibration already in progress" );
	}

	return command_result;
}

#ifdef __cplusplus
}
#endif
