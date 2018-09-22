#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "sensors.h"

sensor_calibration_t _sensor_calibration;

MAV_RESULT mavlink_handle_command_long_preflight_calibration( uint8_t port, float *params, uint8_t sysid, uint8_t compid ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	if(_system_status.state == MAV_STATE_CALIBRATING) {	//XXX: Only allow one calibration request at a time
		if( ( (int)params[4] == SENSOR_CAL_CMD_ACCEL) && ( _sensor_calibration.type == SENSOR_CAL_ACCEL ) ) {
			_sensor_calibration.data.accel.waiting = false;
			command_result = MAV_RESULT_ACCEPTED;
		} else if( ( (int)params[3] == SENSOR_CAL_CMD_RC) && ( _sensor_calibration.type == SENSOR_CAL_RC ) ) {
			if(_sensor_calibration.data.rc.waiting) {
				_sensor_calibration.data.rc.waiting = false;
			} else if(_sensor_calibration.data.rc.step == SENSOR_CAL_RC_RANGE_EXTREMES) {
				_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_DONE;
			}

			command_result = MAV_RESULT_ACCEPTED;
		} else {
			mavlink_queue_broadcast_error("[SENSOR] Automatic calibration in progress");
			command_result = MAV_RESULT_DENIED;
		}
	} else if ( _sensor_calibration.type == SENSOR_CAL_NONE ) {
		_sensor_calibration.req_sysid = sysid;
		_sensor_calibration.req_compid = compid;

		command_result = MAV_RESULT_DENIED;

		if( (int)params[0] == SENSOR_CAL_CMD_GYRO ) {
			command_result = sensors_request_cal( SENSOR_CAL_GYRO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		//XXX: } else if( (int)params[0] == SENSOR_CAL_CMD_GYRO_TEMP ) {
		//XXX: TODO: GYRO TEMP
		} else if( (int)params[1] == SENSOR_CAL_CMD_MAG ) {
			command_result = sensors_request_cal( SENSOR_CAL_MAG ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if( (int)params[2] == SENSOR_CAL_CMD_PRESSURE_GND) {
			command_result = sensors_request_cal( SENSOR_CAL_GND_PRESSURE ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if( (int)params[3] == SENSOR_CAL_CMD_RC ) {
			command_result = sensors_request_cal( SENSOR_CAL_RC ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		//: } else if( (int)params[3] == SENSOR_CAL_CMD_RC_TRIM ) {
		//XXX: RC is done during normal RC cal, maybe it shouldn't?
		} else if( (int)params[4] == SENSOR_CAL_CMD_ACCEL ) {
			command_result = sensors_request_cal( SENSOR_CAL_ACCEL ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		} else if( (int)params[4] == SENSOR_CAL_CMD_ACCEL_LEVEL ) {
			command_result = sensors_request_cal( SENSOR_CAL_LEVEL_HORIZON ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		//XXX: } else if( (int)params[4] == SENSOR_CAL_CMD_ACCEL_TEMP ) {
		//XXX: TODO: ACCEL TEMP
		} else if( (int)params[5] == SENSOR_CAL_CMD_COMPASS_MOTOR ) {
			command_result = sensors_request_cal( SENSOR_CAL_INTER ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		//XXX:} else if( (int)params[5] == SENSOR_CAL_CMD_AIRPSEED ) {
		//XXX: TODO: Airpspeed?
		} else if( (int)params[6] == SENSOR_CAL_CMD_ESC ) {
			//Manually reset calibration mode
			if( set_param_uint( PARAM_DO_ESC_CAL, 1 ) ) {
				write_params();

				command_result = MAV_RESULT_ACCEPTED;
				mavlink_queue_broadcast_notice("[SENSOR] ESC cal will be run next reboot");
			} else {
				mavlink_queue_broadcast_error("[SENSOR] Failed to configure ESC cal!");
			}
		} else if( (int)params[6] == SENSOR_CAL_CMD_BAROMETER ) {
			command_result = sensors_request_cal( SENSOR_CAL_BARO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
		}
	} else {
		mavlink_queue_broadcast_error("[SENSOR] Calibration already in progress");
	}


	return command_result;
}
