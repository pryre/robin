#ifdef __cplusplus
extern "C" {
#endif

#include "calibration.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_ppm.h"
#include "mavlink_system.h"
#include "sensors.h"

#include "fix16.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"

#include <stdlib.h>

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_rc( void ) {
	bool failed = false;

	if ( !_calibrations.data.rc.waiting ) {
		switch ( _calibrations.data.rc.step ) {
		case CAL_RC_RANGE_INIT: {
			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_MIDDOWN;
			mavlink_queue_broadcast_notice(
				"[SENSOR] Set RC to stick and switch centres" );

			break;
		}
		case CAL_RC_RANGE_MIDDOWN: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MID] = _sensors.rc_input.raw[i];

				if ( ( _sensors.rc_input.raw[i] < 1300 ) || ( _sensors.rc_input.raw[i] > 1700 ) ) {
					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					snprintf( text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
							  "[SENSOR] Possible bad trim on channel: %d", i + 1 );
					mavlink_queue_broadcast_error( text );
				}
			}

			set_param_uint( PARAM_RC1_MID,
							_calibrations.data.rc.ranges[0][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC2_MID,
							_calibrations.data.rc.ranges[1][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC3_MID,
							_calibrations.data.rc.ranges[2][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC4_MID,
							_calibrations.data.rc.ranges[3][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC5_MID,
							_calibrations.data.rc.ranges[4][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC6_MID,
							_calibrations.data.rc.ranges[5][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC7_MID,
							_calibrations.data.rc.ranges[6][SENSOR_RC_CAL_MID] );
			set_param_uint( PARAM_RC8_MID,
							_calibrations.data.rc.ranges[7][SENSOR_RC_CAL_MID] );

			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_CORNERS;
			mavlink_queue_broadcast_notice( "[SENSOR] Set RC to lower inner corners" );

			break;
		}
		case CAL_RC_RANGE_CORNERS: {
			uint8_t chan_roll = get_param_uint( PARAM_RC_MAP_ROLL ) - 1;
			uint8_t chan_pitch = get_param_uint( PARAM_RC_MAP_PITCH ) - 1;
			uint8_t chan_yaw = get_param_uint( PARAM_RC_MAP_YAW ) - 1;
			uint8_t chan_throttle = get_param_uint( PARAM_RC_MAP_THROTTLE ) - 1;

			_calibrations.data.rc.rev[chan_roll] = ( _sensors.rc_input.raw[chan_roll] > SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_pitch] = ( _sensors.rc_input.raw[chan_pitch] < SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_yaw] = ( _sensors.rc_input.raw[chan_yaw] < SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_throttle] = ( _sensors.rc_input.raw[chan_throttle] > SENSOR_RC_MIDSTICK );

			set_param_uint( PARAM_RC1_REV, _calibrations.data.rc.rev[0] );
			set_param_uint( PARAM_RC2_REV, _calibrations.data.rc.rev[1] );
			set_param_uint( PARAM_RC3_REV, _calibrations.data.rc.rev[2] );
			set_param_uint( PARAM_RC4_REV, _calibrations.data.rc.rev[3] );
			set_param_uint( PARAM_RC5_REV, _calibrations.data.rc.rev[4] );
			set_param_uint( PARAM_RC6_REV, _calibrations.data.rc.rev[5] );
			set_param_uint( PARAM_RC7_REV, _calibrations.data.rc.rev[6] );
			set_param_uint( PARAM_RC8_REV, _calibrations.data.rc.rev[7] );

			_calibrations.data.rc.step = CAL_RC_RANGE_EXTREMES;
			mavlink_queue_broadcast_notice(
				"[SENSOR] Move all sticks and switches to extremes" );
			mavlink_queue_broadcast_notice( "[SENSOR] Resend cal command when done" );

			break;
		}
		case CAL_RC_RANGE_EXTREMES: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				uint16_t pwmr = _sensors.rc_input.raw[i];
				_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] = ( pwmr < _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] ) ? pwmr : _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN];
				_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] = ( pwmr > _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] ) ? pwmr : _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX];
			}

			break;
		}
		case CAL_RC_RANGE_DONE: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				if ( ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] > 1300 ) || ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] < 1700 ) ) {

					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					snprintf( text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
							  "[SENSOR] Possible bad min/max on channel: %d", i + 1 );
					mavlink_queue_broadcast_error( text );
				}
			}

			set_param_uint( PARAM_RC1_MIN,
							_calibrations.data.rc.ranges[0][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC1_MAX,
							_calibrations.data.rc.ranges[0][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC2_MIN,
							_calibrations.data.rc.ranges[1][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC2_MAX,
							_calibrations.data.rc.ranges[1][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC3_MIN,
							_calibrations.data.rc.ranges[2][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC3_MAX,
							_calibrations.data.rc.ranges[2][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC4_MIN,
							_calibrations.data.rc.ranges[3][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC4_MAX,
							_calibrations.data.rc.ranges[3][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC5_MIN,
							_calibrations.data.rc.ranges[4][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC5_MAX,
							_calibrations.data.rc.ranges[4][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC6_MIN,
							_calibrations.data.rc.ranges[5][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC6_MAX,
							_calibrations.data.rc.ranges[5][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC7_MIN,
							_calibrations.data.rc.ranges[6][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC7_MAX,
							_calibrations.data.rc.ranges[6][SENSOR_RC_CAL_MAX] );
			set_param_uint( PARAM_RC8_MIN,
							_calibrations.data.rc.ranges[7][SENSOR_RC_CAL_MIN] );
			set_param_uint( PARAM_RC8_MAX,
							_calibrations.data.rc.ranges[7][SENSOR_RC_CAL_MAX] );

			calibration_done();
			sensors_update_rc_cal();
			mavlink_queue_broadcast_notice( "[SENSOR] RC calibration complete!" );

			break;
		}
		default: {
			failed = true;

			calibration_done();
			sensors_update_rc_cal();
			mavlink_queue_broadcast_error( "[SENSOR] Issue with RC cal, aborting" );

			break;
		}
		}

		// If we are waiting inside this loop, a step has recently finished
		if ( _calibrations.data.rc.waiting ) {
			mavlink_message_t msg;
			mavlink_prepare_command_ack(
				&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_IN_PROGRESS,
				_calibrations.req_sysid, _calibrations.req_compid,
				_calibrations.data.rc.step / CAL_ACCEL_DONE );
			lpq_queue_broadcast_msg( &msg );

			status_buzzer_success();
		}
	}

	return !failed;
}

#ifdef __cplusplus
}
#endif
