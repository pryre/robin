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

#include "robin_itoa.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

static uint32_t uint32_min(uint32_t a, uint32_t b) {
	return ( a < b ) ? a : b;
}

static uint32_t uint32_max(uint32_t a, uint32_t b) {
	return ( a > b ) ? a : b;
}

bool calibrate_rc( void ) {
	bool failed = false;

	if ( !_calibrations.data.rc.waiting ) {
		switch ( _calibrations.data.rc.step ) {
		case CAL_RC_RANGE_INIT: {
			_calibrations.data.rc.is_stick = 0;
			_calibrations.data.rc.is_switch = 0;

			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_REVERSE;
			mavlink_queue_broadcast_notice("[SENSOR] Set RC sticks to lower inner corners");
			mavlink_queue_broadcast_notice("[SENSOR] Set RC switch to defaults");

			break;
		}
		case CAL_RC_RANGE_REVERSE: {
			uint8_t chan_roll = get_param_uint( PARAM_RC_MAP_ROLL ) - 1;
			uint8_t chan_pitch = get_param_uint( PARAM_RC_MAP_PITCH ) - 1;
			uint8_t chan_yaw = get_param_uint( PARAM_RC_MAP_YAW ) - 1;
			uint8_t chan_throttle = get_param_uint( PARAM_RC_MAP_THROTTLE ) - 1;

			//Handle stick inputs
			_calibrations.data.rc.rev[chan_roll] = ( _sensors.rc_input.raw[chan_roll] > SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_pitch] = ( _sensors.rc_input.raw[chan_pitch] < SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_yaw] = ( _sensors.rc_input.raw[chan_yaw] < SENSOR_RC_MIDSTICK );
			_calibrations.data.rc.rev[chan_throttle] = ( _sensors.rc_input.raw[chan_throttle] > SENSOR_RC_MIDSTICK );

			//Handle remaining inputs
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				if( (i != chan_roll) && (i != chan_pitch) && (i != chan_yaw) && (i != chan_throttle) )
					_calibrations.data.rc.rev[i] = ( _sensors.rc_input.raw[i] > SENSOR_RC_MIDSTICK );
			}

			set_param_uint( PARAM_RC1_REV, _calibrations.data.rc.rev[0] );
			set_param_uint( PARAM_RC2_REV, _calibrations.data.rc.rev[1] );
			set_param_uint( PARAM_RC3_REV, _calibrations.data.rc.rev[2] );
			set_param_uint( PARAM_RC4_REV, _calibrations.data.rc.rev[3] );
			set_param_uint( PARAM_RC5_REV, _calibrations.data.rc.rev[4] );
			set_param_uint( PARAM_RC6_REV, _calibrations.data.rc.rev[5] );
			set_param_uint( PARAM_RC7_REV, _calibrations.data.rc.rev[6] );
			set_param_uint( PARAM_RC8_REV, _calibrations.data.rc.rev[7] );

			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_EXTREMES_STICK;
			mavlink_queue_broadcast_notice( "[SENSOR] Move all sticks to extremes" );

			break;
		}
		case CAL_RC_RANGE_EXTREMES_STICK: {
			//XXX: Everything is handled at once in CAL_RC_RANGE_EXTREMES_SWITCH

			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_EXTREMES_SWITCH;
			mavlink_queue_broadcast_notice( "[SENSOR] Move all switches to extremes" );

			break;
		}
		case CAL_RC_RANGE_EXTREMES_SWITCH: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				if( ( ( _calibrations.data.rc.is_stick & (1 << i) ) ||
					  ( _calibrations.data.rc.is_switch & (1 << i) ) ) &&
					( ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] > 1300 ) ||
					  ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] < 1700 ) ) ) {
					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Possible bad min/max on channel: ";
					char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1, i + 1 , 10);
					strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
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


			_calibrations.data.rc.waiting = true;
			_calibrations.data.rc.step = CAL_RC_RANGE_MID;
			mavlink_queue_broadcast_notice( "[SENSOR] Move all sticks to movement center" );

			break;
		}
		case CAL_RC_RANGE_MID: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				uint32_t midr = _sensors.rc_input.raw[i];

				if ( _calibrations.data.rc.is_stick & (1 << i) ) {
					if ( ( midr < 1300 ) || ( midr > 1700 ) ) {
						char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Possible bad trim on channel: ";
						char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1, i + 1 , 10);
						strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
						mavlink_queue_broadcast_error( text );
					}
				} else if ( _calibrations.data.rc.is_switch & (1 << i) ) {
					//Use half-way of range for sticks
					midr = ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] - _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN]) / 2;
				} else {
					//Set to mid-stick as fallback
					midr = SENSOR_RC_MIDSTICK;
				}

				_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MID] = midr;
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

			//XXX: Calibration done!
			_calibrations.data.rc.step = CAL_RC_RANGE_DONE;

			break;
		}
		case CAL_RC_RANGE_DONE: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				if( !( _calibrations.data.rc.is_stick & (1 << i) ) && !( _calibrations.data.rc.is_switch & (1 << i) ) ) {
					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] No input detected on channel: ";
					char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1, i + 1 , 10);
					strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
					mavlink_queue_broadcast_error( text );
				}
			}

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

		// If a step has re-enabled waiting, we finished a step
		if ( _calibrations.data.rc.waiting ) {
			mavlink_message_t msg;
			mavlink_prepare_command_ack(
				&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_IN_PROGRESS,
				_calibrations.req_sysid, _calibrations.req_compid,
				_calibrations.data.rc.step / CAL_RC_RANGE_DONE );
			lpq_queue_broadcast_msg( &msg );

			status_buzzer_success();

			mavlink_queue_broadcast_notice("[SENSOR] Resend RC CAL when ready");
		}
	} else {
		//XXX: Handle cases that wait for data
		switch( _calibrations.data.rc.step ) {
		case CAL_RC_RANGE_EXTREMES_STICK: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				uint16_t pwmr = _sensors.rc_input.raw[i];
				//If the input is outside normal value, update ranges
				if ( ( pwmr < 1300 ) || ( pwmr > 1700 ) ) {
					_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] = uint32_min(pwmr, _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN]);
					_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] = uint32_max(pwmr, _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX]);
				}

				//If the range is large, mark it as a stick input
				if( ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] - _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] ) > 500 ) {
					_calibrations.data.rc.is_stick |= 1 << i;
				}
			}

			break;
		}
		case CAL_RC_RANGE_EXTREMES_SWITCH: {
			for ( int i = 0; i < DRV_PPM_MAX_INPUTS; i++ ) {
				//Don't recheck/override stick inpus
				if( !( _calibrations.data.rc.is_stick & (1 << i) ) ) {
					uint16_t pwmr = _sensors.rc_input.raw[i];
					//If the input is outside normal value, update ranges
					if ( ( pwmr < 1300 ) || ( pwmr > 1700 ) ) {
						_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] = uint32_min(pwmr, _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN]);
						_calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] = uint32_max(pwmr, _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX]);
					}

					//If the range is large, mark it as a stick input
					if( ( _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MAX] - _calibrations.data.rc.ranges[i][SENSOR_RC_CAL_MIN] ) > 500 ) {
						_calibrations.data.rc.is_switch |= 1 << i;
					}
				}
			}

			break;
		}
		default:
			//Waiting for user input, do nothing
			break;
		}
	}

	return !failed;
}

#ifdef __cplusplus
}
#endif
