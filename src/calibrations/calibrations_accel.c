#ifdef __cplusplus
extern "C" {
#endif

#include "calibration.h"
#include "drivers/drv_status_io.h"
#include "mavlink_system.h"
#include "sensors.h"

#include "fix16.h"
#include "fixextra.h"
#include "params.h"
#include "safety.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_accel( void ) {
	bool failed = false;

	if ( !_calibrations.data.accel.waiting ) {
		if ( _calibrations.data.accel.accel_cal_step == CAL_ACCEL_INIT ) {
			_calibrations.data.accel.waiting = true;
			_calibrations.data.accel.accel_cal_step = CAL_ACCEL_Z_DOWN;
			mavlink_queue_broadcast_notice(
				"[SENSOR] Ready for Z-Down, send accel cal" );
		} else if ( _calibrations.data.accel.accel_cal_step == CAL_ACCEL_DONE ) {
			//==-- bias = sum / count
			//==-- //TODO: bias = (sum - (temp_comp*temp_sum)) / count
			int32_t x_bias = _calibrations.data.accel.data.x_flat_av_sum / 4;
			int32_t y_bias = _calibrations.data.accel.data.y_flat_av_sum / 4;
			int32_t z_bias = _calibrations.data.accel.data.z_flat_av_sum / 4;

			// Correct for measurement biases
			fix16_t accel_x_down_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.x_down_av - get_param_int( PARAM_ACC_X_BIAS ) ),
												 _sensors.imu.accel_scale );
			fix16_t accel_y_down_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.y_down_av - get_param_int( PARAM_ACC_Y_BIAS ) ),
												 _sensors.imu.accel_scale );
			fix16_t accel_z_down_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.z_down_av - get_param_int( PARAM_ACC_Z_BIAS ) ),
												 _sensors.imu.accel_scale );
			fix16_t accel_x_up_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.x_up_av - get_param_int( PARAM_ACC_X_BIAS ) ),
											   _sensors.imu.accel_scale );
			fix16_t accel_y_up_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.y_up_av - get_param_int( PARAM_ACC_Y_BIAS ) ),
											   _sensors.imu.accel_scale );
			fix16_t accel_z_up_1g = fix16_mul( fix16_from_int( _calibrations.data.accel.data.z_up_av - get_param_int( PARAM_ACC_Z_BIAS ) ),
											   _sensors.imu.accel_scale );

			fix16_t accel_x_scale_p = fix16_div( _fc_gravity, accel_x_down_1g );
			fix16_t accel_y_scale_p = fix16_div( _fc_gravity, accel_y_down_1g );
			fix16_t accel_z_scale_p = fix16_div( _fc_gravity, accel_z_down_1g );
			fix16_t accel_x_scale_n = fix16_div( -_fc_gravity, accel_x_up_1g );
			fix16_t accel_y_scale_n = fix16_div( -_fc_gravity, accel_y_up_1g );
			fix16_t accel_z_scale_n = fix16_div( -_fc_gravity, accel_z_up_1g );

			// Sanity check to make sure the scaling is positive and not far too large
			if ( ( ( accel_x_scale_p > _fc_0_5 ) && ( accel_x_scale_p < _fc_2 ) ) && ( ( accel_y_scale_p > _fc_0_5 ) && ( accel_y_scale_p < _fc_2 ) ) && ( ( accel_z_scale_p > _fc_0_5 ) && ( accel_z_scale_p < _fc_2 ) ) && ( ( accel_x_scale_n > _fc_0_5 ) && ( accel_x_scale_n < _fc_2 ) ) && ( ( accel_y_scale_n > _fc_0_5 ) && ( accel_y_scale_n < _fc_2 ) ) && ( ( accel_z_scale_n > _fc_0_5 ) && ( accel_z_scale_n < _fc_2 ) ) ) {

				set_param_int( PARAM_ACC_X_BIAS, x_bias );
				set_param_int( PARAM_ACC_Y_BIAS, y_bias );
				set_param_int( PARAM_ACC_Z_BIAS, z_bias );

				set_param_fix16( PARAM_ACC_X_SCALE_POS, accel_x_scale_p );
				set_param_fix16( PARAM_ACC_Y_SCALE_POS, accel_y_scale_p );
				set_param_fix16( PARAM_ACC_Z_SCALE_POS, accel_z_scale_p );
				set_param_fix16( PARAM_ACC_X_SCALE_NEG, accel_x_scale_n );
				set_param_fix16( PARAM_ACC_Y_SCALE_NEG, accel_y_scale_n );
				set_param_fix16( PARAM_ACC_Z_SCALE_NEG, accel_z_scale_n );

				mavlink_queue_broadcast_notice( "[SENSOR] Accel calibration complete!" );
			} else {
				failed = true;

				mavlink_queue_broadcast_error(
					"[SENSOR] Accel calibration failed, bad scaling!" );
			}

			calibration_done();
		} else {
			_calibrations.data.accel.data.t_sum += _sensors.imu.temp_raw;
			_calibrations.data.accel.data.x_sum += _sensors.imu.accel_raw.x;
			_calibrations.data.accel.data.y_sum += _sensors.imu.accel_raw.y;
			_calibrations.data.accel.data.z_sum += _sensors.imu.accel_raw.z;

			_calibrations.data.accel.data.count++;

			if ( _calibrations.data.accel.data.count >= get_param_uint( PARAM_CAL_IMU_PASSES ) ) {
				switch ( _calibrations.data.accel.accel_cal_step ) {
				case CAL_ACCEL_Z_DOWN: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_flat_av_sum += _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_flat_av_sum += _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_down_av = _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_Z_UP;
					mavlink_queue_broadcast_notice(
						"[SENSOR] Ready for Z-Up, send accel cal" );

					break;
				}
				case CAL_ACCEL_Z_UP: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_flat_av_sum += _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_flat_av_sum += _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_up_av = _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_Y_DOWN;
					mavlink_queue_broadcast_notice(
						"[SENSOR] Ready for Y-Down, send accel cal" );

					break;
				}
				case CAL_ACCEL_Y_DOWN: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_flat_av_sum += _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_down_av = _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_flat_av_sum += _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_Y_UP;
					mavlink_queue_broadcast_notice(
						"[SENSOR] Ready for Y-Up, send accel cal" );

					break;
				}
				case CAL_ACCEL_Y_UP: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_flat_av_sum += _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_up_av = _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_flat_av_sum += _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_X_DOWN;
					mavlink_queue_broadcast_notice(
						"[SENSOR] Ready for X-Down, send accel cal" );

					break;
				}
				case CAL_ACCEL_X_DOWN: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_down_av = _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_flat_av_sum += _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_flat_av_sum += _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_X_UP;
					mavlink_queue_broadcast_notice(
						"[SENSOR] Ready for X-Up, send accel cal" );

					break;
				}
				case CAL_ACCEL_X_UP: {
					_calibrations.data.accel.data.t_av_sum += _calibrations.data.accel.data.t_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.x_up_av = _calibrations.data.accel.data.x_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.y_flat_av_sum += _calibrations.data.accel.data.y_sum / _calibrations.data.accel.data.count;
					_calibrations.data.accel.data.z_flat_av_sum += _calibrations.data.accel.data.z_sum / _calibrations.data.accel.data.count;

					_calibrations.data.accel.accel_cal_step = CAL_ACCEL_DONE;

					break;
				}
				default: {
					failed = true;

					calibration_done();
					mavlink_queue_broadcast_error(
						"[SENSOR] Issue with accel cal, aborting" );

					break;
				}
				}

				mavlink_message_t msg;
				mavlink_prepare_command_ack(
					&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_IN_PROGRESS,
					_calibrations.req_sysid, _calibrations.req_compid,
					_calibrations.data.accel.accel_cal_step / CAL_ACCEL_DONE );
				lpq_queue_broadcast_msg( &msg );

				status_buzzer_success();

				// Reset for the next calibration
				_calibrations.data.accel.data.count = 0;
				_calibrations.data.accel.data.x_sum = 0;
				_calibrations.data.accel.data.y_sum = 0;
				_calibrations.data.accel.data.z_sum = 0;
				_calibrations.data.accel.data.t_sum = 0;

				// Make sure we wait for confirmation before continuing
				if ( _calibrations.data.accel.accel_cal_step != CAL_ACCEL_DONE )
					_calibrations.data.accel.waiting = true;
			}
		}
	}

	return !failed;
}

#ifdef __cplusplus
}
#endif
