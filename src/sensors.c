#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sensors.h"
#include "estimator.h"
#include "safety.h"
#include "params.h"
#include "mavlink_system.h"
#include "breezystm32.h"
#include "drv_mpu.h"
#include "gpio.h"

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"
#include "fixextra.h"

#include "stdio.h"

//Number of itterations of averaging to use with IMU calibrations
//#define SENSOR_CAL_IMU_PASSES 1000

//==-- Local Variables
int32_t _imu_time_read = 0;
static volatile uint8_t accel_status = 0;
static volatile uint8_t gyro_status = 0;
static volatile uint8_t temp_status = 0;

static int16_t read_accel_raw[3];
static int16_t read_gyro_raw[3];
static volatile int16_t read_temp_raw;

sensor_readings_t _sensors;
uint8_t _sensor_calibration;
sensor_calibration_data_t _sensor_cal_data;

system_status_t _system_status;

//==-- Functions
static void clock_init(void) {
	_sensors.clock.present = true;

	_sensors.clock.start = 0;
	_sensors.clock.end = 0;
	_sensors.clock.dt = 0;

	_sensors.clock.counter = 0;
	_sensors.clock.max = 0;
	_sensors.clock.min = 1000;

	_sensors.clock.imu_time_read = 0;

	_sensors.clock.rt_offset_ns = 0;
	_sensors.clock.rt_drift = 1.0;
	_sensors.clock.rt_ts_last = 0;
	_sensors.clock.rt_tc_last = 0;
	_sensors.clock.rt_sync_last = 0;
}

static void sensors_imu_poll(void) {
	//==-- Timing setup get loop time
	_imu_time_read = micros();

	mpu_request_async_accel_read(read_accel_raw, &accel_status);
	mpu_request_async_gyro_read(read_gyro_raw, &gyro_status);
	mpu_request_async_temp_read(&(read_temp_raw), &temp_status);
}

static void sensor_status_init(sensor_status_t *status, bool sensor_present) {
	status->present = sensor_present;
	status->new_data = false;
	status->time_read = 0;
}

static void sensors_cal_init(void) {
	_sensor_calibration = SENSOR_CAL_NONE;

	_sensor_cal_data.gyro.count = 0;
	_sensor_cal_data.gyro.sum_x = 0;
	_sensor_cal_data.gyro.sum_y = 0;
	_sensor_cal_data.gyro.sum_z = 0;

	_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_INIT;
	_sensor_cal_data.accel.waiting = false;
	_sensor_cal_data.accel.data.count = 0;

	_sensor_cal_data.accel.data.t_sum = 0;
	_sensor_cal_data.accel.data.x_sum = 0;
	_sensor_cal_data.accel.data.y_sum = 0;
	_sensor_cal_data.accel.data.z_sum = 0;

	_sensor_cal_data.accel.data.t_av_sum = 0;
	_sensor_cal_data.accel.data.x_flat_av_sum = 0;
	_sensor_cal_data.accel.data.y_flat_av_sum = 0;
	_sensor_cal_data.accel.data.z_flat_av_sum = 0;
	_sensor_cal_data.accel.data.x_up_av = 0;
	_sensor_cal_data.accel.data.x_down_av = 0;
	_sensor_cal_data.accel.data.y_up_av = 0;
	_sensor_cal_data.accel.data.y_down_av = 0;
	_sensor_cal_data.accel.data.z_up_av = 0;
	_sensor_cal_data.accel.data.z_down_av = 0;

	_sensor_cal_data.accel.temp_scale = fix16_from_float(340.0f);
	_sensor_cal_data.accel.temp_shift = fix16_from_float(36.53f);
}

void sensors_init_internal(void) {
	//==-- IMU-MPU6050
	sensor_status_init(&_sensors.imu.status, (bool)get_param_uint(PARAM_SENSOR_IMU_CBRK));
	mpu_register_interrupt_cb(&sensors_imu_poll, get_param_uint(PARAM_BOARD_REVISION));
	
	switch(get_param_uint(PARAM_BOARD_REVISION)) {
		case 5: {
			//Get the 1g gravity scale (raw->g's)
			_sensor_cal_data.accel.acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
			break;
		}
		case 6: {
			//Get the 1g gravity scale (raw->g's)
			_sensor_cal_data.accel.acc1G = mpu6500_init(INV_FSR_8G, INV_FSR_2000DPS);
			break;
		}
		default: {
			//Could not determine IMU!
			failureMode(5);
		}
	}
	
	_sensors.imu.accel_scale = fix16_div(_fc_gravity, fix16_from_int(_sensor_cal_data.accel.acc1G));	//Get the m/s scale (raw->g's->m/s/s)
	_sensors.imu.gyro_scale = fix16_from_float(MPU_GYRO_SCALE);	//Get radians scale (raw->rad/s)


	//==-- Calibrations
	sensors_cal_init();

	//==-- Timer
	clock_init();
}

void sensors_init_external(void) {
	//==-- Mag
	sensor_status_init( &_sensors.mag.status, (bool)get_param_uint( PARAM_SENSOR_MAG_CBRK ) );

	//==-- Baro
	sensor_status_init( &_sensors.baro.status, (bool)get_param_uint( PARAM_SENSOR_BARO_CBRK ) );

	//==-- Sonar
	sensor_status_init( &_sensors.sonar.status, (bool)get_param_uint( PARAM_SENSOR_SONAR_CBRK ) );

	//==-- External Pose
	sensor_status_init( &_sensors.ext_pose.status, false );

	//==-- Safety button
	sensor_status_init(&_sensors.safety_button.status, (bool)get_param_uint(PARAM_SENSOR_SAFETY_CBRK));
	_sensors.safety_button.gpio_p = GPIOA;
	_sensors.safety_button.pin = Pin_6;

	gpio_config_t safety_button_cfg;
    safety_button_cfg.pin = _sensors.safety_button.pin;
    safety_button_cfg.mode = Mode_IPU;
    safety_button_cfg.speed = Speed_2MHz;
    gpioInit(_sensors.safety_button.gpio_p, &safety_button_cfg);

	_sensors.safety_button.state = false;
	_sensors.safety_button.period_us = 100000;		//100ms update rate
	_sensors.safety_button.time_db_read = 0;
	_sensors.safety_button.period_db_us = 50000;	//50ms debounce period
	_sensors.safety_button.state_db = false;
}

bool sensors_read(void) {
	bool imu_job_complete = false;

	//Check IMU status
	if(accel_status == I2C_JOB_COMPLETE && gyro_status == I2C_JOB_COMPLETE && temp_status == I2C_JOB_COMPLETE) {
		imu_job_complete = true;
		accel_status = I2C_JOB_DEFAULT;
		gyro_status = I2C_JOB_DEFAULT;
		temp_status = I2C_JOB_DEFAULT;

		_sensors.clock.imu_time_read = _imu_time_read;

		//XXX: Some values need to be inversed to be in the NED frame
		_sensors.imu.accel_raw.x = -read_accel_raw[0];
		_sensors.imu.accel_raw.y = read_accel_raw[1];
		_sensors.imu.accel_raw.z = read_accel_raw[2];

		_sensors.imu.gyro_raw.x = read_gyro_raw[0];
		_sensors.imu.gyro_raw.y = -read_gyro_raw[1];
		_sensors.imu.gyro_raw.z = -read_gyro_raw[2];

		_sensors.imu.temp_raw = read_temp_raw;
	}

	//TODO: Check other status
	//TODO: May need to offset these so they don't all check at once(?)

	//Return the results
	return imu_job_complete;
}

uint32_t sensors_clock_ls_get(void) {
	return _sensors.clock.start;
}

void sensors_clock_ls_set(uint32_t time_us) {
	_sensors.clock.start = time_us;
}

void sensors_clock_update(uint32_t time_us) {
	_sensors.clock.end = time_us;
	_sensors.clock.dt = _sensors.clock.end - _sensors.clock.start;
	_sensors.clock.average_time += _sensors.clock.dt;
	_sensors.clock.counter++;
	_sensors.clock.max = (_sensors.clock.dt > _sensors.clock.max) ? _sensors.clock.dt : _sensors.clock.max;
	_sensors.clock.min = (_sensors.clock.dt < _sensors.clock.min) ? _sensors.clock.dt : _sensors.clock.min;
}

//==-- Low Pass Filter for time offsets
int64_t sensors_clock_smooth_time_skew(int64_t tc, int64_t tn) {
	/* The closer alpha is to 1.0, the faster the moving
	 * average updates in response to new offset samples.
	 */
	//Do this in floating point as fix16_t does not have an easy interface for uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return (int64_t)( alpha * tn ) + (int64_t)( ( 1.0f - alpha ) * tc );
}

float sensors_clock_smooth_time_drift(float tc, float tn) {
	/* The closer alpha is to 1.0, the faster the moving
	 * average updates in response to new offset samples.
	 */
	//Do this in floating point as fix16_t does not have an easy interface for uint64_t
	float alpha = fix16_to_float( get_param_fix16( PARAM_TIMESYNC_ALPHA ) );
	return (int64_t)( alpha * tn ) + (int64_t)( ( 1.0f - alpha ) * tc );
}

uint64_t sensors_clock_rt_get(void) {
	return (uint64_t)( micros() * 1000LL ) + _sensors.clock.rt_offset_ns;
}

uint32_t sensors_clock_imu_int_get(void) {
	return _sensors.clock.imu_time_read;
}

//TODO: This does not take into account temperature
//Returns true if all calibrations are complete
static bool sensors_calibrate(void) {
	bool cal_mode_error = false;

	switch(_sensor_calibration) {
		case SENSOR_CAL_GYRO: {
			_sensor_cal_data.gyro.sum_x += _sensors.imu.gyro_raw.x;
			_sensor_cal_data.gyro.sum_y += _sensors.imu.gyro_raw.y;
			_sensor_cal_data.gyro.sum_z += _sensors.imu.gyro_raw.z;

			_sensor_cal_data.gyro.count++;
					
			if (_sensor_cal_data.gyro.count >= get_param_uint(PARAM_CAL_IMU_PASSES)) {
				set_param_int(PARAM_GYRO_X_BIAS, (_sensor_cal_data.gyro.sum_x / _sensor_cal_data.gyro.count));
				set_param_int(PARAM_GYRO_Y_BIAS, (_sensor_cal_data.gyro.sum_y / _sensor_cal_data.gyro.count));
				set_param_int(PARAM_GYRO_Z_BIAS, (_sensor_cal_data.gyro.sum_z / _sensor_cal_data.gyro.count));

				_sensor_cal_data.gyro.count = 0;
				_sensor_cal_data.gyro.sum_x = 0;
				_sensor_cal_data.gyro.sum_y = 0;
				_sensor_cal_data.gyro.sum_z = 0;

				reset_adaptive_gyro_bias();

				_sensor_calibration ^= SENSOR_CAL_GYRO;	//Turn off SENSOR_CAL_GYRO bit
				//TODO: "we could do some sanity checking here if we wanted to."

				mavlink_queue_broadcast_notice("[SENSOR] Gyro calibration complete!");
				status_buzzer_success();
			}
			
			break;
		}
		case SENSOR_CAL_MAG: {
			//TODO

			//======== TODO! REMOVE THIS LATER ========//
			/*
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
			snprintf(text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "[CAL]:%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f",
					 fix16_to_float(_sensors.imu.accel.x),
					 fix16_to_float(_sensors.imu.accel.y),
					 fix16_to_float(_sensors.imu.accel.z),
					 fix16_to_float(_sensors.imu.gyro.x),
					 fix16_to_float(_sensors.imu.gyro.y),
					 fix16_to_float(_sensors.imu.gyro.z));
			mavlink_queue_broadcast_error(text);
			*/
			//======== TODO! REMOVE THIS LATER ========//

			_sensor_calibration ^= SENSOR_CAL_MAG;

			break;
		}
		case SENSOR_CAL_BARO: {
			//TODO
			_sensor_calibration ^= SENSOR_CAL_BARO;

			break;
		}
		case SENSOR_CAL_RC: {
			//TODO
			_sensor_calibration ^= SENSOR_CAL_RC;

			break;
		}
		case SENSOR_CAL_ACCEL: {
			if(!_sensor_cal_data.accel.waiting) {
				if( _sensor_cal_data.accel.accel_cal_step == SENSOR_CAL_ACCEL_INIT ) {
					_sensor_cal_data.accel.waiting = true;
					_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Z_DOWN;
					mavlink_queue_broadcast_notice("[SENSOR] Ready for Z-Down, send accel cal");
				} else if ( _sensor_cal_data.accel.accel_cal_step == SENSOR_CAL_ACCEL_DONE ) {
					//==-- bias = sum / count
					//==-- //TODO: bias = (sum - (temp_comp*temp_sum)) / count
					int32_t x_bias = _sensor_cal_data.accel.data.x_flat_av_sum / 4;
					int32_t y_bias = _sensor_cal_data.accel.data.y_flat_av_sum / 4;
					int32_t z_bias = _sensor_cal_data.accel.data.z_flat_av_sum / 4;

					set_param_int( PARAM_ACC_X_BIAS, x_bias );
					set_param_int( PARAM_ACC_Y_BIAS, y_bias );
					set_param_int( PARAM_ACC_Z_BIAS, z_bias );

					//Correct for measurement biases
					fix16_t accel_x_down_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.x_down_av - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
					fix16_t accel_y_down_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.y_down_av - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
					fix16_t accel_z_down_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.z_down_av - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);
					fix16_t accel_x_up_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.x_up_av - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
					fix16_t accel_y_up_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.y_up_av - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
					fix16_t accel_z_up_1g = fix16_mul(fix16_from_int(_sensor_cal_data.accel.data.z_up_av - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);

					set_param_fix16( PARAM_ACC_X_SCALE_POS, fix16_div( _fc_gravity, accel_x_down_1g ) );
					set_param_fix16( PARAM_ACC_Y_SCALE_POS, fix16_div( _fc_gravity, accel_y_down_1g ) );
					set_param_fix16( PARAM_ACC_Z_SCALE_POS, fix16_div( _fc_gravity, accel_z_down_1g ) );
					set_param_fix16( PARAM_ACC_X_SCALE_NEG, fix16_div( -_fc_gravity, accel_x_up_1g ) );
					set_param_fix16( PARAM_ACC_Y_SCALE_NEG, fix16_div( -_fc_gravity, accel_y_up_1g ) );
					set_param_fix16( PARAM_ACC_Z_SCALE_NEG, fix16_div( -_fc_gravity, accel_z_up_1g ) );

					_sensor_calibration ^= SENSOR_CAL_ACCEL;	//Turn off SENSOR_CAL_ACCEL bit
					//TODO: "we could do some sanity checking here if we wanted to."

					mavlink_queue_broadcast_notice("[SENSOR] Accel calibration complete!");
				} else {
					_sensor_cal_data.accel.data.t_sum += _sensors.imu.temp_raw;
					_sensor_cal_data.accel.data.x_sum += _sensors.imu.accel_raw.x;
					_sensor_cal_data.accel.data.y_sum += _sensors.imu.accel_raw.y;
					_sensor_cal_data.accel.data.z_sum += _sensors.imu.accel_raw.z;

					_sensor_cal_data.accel.data.count++;

					if (_sensor_cal_data.accel.data.count >= get_param_uint(PARAM_CAL_IMU_PASSES)) {
						switch(_sensor_cal_data.accel.accel_cal_step) {
							case SENSOR_CAL_ACCEL_Z_DOWN: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_flat_av_sum += _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_flat_av_sum += _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_down_av = _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Z_UP;
								mavlink_queue_broadcast_notice("[SENSOR] Ready for Z-Up, send accel cal");

								break;
							}
							case SENSOR_CAL_ACCEL_Z_UP: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_flat_av_sum += _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_flat_av_sum += _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_up_av = _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Y_DOWN;
								mavlink_queue_broadcast_notice("[SENSOR] Ready for Y-Down, send accel cal");

								break;
							}
							case SENSOR_CAL_ACCEL_Y_DOWN: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_flat_av_sum += _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_down_av = _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_flat_av_sum += _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_Y_UP;
								mavlink_queue_broadcast_notice("[SENSOR] Ready for Y-Up, send accel cal");

								break;
							}
							case SENSOR_CAL_ACCEL_Y_UP: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_flat_av_sum += _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_up_av = _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_flat_av_sum += _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_X_DOWN;
								mavlink_queue_broadcast_notice("[SENSOR] Ready for X-Down, send accel cal");

								break;
							}
							case SENSOR_CAL_ACCEL_X_DOWN: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_down_av = _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_flat_av_sum += _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_flat_av_sum += _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_X_UP;
								mavlink_queue_broadcast_notice("[SENSOR] Ready for X-Up, send accel cal");

								break;
							}
							case SENSOR_CAL_ACCEL_X_UP: {
								_sensor_cal_data.accel.data.t_av_sum += _sensor_cal_data.accel.data.t_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.x_up_av = _sensor_cal_data.accel.data.x_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.y_flat_av_sum += _sensor_cal_data.accel.data.y_sum / _sensor_cal_data.accel.data.count;
								_sensor_cal_data.accel.data.z_flat_av_sum += _sensor_cal_data.accel.data.z_sum / _sensor_cal_data.accel.data.count;

								_sensor_cal_data.accel.accel_cal_step = SENSOR_CAL_ACCEL_DONE;

								break;
							}
							default: {
								mavlink_queue_broadcast_error("[SENSOR] Issue with accel cal, aborting");
								sensors_cal_init();
								cal_mode_error = true;
								status_buzzer_failure();
								break;
							}
						}

						status_buzzer_success();

						//Reset for the next calibration
						_sensor_cal_data.accel.data.count = 0;
						_sensor_cal_data.accel.data.x_sum = 0;
						_sensor_cal_data.accel.data.y_sum = 0;
						_sensor_cal_data.accel.data.z_sum = 0;
						_sensor_cal_data.accel.data.t_sum = 0;

						//Make sure we wait for confirmation before continuing
						if(_sensor_cal_data.accel.accel_cal_step != SENSOR_CAL_ACCEL_DONE)
							_sensor_cal_data.accel.waiting = true;
					}
				}
			}

			break;
		}
		case SENSOR_CAL_INTER: {
			//TODO
			_sensor_calibration ^= SENSOR_CAL_INTER;

			break;
		}
		default: {
			sensors_cal_init();
			cal_mode_error = true;
			mavlink_queue_broadcast_error("[SENSOR] Invalid calibration mode, clearing");

			break;
		}
	}

	//If there are no longer any sensors to calibrate
	if( !cal_mode_error && (_sensor_calibration == SENSOR_CAL_NONE ) ) {
		mavlink_message_t msg;
		mavlink_prepare_command_ack(&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED);
		lpq_queue_broadcast_msg(&msg);
	}

	return !_sensor_calibration;
}

bool sensors_update(uint32_t time_us) {
	//bool update_success = false;
	//TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU
    //Convert temperature SI units (degC, m/s^2, rad/s)

	//Temperature in degC
	// value = (_sensors.imu.temp_raw/temp_scale) + temp_shift
	_sensors.imu.temperature = fix16_add(fix16_div(fix16_from_int(_sensors.imu.temp_raw), _sensor_cal_data.accel.temp_scale), _sensor_cal_data.accel.temp_shift);

	//Accel
	//TODO: value = (raw - BIAS - (EMP_COMP * TEMP)) * scale
	// value = (raw - BIAS) * scale

	//Correct for measurement biases
	fix16_t accel_x_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.x - get_param_int(PARAM_ACC_X_BIAS)), _sensors.imu.accel_scale);
	fix16_t accel_y_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.y - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
	fix16_t accel_z_tmp = fix16_mul(fix16_from_int(_sensors.imu.accel_raw.z - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);

	//Scale the accelerometer to match 1G
	_sensors.imu.accel.x = fix16_mul(accel_x_tmp, ( accel_x_tmp > 0 ) ? get_param_fix16(PARAM_ACC_X_SCALE_POS) : get_param_fix16(PARAM_ACC_X_SCALE_NEG) );
	_sensors.imu.accel.y = fix16_mul(accel_y_tmp, ( accel_y_tmp > 0 ) ? get_param_fix16(PARAM_ACC_Y_SCALE_POS) : get_param_fix16(PARAM_ACC_Y_SCALE_NEG) );
	_sensors.imu.accel.z = fix16_mul(accel_z_tmp, ( accel_z_tmp > 0 ) ? get_param_fix16(PARAM_ACC_Z_SCALE_POS) : get_param_fix16(PARAM_ACC_Z_SCALE_NEG) );

	//Gyro
	// value = (raw - BIAS) * scale
	_sensors.imu.gyro.x = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.x - get_param_int(PARAM_GYRO_X_BIAS)), _sensors.imu.gyro_scale);
	_sensors.imu.gyro.y = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.y - get_param_int(PARAM_GYRO_Y_BIAS)), _sensors.imu.gyro_scale);
	_sensors.imu.gyro.z = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw.z - get_param_int(PARAM_GYRO_Z_BIAS)), _sensors.imu.gyro_scale);


	//Other IMU updates
	_sensors.imu.status.time_read = sensors_clock_imu_int_get();
	_sensors.imu.status.new_data = true;

	safety_update_sensor(&_system_status.sensors.imu);

	//==-- Safety Button
	bool safety_button_reading = false;

	safety_button_reading = digitalIn(_sensors.safety_button.gpio_p, _sensors.safety_button.pin);

	if(safety_button_reading != _sensors.safety_button.state_db )
		_sensors.safety_button.time_db_read = time_us;

	if( ( time_us - _sensors.safety_button.time_db_read ) > _sensors.safety_button.period_db_us ) {
		if(safety_button_reading != _sensors.safety_button.state) {	//The reading has changed
			_sensors.safety_button.state = safety_button_reading;

			_sensors.safety_button.status.time_read = time_us;
			_sensors.safety_button.status.new_data = true;
		}
	}

	_sensors.safety_button.state_db = safety_button_reading;

	//==-- Calibrations
	if( _system_status.state == MAV_STATE_CALIBRATING ) {	//If any calibration is in progress
		//Run the rest of the calibration logic
		if( sensors_calibrate() )	//If calibration finished
			if( safety_request_state( MAV_STATE_STANDBY ) )	//Return to standby
				sensors_cal_init();	//Reset calibration data
	}

	//TODO: This should be aware of failures
	return true;
}

