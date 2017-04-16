#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sensors.h"
#include "params.h"
#include "mavlink_system.h"
#include "drivers/mpu.h"
#include "breezystm32.h"

#include "fix16.h"
#include "fixvector3d.h"

//Number of itterations of averaging to use with IMU calibrations
#define SENSOR_CAL_IMU_PASSES 1000

//==-- Local Variables

//static int32_t last_check_imu = 0;
/*static float accel_scale; // converts to units of m/s^2

static int16_t accel_data[3];
static int16_t gyro_data[3];
static int16_t temp_data;
*/
static volatile uint32_t _imu_time_read = 0;
static volatile uint8_t accel_status = 0;
static volatile uint8_t gyro_status = 0;
static volatile uint8_t temp_status = 0;

sensor_readings_t _sensors;
uint8_t _sensor_calibration;
sensor_calibration_data_t _sensor_cal_data;


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

/*
static void imu_init(void) {
	_sensors.imu.present = true;
	_sensors.imu.temperature = 0;
	uint32_t time;		//Time measured
	float accel_scale;	//Scale to correct raw accel data
	float gyro_scale;	//Scale to correct raw gyro data
}

static void baro_init(void) {
	_sensor_baro.present = false;
}

static void sonar_init(void) {
	_sensor_sonar.present = false;
}*/

static void sensors_imu_poll(void) {
		//==-- Timing setup get loop time
		_imu_time_read = micros();

		mpu6050_request_async_accel_read(_sensors.imu.accel_raw, &accel_status);
		mpu6050_request_async_gyro_read(_sensors.imu.gyro_raw, &gyro_status);
		mpu6050_request_async_temp_read(&(_sensors.imu.temp_raw), &temp_status);
}

void sensors_init(void) {
	//==-- IMU-MPU6050
	//TODO: Set IMU to be calibrated if not already
    mpu6050_register_interrupt_cb(&sensors_imu_poll, get_param_int(PARAM_BOARD_REVISION));
	_sensor_cal_data.accel.acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);	//Get the 1g gravity scale (raw->g's)

	_sensors.imu.accel_scale = fix16_div(CONST_GRAVITY, fix16_from_int(_sensor_cal_data.accel.acc1G));	//Get the m/s scale (raw->g's->m/s/s)
	_sensors.imu.gyro_scale = fix16_from_float(MPU_GYRO_SCALE);	//Get radians scale (raw->rad/s)

	_sensor_cal_data.accel.temp_scale = fix16_from_float(340.0f);
	_sensor_cal_data.accel.temp_shift = fix16_from_float(36.53f);

	_sensor_cal_data.gyro.count = 0;
	_sensor_cal_data.gyro.sum_x = 0;
	_sensor_cal_data.gyro.sum_y = 0;
	_sensor_cal_data.gyro.sum_z = 0;

	_sensor_cal_data.accel.count = 0;
	_sensor_cal_data.accel.sum_x = 0;
	_sensor_cal_data.accel.sum_y = 0;
	_sensor_cal_data.accel.sum_z = 0;
	_sensor_cal_data.accel.sum_t = 0;

	//==-- Timer
	clock_init();
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
		//TODO: Maybe the imu data should be aggregated so it can be called at a lower Hz?
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

bool sensors_update(uint32_t time_us) {
	//bool update_success = false;
	//TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU

    // convert temperature SI units (degC, m/s^2, rad/s)
    // convert to NED (first of braces)
	//Correct for biases and temperature (second braces)
	//TODO: Calibration has to be done without biases and temp factored in

	//TODO: Perhaps X is being calculated in revearse?
	//TODO:Something about the frame of reference (NED/ENU) isn't quite right

	//==-- Temperature in degC
	//value = (_sensors.imu.temp_raw/temp_scale) + temp_shift
	_sensors.imu.temperature = fix16_add(fix16_div(fix16_from_int(_sensors.imu.temp_raw), _sensor_cal_data.accel.temp_scale), _sensor_cal_data.accel.temp_shift);

	//==-- Accel in NED
	//TODO: value = (raw - BIAS - (EMP_COMP * TEMP)) * scale
	// value = (raw - BIAS) * scale
	_sensors.imu.accel.x = fix16_mul(fix16_from_int(-(_sensors.imu.accel_raw[0] - get_param_int(PARAM_ACC_X_BIAS))), _sensors.imu.accel_scale);
	_sensors.imu.accel.y = fix16_mul(fix16_from_int(_sensors.imu.accel_raw[1] - get_param_int(PARAM_ACC_Y_BIAS)), _sensors.imu.accel_scale);
	_sensors.imu.accel.z = fix16_mul(fix16_from_int(_sensors.imu.accel_raw[2] - get_param_int(PARAM_ACC_Z_BIAS)), _sensors.imu.accel_scale);

	//==-- Gyro in NED
	// value = (raw - BIAS) * scale
	_sensors.imu.gyro.x = fix16_mul(fix16_from_int(_sensors.imu.gyro_raw[0] - get_param_int(PARAM_GYRO_X_BIAS)), _sensors.imu.gyro_scale);
	_sensors.imu.gyro.y = fix16_mul(fix16_from_int(-(_sensors.imu.gyro_raw[1] - get_param_int(PARAM_GYRO_Y_BIAS))), _sensors.imu.gyro_scale);
	_sensors.imu.gyro.z = fix16_mul(fix16_from_int(-(_sensors.imu.gyro_raw[2] - get_param_int(PARAM_GYRO_Z_BIAS))), _sensors.imu.gyro_scale);

	_sensors.imu.status.time_read = time_us;
	_sensors.imu.status.new_data = true;

	//TODO: This should be aware of failures
	return true;
}

//TODO: This does not take into account temperature
//TODO: These parameters are not being written to EEPROM (Check to see if mavlink has a "save params" command)
//TODO: This calibration method is very basic, doesn't take into acount very much...mabye?
//Returns true if all calibrations are complete
bool sensors_calibrate(void) {
	if(_sensor_calibration & SENSOR_CAL_GYRO) {
		_sensor_cal_data.gyro.sum_x += _sensors.imu.gyro_raw[0];
		_sensor_cal_data.gyro.sum_y += _sensors.imu.gyro_raw[1];
		_sensor_cal_data.gyro.sum_z += _sensors.imu.gyro_raw[2];

		_sensor_cal_data.gyro.count++;

		if (_sensor_cal_data.gyro.count >= SENSOR_CAL_IMU_PASSES) {
			set_param_int(PARAM_GYRO_X_BIAS, (_sensor_cal_data.gyro.sum_x / _sensor_cal_data.gyro.count));
			set_param_int(PARAM_GYRO_Y_BIAS, (_sensor_cal_data.gyro.sum_y / _sensor_cal_data.gyro.count));
			set_param_int(PARAM_GYRO_Z_BIAS, (_sensor_cal_data.gyro.sum_z / _sensor_cal_data.gyro.count));

			_sensor_cal_data.gyro.count = 0;
			_sensor_cal_data.gyro.sum_x = 0;
			_sensor_cal_data.gyro.sum_y = 0;
			_sensor_cal_data.gyro.sum_z = 0;

			_sensor_calibration ^= SENSOR_CAL_GYRO;	//Turn off SENSOR_CAL_GYRO bit
			//TODO: "we could do some sanity checking here if we wanted to."
		}
	}

	if(_sensor_calibration & SENSOR_CAL_ACCEL) {
		//Compensate for the gravity in Z axis, that way bias can be relative to 0
		_sensor_cal_data.accel.sum_x += _sensors.imu.accel_raw[0];
		_sensor_cal_data.accel.sum_y += _sensors.imu.accel_raw[1];
		_sensor_cal_data.accel.sum_z += _sensors.imu.accel_raw[2] - _sensor_cal_data.accel.acc1G;
		_sensor_cal_data.accel.sum_t += _sensors.imu.temp_raw;

		_sensor_cal_data.accel.count++;

		if (_sensor_cal_data.accel.count >= SENSOR_CAL_IMU_PASSES) {
			//==-- bias = sum / count
			//==-- //TODO: bias = (sum - (temp_comp*temp_sum)) / count

			set_param_int(PARAM_ACC_X_BIAS, (_sensor_cal_data.accel.sum_x / _sensor_cal_data.accel.count));
			set_param_int(PARAM_ACC_Y_BIAS, (_sensor_cal_data.accel.sum_y / _sensor_cal_data.accel.count));
			set_param_int(PARAM_ACC_Z_BIAS, (_sensor_cal_data.accel.sum_z / _sensor_cal_data.accel.count));

			_sensor_cal_data.accel.count = 0;
			_sensor_cal_data.accel.sum_x = 0;
			_sensor_cal_data.accel.sum_y = 0;
			_sensor_cal_data.accel.sum_z = 0;
			_sensor_cal_data.accel.sum_t = 0;

			_sensor_calibration ^= SENSOR_CAL_ACCEL;	//Turn off SENSOR_CAL_ACCEL bit
			//TODO: "we could do some sanity checking here if we wanted to."
		}
	}

	//If there are no longer any sensors to calibrate
	if(_sensor_calibration == SENSOR_CAL_NONE) {
		mavlink_message_t msg;
		mavlink_prepare_command_ack(&msg, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED);
		lpq_queue_msg(MAVLINK_COMM_0, &msg);	//TODO: Select correct port
	}

	return !_sensor_calibration;
}

