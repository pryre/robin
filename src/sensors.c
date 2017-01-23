#include <stdint.h>
#include <stdbool.h>

#include "sensors.h"
#include "params.h"
#include "mavlink_system.h"
#include "drivers/mpu.h"
#include "breezystm32.h"

#include "fix16.h"
#include "fixvector3d.h"

//==-- Local Variables
//static int32_t last_check_imu = 0;
/*static float accel_scale; // converts to units of m/s^2

static int16_t accel_data[3];
static int16_t gyro_data[3];
static int16_t temp_data;
*/
static uint8_t accel_status = 0;
static uint8_t gyro_status = 0;
static uint8_t temp_status = 0;
static volatile bool imu_new_reading_status = false;

sensor_readings_imu_t _sensor_imu;
sensor_readings_barometer_t _sensor_baro;
sensor_readings_sonar_t _sensor_sonar;


//==-- Functions
static void time_init(void) {
	_sensor_time.present = true;
	_sensor_time.dt = 0;
	_sensor_time.counter = 0;
	_sensor_time.start = 0;
	_sensor_time.end = 0;
	_sensor_time.max = 0;
	_sensor_time.min = 1000;
}
/*
static void imu_init(void) {
	_sensor_imu.present = true;
	_sensor_imu.temperature = 0;
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
		imu_new_reading_status = true;
		mpu6050_request_async_accel_read(_sensor_imu.accel_raw, &accel_status);
		mpu6050_request_async_gyro_read(_sensor_imu.gyro_raw, &gyro_status);
		mpu6050_request_async_temp_read(&(_sensor_imu.temp_raw), &temp_status);
}

void init_sensors(void) {
	uint16_t acc1G;

	//==-- IMU-MPU6050
	//TODO: Set IMU to be calibrated if not already
    mpu6050_register_interrupt_cb(&sensors_imu_poll, BOARD_REV);
	acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
	_sensor_imu.accel_scale = fix16_sdiv(fix16_from_float(9.80665f), fix16_from_int(acc1G));
	_sensor_imu.accel_scale = fix16_from_float(MPU_GYRO_SCALE);

	//==-- Timer
	time_init();
}

bool sensors_read(void) {
	bool imu_job_complete = false;

	//Check IMU status
	if(imu_new_reading_status && accel_status == I2C_JOB_COMPLETE && gyro_status == I2C_JOB_COMPLETE && temp_status == I2C_JOB_COMPLETE) {
		imu_new_reading_status = false; //TODO: Maybe not actually needed as I2C_JOB_COMPLETE might be enough
		imu_job_complete = true;
		accel_status = I2C_JOB_DEFAULT;
		gyro_status = I2C_JOB_DEFAULT;
		temp_status = I2C_JOB_DEFAULT;

		//TODO: Maybe the imu data should be aggregated so it can be called at a lower Hz?
	}

	//TODO: Check other status

	//TODO: May need to offset these so they don't all check at once

	//Return the results
	return imu_job_complete;
}

bool sensors_update(uint32_t time_us) {
	//bool update_success = false;
	//TODO: Remember not to expect all sensors to be ready

	//==-- Update IMU

    // convert temperature SI units (degC, m/s^2, rad/s)
    // convert to NED (first of braces)
	//Correct for biases and temperature (second braces)
	//TODO: Fix get_param_float to be of the right type so the extra conversion here is not needed
	//TODO: Calibration has to be done without biases and temp factored in
	_sensor_imu.time = time_us;

	_sensor_imu.temperature = fix16_sadd(fix16_sdiv(fix16_from_int(_sensor_imu.temp_raw), fix16_from_float(340.0f)), fix16_from_float(36.53f));

    _sensor_imu.accel.x = fix16_ssub(fix16_smul(fix16_from_int(_sensor_imu.accel_raw[0]), _sensor_imu.accel_scale), fix16_sadd(fix16_smul(fix16_from_float(get_param_float(PARAM_ACC_X_TEMP_COMP)), _sensor_imu.temperature), fix16_from_float(get_param_float(PARAM_ACC_X_BIAS))));
    _sensor_imu.accel.y = fix16_ssub(fix16_smul(fix16_from_int(-_sensor_imu.accel_raw[1]), _sensor_imu.accel_scale), fix16_sadd(fix16_smul(fix16_from_float(get_param_float(PARAM_ACC_Y_TEMP_COMP)), _sensor_imu.temperature), fix16_from_float(get_param_float(PARAM_ACC_Y_BIAS))));
    _sensor_imu.accel.z = fix16_ssub(fix16_smul(fix16_from_int(-_sensor_imu.accel_raw[2]), _sensor_imu.accel_scale), fix16_sadd(fix16_smul(fix16_from_float(get_param_float(PARAM_ACC_Z_TEMP_COMP)), _sensor_imu.temperature), fix16_from_float(get_param_float(PARAM_ACC_Z_BIAS))));

	_sensor_imu.gyro.x = fix16_ssub(fix16_smul(fix16_from_int(_sensor_imu.gyro_raw[0]), _sensor_imu.gyro_scale), fix16_from_float(get_param_float(PARAM_GYRO_X_BIAS)));
	_sensor_imu.gyro.y = fix16_ssub(fix16_smul(fix16_from_int(-_sensor_imu.gyro_raw[1]), _sensor_imu.gyro_scale), fix16_from_float(get_param_float(PARAM_GYRO_Y_BIAS)));
	_sensor_imu.gyro.z = fix16_ssub(fix16_smul(fix16_from_int(-_sensor_imu.gyro_raw[2]), _sensor_imu.gyro_scale), fix16_from_float(get_param_float(PARAM_GYRO_Z_BIAS)));

	//Send mavlink HIGHRES_IMU message
	//[timestamp, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag, abs_pressure, diff_pressure, pressure_alt, temperature, updated_bitmask]
	mavlink_msg_highres_imu_send(MAVLINK_COMM_0,
		_sensor_imu.time,
		fix16_to_float(_sensor_imu.accel.x),
		fix16_to_float(_sensor_imu.accel.y),
		fix16_to_float(_sensor_imu.accel.z),
		fix16_to_float(_sensor_imu.gyro.x),
		fix16_to_float(_sensor_imu.gyro.y),
		fix16_to_float(_sensor_imu.gyro.z),
		0, 0, 0,
		0, 0, 0,
		0,
		((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)) );

	/*
	//if( time_us - last_check_imu > 100) {

		if (accel_status == I2C_JOB_COMPLETE
			&& gyro_status == I2C_JOB_COMPLETE
			&& temp_status == I2C_JOB_COMPLETE) {

			mpu_measurement_in_progress = false;

			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,
										time_us,
										(uint16_t)(accel_data[0]*accel_scale*1000.0f),
										(uint16_t)(accel_data[1]*accel_scale*1000.0f),
										(uint16_t)(accel_data[2]*accel_scale*1000.0f),
										(uint16_t)(gyro_data[0]*MPU_GYRO_SCALE*1000.0f),
										(uint16_t)(gyro_data[1]*MPU_GYRO_SCALE*1000.0f),
										(uint16_t)(gyro_data[2]*MPU_GYRO_SCALE*1000.0f),
										0, 0, 0);

			//			last_check_imu = time_us;
		}
	//}
	*/

	//TODO: This should be aware of failures
	return true;
}

