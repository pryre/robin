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

static const fix16_t GRAVITY_CONST = 0x0009CE80; //Is equal to 9.80665 (Positive!) in Q16.16
//static int32_t last_check_imu = 0;
/*static float accel_scale; // converts to units of m/s^2

static int16_t accel_data[3];
static int16_t gyro_data[3];
static int16_t temp_data;
*/
static uint8_t accel_status = 0;
static uint8_t gyro_status = 0;
static uint8_t temp_status = 0;
extern volatile bool imu_interrupt;

sensor_readings_t _sensors;


//==-- Functions
static void time_init(void) {
	_sensors.time.present = true;
	_sensors.time.dt = 0;
	_sensors.time.counter = 0;
	_sensors.time.start = 0;
	_sensors.time.end = 0;
	_sensors.time.max = 0;
	_sensors.time.min = 1000;
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
		imu_interrupt = true;
		mpu6050_request_async_accel_read(_sensors.imu.accel_raw, &accel_status);
		mpu6050_request_async_gyro_read(_sensors.imu.gyro_raw, &gyro_status);
		mpu6050_request_async_temp_read(&(_sensors.imu.temp_raw), &temp_status);
}

void init_sensors(void) {
	uint16_t acc1G;

	//==-- IMU-MPU6050
	//TODO: Set IMU to be calibrated if not already
    mpu6050_register_interrupt_cb(&sensors_imu_poll, get_param_int(PARAM_BAUD_RATE));
	acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
	_sensors.imu.accel_scale = fix16_sdiv(GRAVITY_CONST, fix16_from_int(acc1G));
	_sensors.imu.accel_scale = fix16_from_float(MPU_GYRO_SCALE);

	//==-- Timer
	time_init();
}

bool sensors_read(void) {
	bool imu_job_complete = false;

	//Check IMU status
	if(accel_status == I2C_JOB_COMPLETE && gyro_status == I2C_JOB_COMPLETE && temp_status == I2C_JOB_COMPLETE) {
		imu_interrupt = false;	//TODO: There might be a better place to have this
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
	//TODO: Calibration has to be done without biases and temp factored in
	//TODO: Perhaps X is being calculated in revearse
	_sensors.imu.time = time_us;

	_sensors.imu.temperature = fix16_sadd(fix16_sdiv(fix16_from_int(_sensors.imu.temp_raw), fix16_from_float(340.0f)), fix16_from_float(36.53f));

    _sensors.imu.accel.x = fix16_ssub(fix16_smul(fix16_from_int(_sensors.imu.accel_raw[0]), _sensors.imu.accel_scale), fix16_sadd(fix16_smul(get_param_fix16(PARAM_ACC_X_TEMP_COMP), _sensors.imu.temperature), get_param_fix16(PARAM_ACC_X_BIAS)));
    _sensors.imu.accel.y = fix16_ssub(fix16_smul(fix16_from_int(-_sensors.imu.accel_raw[1]), _sensors.imu.accel_scale), fix16_sadd(fix16_smul(get_param_fix16(PARAM_ACC_Y_TEMP_COMP), _sensors.imu.temperature), get_param_fix16(PARAM_ACC_Y_BIAS)));
    _sensors.imu.accel.z = fix16_ssub(fix16_smul(fix16_from_int(-_sensors.imu.accel_raw[2]), _sensors.imu.accel_scale), fix16_sadd(fix16_smul(get_param_fix16(PARAM_ACC_Z_TEMP_COMP), _sensors.imu.temperature), get_param_fix16(PARAM_ACC_Z_BIAS)));

	_sensors.imu.gyro.x = fix16_ssub(fix16_smul(fix16_from_int(_sensors.imu.gyro_raw[0]), _sensors.imu.gyro_scale), get_param_fix16(PARAM_GYRO_X_BIAS));
	_sensors.imu.gyro.y = fix16_ssub(fix16_smul(fix16_from_int(-_sensors.imu.gyro_raw[1]), _sensors.imu.gyro_scale), get_param_fix16(PARAM_GYRO_Y_BIAS));
	_sensors.imu.gyro.z = fix16_ssub(fix16_smul(fix16_from_int(-_sensors.imu.gyro_raw[2]), _sensors.imu.gyro_scale), get_param_fix16(PARAM_GYRO_Z_BIAS));

	//Send mavlink HIGHRES_IMU message
	//[timestamp, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag, abs_pressure, diff_pressure, pressure_alt, temperature, updated_bitmask]

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

