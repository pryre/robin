#include <stdint.h>
#include <stdbool.h>

#include "sensors.h"
#include "mavlink_system.h"
#include "drivers/mpu.h"
#include "breezystm32.h"

//==-- Local Variables
//static int32_t last_check_imu = 0;
static float accel_scale; // converts to units of m/s^2

static int16_t accel_data[3];
static int16_t gyro_data[3];
static int16_t temp_data;

static uint8_t accel_status = 0;
static uint8_t gyro_status = 0;
static uint8_t temp_status = 0;
static bool mpu_measurement_in_progress = false;

sensor_readings_time_t sensor_time;


//==-- Functions
static void time_init(void) {
	sensor_time.dt = 0;
	sensor_time.counter = 0;
	sensor_time.start = 0;
	sensor_time.end = 0;
	sensor_time.max = 0;
	sensor_time.min = 1000;
}

void init_sensors(void) {
	//==-- IMU-MPU6050
	uint16_t acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
	accel_scale = 9.80665f / acc1G;

	//==-- Timer
	time_init();
}

bool update_sensors(uint32_t time_us) {
	bool update_success = false;

    if(!mpu_measurement_in_progress) {
		mpu_measurement_in_progress = true;

		mpu6050_request_async_accel_read(accel_data, &accel_status);
		mpu6050_request_async_gyro_read(gyro_data, &gyro_status);
		mpu6050_request_async_temp_read(&temp_data, &temp_status);
	}
//	if( time_us - last_check_imu > 100) {

		if (accel_status == I2C_JOB_COMPLETE
			&& gyro_status == I2C_JOB_COMPLETE
			&& temp_status == I2C_JOB_COMPLETE) {

			mpu_measurement_in_progress = false;
			/*
			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,
										time_us,
										(uint16_t)(accel_data[0]*accel_scale*1000.0f),
										(uint16_t)(accel_data[1]*accel_scale*1000.0f),
										(uint16_t)(accel_data[2]*accel_scale*1000.0f),
										(uint16_t)(gyro_data[0]*MPU_GYRO_SCALE*1000.0f),
										(uint16_t)(gyro_data[1]*MPU_GYRO_SCALE*1000.0f),
										(uint16_t)(gyro_data[2]*MPU_GYRO_SCALE*1000.0f),
										0, 0, 0);
			*/
//			last_check_imu = time_us;
		}
//	}

	return update_success;
}

