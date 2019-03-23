/*
   mpu.c : driver for Invensense MPU devices (currently just MPU6050)

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_mpu.c

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "drivers/opencm3_naze32_common/drv_i2c.h"
#include "drivers/opencm3_naze32_common/drv_mpu.h"

#include <math.h>

void mpu_read_accel( uint32_t i2c, int16_t* accData ) {
	uint8_t buf[6];

	drv_i2c_read_buffer( i2c, MPU_ADDRESS, MPU_RA_ACCEL_XOUT_H, buf, 6);

	accData[0] = ( int16_t )( ( buf[0] << 8 ) | buf[1] );
	accData[1] = ( int16_t )( ( buf[2] << 8 ) | buf[3] );
	accData[2] = ( int16_t )( ( buf[4] << 8 ) | buf[5] );
}

void mpu_read_gyro( uint32_t i2c, int16_t* gyroData ) {
	uint8_t buf[6];

	drv_i2c_read_buffer( i2c, MPU_ADDRESS, MPU_RA_GYRO_XOUT_H, buf, 6 );

	gyroData[0] = ( int16_t )( ( buf[0] << 8 ) | buf[1] );
	gyroData[1] = ( int16_t )( ( buf[2] << 8 ) | buf[3] );
	gyroData[2] = ( int16_t )( ( buf[4] << 8 ) | buf[5] );
}

void mpu_read_temperature( uint32_t i2c, int16_t* tempData ) {
	uint8_t buf[2];

	drv_i2c_read_buffer( i2c, MPU_ADDRESS, MPU_RA_TEMP_OUT_A, buf, 2 );

	*tempData = ( int16_t )( ( buf[0] << 8 ) | buf[1] ) / 4;
}

/*=======================================================
 * Asynchronous I2C Read Functions:
 * These methods use the asynchronous I2C
 * read capability on the naze32.
 */

// Allocate storage for register data collected from a read
static volatile uint8_t accel_buffer[6];
static volatile uint8_t gyro_buffer[6];
static volatile uint8_t temp_buffer[2];

// Pointers for where the formatted data will end up
static volatile int16_t* gyro_data;
static volatile int16_t* temp_data;
static volatile int16_t* accel_data;

// These functions are called when an I2C job is finished
static void accel_read_CB( void ) {
	accel_data[0] = ( int16_t )( ( accel_buffer[0] << 8 ) | accel_buffer[1] );
	accel_data[1] = ( int16_t )( ( accel_buffer[2] << 8 ) | accel_buffer[3] );
	accel_data[2] = ( int16_t )( ( accel_buffer[4] << 8 ) | accel_buffer[5] );
}

static void gyro_read_CB( void ) {
	gyro_data[0] = ( int16_t )( ( gyro_buffer[0] << 8 ) | gyro_buffer[1] );
	gyro_data[1] = ( int16_t )( ( gyro_buffer[2] << 8 ) | gyro_buffer[3] );
	gyro_data[2] = ( int16_t )( ( gyro_buffer[4] << 8 ) | gyro_buffer[5] );
}

static void temp_read_CB( void ) {
	( *temp_data ) = ( int16_t )( ( temp_buffer[0] << 8 ) | temp_buffer[1] ) / 4;
}

// Adds a new i2c job to the I2C job queue.
// Current status of the job can be read by polling the
// status variable, and the callback will be called when the function
// is finished
void mpu_request_async_accel_read( uint32_t i2c, volatile int16_t* accData, volatile uint8_t* status ) {
	accel_data = accData;
	drv_i2c_queue_job( i2c,
					   I2C_JOB_TYPE_READ,
					   MPU_ADDRESS,
					   MPU_RA_ACCEL_XOUT_H,
					   accel_buffer,
					   6,
					   status,
					   &accel_read_CB );
}

void mpu_request_async_gyro_read( uint32_t i2c, volatile int16_t* gyroData, volatile uint8_t* status ) {
	gyro_data = gyroData;
	drv_i2c_queue_job( i2c,
					   I2C_JOB_TYPE_READ,
					   MPU_ADDRESS,
					   MPU_RA_GYRO_XOUT_H,
					   gyro_buffer,
					   6,
					   status,
					   &gyro_read_CB );
}

void mpu_request_async_temp_read( uint32_t i2c, volatile int16_t* tempData, volatile uint8_t* status ) {
	temp_data = tempData;
	drv_i2c_queue_job( i2c,
					   I2C_JOB_TYPE_READ,
					   MPU_ADDRESS,
					   MPU_RA_TEMP_OUT_A,
					   temp_buffer,
					   2,
					   status,
					   &temp_read_CB );
}
