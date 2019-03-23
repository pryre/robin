/*
   mpu6050.h : driver for Invensense MPU6050

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_mpu.h

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

#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Generic driver for invensense gyro/acc devices.
 *
 * Supported hardware:
 * MPU6050 (gyro + acc)
 *
 * AUX_I2C is enabled on devices which have bypass, to allow forwarding to
 * compass in MPU9150-style devices
 */

// This is generally where all Invensense devices are at, for default (AD0 down)
// I2C address
#define MPU_ADDRESS ( 0x68 )

#define GYRO_INT_PIN ( Pin_13 )

#define MPU_RA_WHO_AM_I ( 0x75 )
#define MPU_RA_GYRO_XOUT_H ( 0x43 )
#define MPU_RA_ACCEL_XOUT_H ( 0x3B )
#define MPU_RA_TEMP_OUT_A ( 0x41 )
// For debugging/identification purposes
#define MPU_RA_XA_OFFS_H ( 0x06 )  //[15:0] XA_OFFS
#define MPU_RA_PRODUCT_ID ( 0x0C ) // Product ID Register

// WHO_AM_I register contents for 6050
#define MPUx0x0_WHO_AM_I_CONST ( 0x68 )

// 16.4 dps/lsb scalefactor for all Invensense devices
#define MPU_GYRO_SCALE ( ( 1.0f / 16.4f ) * ( (float)M_PI / 180.0f ) )

// MPU6xxx registers
#define MPU_RA_SMPLRT_DIV 0x19
#define MPU_RA_CONFIG 0x1A
#define MPU_RA_GYRO_CONFIG 0x1B
#define MPU_RA_ACCEL_CONFIG 0x1C
#define MPU_RA_FIFO_EN 0x23
#define MPU_RA_I2C_MST_CTRL 0x24
#define MPU_RA_INT_PIN_CFG 0x37
#define MPU_RA_INT_ENABLE 0x38
#define MPU_RA_SIGNAL_PATH_RST 0x68
#define MPU_RA_USER_CTRL 0x6A
#define MPU_RA_PWR_MGMT_1 0x6B
#define MPU_RA_PWR_MGMT_2 0x6C
#define MPU_RA_FIFO_COUNT_H 0x72
#define MPU_RA_FIFO_R_W 0x74

// MPU6xxx bits
#define MPU_BIT_DEVICE_RESET 0x80

// MPU6050 bits
#define MPU6050_INV_CLK_GYROZ 0x03
#define MPU6050_BIT_FIFO_RST 0x04
#define MPU6050_BIT_DMP_RST 0x08
#define MPU6050_BIT_FIFO_EN 0x40


typedef enum {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
} drv_mpu_lpf_e;

typedef enum {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
} drv_mpu_clock_sel_e;

// Gyro scale constants
typedef enum {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
} drv_mpu_gyro_fsr_e;

// Accelerometer constants
typedef enum {
	INV_FSR_2G = 0,
	INV_FSR_4G,
	INV_FSR_8G,
	INV_FSR_16G,
	NUM_ACCEL_FSR
} drv_mpu_accel_fsr_e;


bool mpu6050_init( uint32_t i2c, drv_mpu_accel_fsr_e accelFSR, drv_mpu_gyro_fsr_e gyroFSR, uint16_t *acc1G );
bool mpu6500_init( uint32_t i2c, drv_mpu_accel_fsr_e accelFSR, drv_mpu_gyro_fsr_e gyroFSR, uint16_t *acc1G );

// Blocking Read Functions
void mpu_read_accel( uint32_t i2c, int16_t* accData );
void mpu_read_gyro( uint32_t i2c, int16_t* gyroData );
void mpu_read_temperature( uint32_t i2c, int16_t* tempData );

// Asynchronous Read Functions
void mpu_request_async_accel_read( uint32_t i2c, volatile int16_t* accData, volatile uint8_t* status_ );
void mpu_request_async_gyro_read( uint32_t i2c, volatile int16_t* gyroData, volatile uint8_t* status_ );
void mpu_request_async_temp_read( uint32_t i2c, volatile int16_t* tempData, volatile uint8_t* status_ );
