/*
   mpu.c : driver for Invensense MPU devices (currently just MPU6050)

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_mpu.c
 */

#include "drivers/opencm3_naze32_common/drv_mpu.h"
#include "drivers/opencm3_naze32_common/drv_i2c.h"
#include "drivers/drv_system.h"

#include <math.h>


bool mpu6500_init( uint32_t i2c, drv_mpu_accel_fsr_e accelFSR, drv_mpu_gyro_fsr_e gyroFSR, uint16_t *acc1G ) {
	bool success = true;
	// Default acc1G. Modified once by for old (hopefully nonexistent outside of
	// clones) parts
	(*acc1G) = 4096;

	// determine product ID and accel revision
	uint8_t tmp[6];
	drv_i2c_read_buffer(i2c, MPU_ADDRESS, MPU_RA_XA_OFFS_H, tmp, 6 );
	uint8_t rev = ( ( tmp[5] & 0x01 ) << 2 ) | ( ( tmp[3] & 0x01 ) << 1 ) | ( tmp[1] & 0x01 );

	// No difference in accel revisions
	if ( !rev )
		success = false;

	if(success) {
		// Device reset
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_PWR_MGMT_1, MPU_BIT_DEVICE_RESET ); // Device reset
		system_pause_ms( 100 );

		// Gyro config
		// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00 );
		// Clock source = 3 (PLL with Z Gyro reference)
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_PWR_MGMT_1, MPU6050_INV_CLK_GYROZ );

		system_pause_ms( 10 );

		// Set DLPF
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_CONFIG, INV_FILTER_42HZ );
		// gyroFSR := INV_FSR_2000DPS
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_GYRO_CONFIG, gyroFSR << 3 );

		// Accel config
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_ACCEL_CONFIG, accelFSR << 3 );

		// Data ready interrupt configuration:  INT_RD_CLEAR_DIS, I2C_BYPASS_EN
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0 );
		drv_i2c_write_register(i2c, MPU_ADDRESS, MPU_RA_INT_ENABLE, 0x01 ); // DATA_RDY_EN interrupt enable
	}

	return success;
}
