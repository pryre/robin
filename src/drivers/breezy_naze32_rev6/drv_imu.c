#include "drivers/drv_sensors.h"
#include "drivers/drv_system.h"
#include "drivers/breezy_naze32_common/drv_mpu.h"
#include "sensors.h"

#include "fix16.h"
#include "fixextra.h"

uint32_t volatile _imu_time_ready;

static void drv_sensors_imu_poll( void ) {
	//==-- Timing setup get loop time
	_imu_time_ready = system_micros();
}

bool drv_sensors_imu_init( uint32_t i2c, fix16_t* scale_accel, fix16_t* scale_gyro ) {
	// Get the 1g gravity scale (raw->g's)
	mpu_register_interrupt_cb( &drv_sensors_imu_poll, 6 ); // Naze32 Rev.6
	uint16_t acc1G = mpu6500_init( INV_FSR_8G, INV_FSR_2000DPS );

	*scale_accel = fix16_div( _fc_gravity,
							  fix16_from_int( acc1G ) ); // Get the m/s scale (raw->g's->m/s/s)
	*scale_gyro = fix16_from_float( MPU_GYRO_SCALE );	// Get radians scale (raw->rad/s)

	return true;
}
