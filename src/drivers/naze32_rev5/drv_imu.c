#include "drv_sensors.h"
#include "drv_mpu.h"
#include "sensor.h"

#include "fix16.h"

bool drv_sensors_imu_init( fix16_t *scale_accel, fix16_t *scale_gyro ) {
	//Get the 1g gravity scale (raw->g's)
	mpu_register_interrupt_cb(&sensors_imu_poll, 5);	//Naze32 Rev.5
	uint16_t acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);

	*scale_accel = fix16_div(_fc_gravity, fix16_from_int(acc1G));	//Get the m/s scale (raw->g's->m/s/s)
	*scale_gyro = fix16_from_float(MPU_GYRO_SCALE);	//Get radians scale (raw->rad/s)

	return true;
}
