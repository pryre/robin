#pragma once

#include <stdbool.h>

bool drv_sensors_imu_init( fix16_t *scale_accel, fix16_t *scale_gyro );

bool drv_sensors_mag_init(void);

bool drv_sensors_baro_init(void);

bool drv_sensors_sonar_init(void);
