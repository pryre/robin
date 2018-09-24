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


#include <breezystm32.h>
#include <drv_mpu.h>

#include <math.h>

// ======================================================================
uint8_t mpuLowPassFilter;

uint16_t mpu6500_init(accel_fsr_e accelFSR, gyro_fsr_e gyroFSR)
{
    // Default acc1G. Modified once by for old (hopefully nonexistent outside of clones) parts
    uint16_t acc1G = 4096;
	mpuLowPassFilter = INV_FILTER_42HZ;

    // determine product ID and accel revision
    uint8_t tmp[6];
    mpuReadRegisterI2C(MPU_RA_XA_OFFS_H, tmp, 6);
    uint8_t rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);

	//No difference in accel revisions
    if (!rev)
        failureMode(5);

    // Device reset
    mpuWriteRegisterI2C(MPU_RA_PWR_MGMT_1, 0x80); // Device reset
    delay(100);

    // Gyro config
    mpuWriteRegisterI2C(MPU_RA_SMPLRT_DIV, 0x00); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    mpuWriteRegisterI2C(MPU_RA_PWR_MGMT_1, MPU6050_INV_CLK_GYROZ); // Clock source = 3 (PLL with Z Gyro reference)
    delay(10);
    mpuWriteRegisterI2C(MPU_RA_CONFIG, mpuLowPassFilter); // set DLPF
    mpuWriteRegisterI2C(MPU_RA_GYRO_CONFIG, gyroFSR /*INV_FSR_2000DPS*/ << 3); 

    // Accel config
    mpuWriteRegisterI2C(MPU_RA_ACCEL_CONFIG, accelFSR << 3);

    // Data ready interrupt configuration:  INT_RD_CLEAR_DIS, I2C_BYPASS_EN
    mpuWriteRegisterI2C(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    mpuWriteRegisterI2C(MPU_RA_INT_ENABLE, 0x01); // DATA_RDY_EN interrupt enable

    // Return acceleration constant
    return acc1G;
}
