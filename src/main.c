/*
   main.c : entry routine for for STM32

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/main.c

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
#include <stdio.h>
#include <stdarg.h>

#include "breezystm32.h"
#include "params.h"
#include "sensors.h"
#include "mavlink_receive.h"
#include "mavlink_transmit.h"

serialPort_t * Serial1;
extern void SetSysClock(bool overclock);
sensor_readings_time_t sensor_time;

int main(void)
{
    SetSysClock(false);

    systemInit();

    setup();

    Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX, SERIAL_NOT_INVERTED);

    while (true) {
        loop();
    }
}

void setup(void)
{
	init_params();

	delay(500);

    i2cInit(I2CDEV);

	init_sensors();

	communications_init();
}

void loop(void)
{
	//==-- Timing setup get loop time
	sensor_time.start = micros();

	//==-- Critical

	//Check Sensors
	update_sensors(micros());
	//Check Serial
	communication_receive();

	//Timeout Checks

	//Update Estimator

	//Send Motor Commands

	//==-- Remaining Time

	//Send Serial
	communication_transmit(micros());

	//Parameter Handilng

	//Debug Information


	/*
    if (accel_status == I2C_JOB_COMPLETE
        && gyro_status == I2C_JOB_COMPLETE
        && temp_status == I2C_JOB_COMPLETE)
    {
        static int32_t count = 0;

        // Throttle printing
        if(count > 10000)
        {

            count = 0;
            debug("%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n",
                   (int32_t)(accel_data[0]*accel_scale*1000.0f),
                    (int32_t)(accel_data[1]*accel_scale*1000.0f),
                    (int32_t)(accel_data[2]*accel_scale*1000.0f),
                    (int32_t)(gyro_data[0]*MPU_GYRO_SCALE*1000.0f),
                    (int32_t)(gyro_data[1]*MPU_GYRO_SCALE*1000.0f),
                    (int32_t)(gyro_data[2]*MPU_GYRO_SCALE*1000.0f),
                    temp_data);
        }
        count++;

    }*/


    // loop time calculation
    sensor_time.end = micros();
    sensor_time.dt = sensor_time.end - sensor_time.start;
    sensor_time.average_time += sensor_time.dt;
    sensor_time.counter++;
    sensor_time.max = (sensor_time.dt > sensor_time.max) ? sensor_time.dt : sensor_time.max;
    sensor_time.min = (sensor_time.dt < sensor_time.min) ? sensor_time.dt : sensor_time.min;
}
