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
#include "mavlink_receive.h"

serialPort_t * Serial1;
extern void SetSysClock(bool overclock);
uint32_t DEBUG_PARAM_COUNTER = 0;

#ifndef EXTERNAL_DEBUG
void debug(const char * fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);

    char buf[1000];

    vsprintf(buf, fmt, ap);

    for (char * p = buf; *p; p++)
        serialWrite(Serial1, *p);

    va_end(ap);

    while (!isSerialTransmitBufferEmpty(Serial1));
}

void sendNextParam() {
	if( DEBUG_PARAM_COUNTER >= PARAMS_COUNT ) {
		DEBUG_PARAM_COUNTER = 0;
	}

	char str[80];
	debug( "\n\r" );
	debug(get_param_name(DEBUG_PARAM_COUNTER));
	debug(": ");

	if(get_param_type(DEBUG_PARAM_COUNTER) == PARAM_TYPE_FLOAT) {
		sprintf(str, "%f", get_param_float(DEBUG_PARAM_COUNTER));
	}
	else {
		sprintf(str, "%i", get_param_int(DEBUG_PARAM_COUNTER));
	}

	debug( str );
	debug( "\n\r" );

	DEBUG_PARAM_COUNTER++;
}
#endif

int main(void)
{
    SetSysClock(false);

    systemInit();

    setup();

    Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX, SERIAL_NOT_INVERTED);

    while (true) {

#ifndef EXTERNAL_DEBUG
        // support reboot from host computer
        while (serialTotalRxBytesWaiting(Serial1)) {
            uint8_t c = serialRead(Serial1);
            if (c == 'R')
                systemResetToBootloader();

			if (c == 'n')
				sendNextParam();
         }
#endif

        loop();
    }
}

void setup(void)
{
	delay(500);

    i2cInit(I2CDEV);
	/*
    delay(500);
    i2cInit(I2CDEV_2);
    mpu6050_register_interrupt_cb(&interruptCallback, BOARD_REV);

    uint16_t acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
    accel_scale = 9.80665f / acc1G;
	*/
	init_params();
}

void loop(void)
{
	//==-- Critical

	//Check Sensors

	//Check Serial
	communication_receive();

	//Timeout Checks

	//Update Estimator

	//Send Motor Commands

	//==-- Remaining Time

	//Send Serial

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
	delay(500);
	debug(".");
	LED0_TOGGLE;
}
