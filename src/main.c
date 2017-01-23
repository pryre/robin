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
sensor_readings_time_t _sensor_time;
volatile bool imu_interrupt;

int main(void)
{
    SetSysClock(false);
	imu_interrupt = false;

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

	//Wait here for the first imu message (probably not really neaded)
	while(!imu_interrupt);
}

void loop(void)
{
	//==-- Timing setup get loop time
	_sensor_time.start = micros();

	//==-- Critical

	//Async Sensor Poll
	//sensors_poll(); //TODO: This is done via the interrupt

	//Keep busy until the sensor data is ready
	//Non-critical functions should be here, but we should also
	//do this loop multiple times, so it shouldn't lock the
	//thread for a long time
	while(!sensors_read()) { //XXX: With no load, it takes ~557us to complete sensor_read()
		//TODO: More things should be here if possible
		//TODO: Metrics!

		//Check Serial
		communication_receive();

		//Send Serial
		//This might be ideal to do after all processing is done,
		//but that puts it above everything else from the previous loop
		//so no loss, especially if we have to wait anyway
		communication_transmit(micros());

		//==-- Remaining Time

		//Parameter Handilng

		//Debug Information
	}

	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, micros(), "loop_ping", 0.0f);


	//Update Sensor Data
	sensors_update(micros());	//XXX: This takes ~230us with just IMU

	//Timeout Checks

	//Update Estimator

	//Send Motor Commands

    // loop time calculation

	//TODO: Move this elsewhere
    _sensor_time.end = micros();
    _sensor_time.dt = _sensor_time.end - _sensor_time.start;
    _sensor_time.average_time += _sensor_time.dt;
    _sensor_time.counter++;
    _sensor_time.max = (_sensor_time.dt > _sensor_time.max) ? _sensor_time.dt : _sensor_time.max;
    _sensor_time.min = (_sensor_time.dt < _sensor_time.min) ? _sensor_time.dt : _sensor_time.min;

	while(!imu_interrupt);
}
