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
#include "mavlink/mavlink_types.h"
#include "mavlink_receive.h"
#include "mavlink_transmit.h"

serialPort_t * Serial1;
extern void SetSysClock(bool overclock);
sensor_readings_t _sensors;
system_t _system_status;
volatile bool imu_interrupt;

int main(void)
{
	_system_status.state = MAV_STATE_UNINIT;

    SetSysClock(false);
	imu_interrupt = false;

    systemInit();

	_system_status.state = MAV_STATE_BOOT;

    setup();

    Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX, SERIAL_NOT_INVERTED);

	_system_status.state = MAV_STATE_STANDBY;

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
	_sensors.time.start = micros();

	//==-- Read Sensors

	//Sensor Poll
	//Take a poll of any sensors (that aren't the IMU) that need to be updated
	//sensors_poll();	//TODO: Good time to check other sensors for more raw data
						//TODO: This should alert sensors_read() somhow to let it know there's more data to wait for

	//Keep busy until the sensor data is ready
	//Non-critical functions should be here, but we should also
	//do this loop multiple times, so it shouldn't lock the
	//thread for a long time
	while(!sensors_read()) { //XXX: With no load, it takes ~557us to complete sensor_read()

		//==-- Check Serial
		communication_receive();

		//==-- Send Serial
		//This might be ideal to do after all processing is done,
		//but that puts it above everything else from the previous loop
		//so no loss, especially if we have to wait anyway
		communication_transmit(micros());

		//==-- Parameter Handilng

		//==-- Debug Information
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, micros(), "loop_ping", 0.0f);
	}



	//Update Sensor Data
	sensors_update(micros());	//XXX: This takes ~230us with just IMU

	//==-- Timeout Checks
	//TODO: Set MAV_STATE in this function

	//==-- Update Estimator

	//==-- Send Motor Commands

    //==-- loop time calculation

	//TODO: Move this elsewhere
    _sensors.time.end = micros();
    _sensors.time.dt = _sensors.time.end - _sensors.time.start;
    _sensors.time.average_time += _sensors.time.dt;
    _sensors.time.counter++;
    _sensors.time.max = (_sensors.time.dt > _sensors.time.max) ? _sensors.time.dt : _sensors.time.max;
    _sensors.time.min = (_sensors.time.dt < _sensors.time.min) ? _sensors.time.dt : _sensors.time.min;

	//==-- Waste remaining time
	while(!imu_interrupt);
}
