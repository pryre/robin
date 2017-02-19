#include <stdio.h>
#include <stdarg.h>

#include "breezystm32.h"
#include "params.h"
#include "sensors.h"
#include "estimator.h"
#include "mavlink/mavlink_types.h"
#include "mavlink_receive.h"
#include "mavlink_transmit.h"

serialPort_t * Serial1;
extern void SetSysClock(bool overclock);
sensor_readings_t _sensors;
system_t _system_status;
uint8_t _system_operation_control;
volatile bool imu_interrupt;

int main(void)
{
	_system_status.state = MAV_STATE_UNINIT;

    SetSysClock(false);
	imu_interrupt = false;

    systemInit();

	_system_status.state = MAV_STATE_BOOT;

    setup();

	//We really need to consider how the baud affects Comms
	//A baud rate of 921600 will get us a data rate of 115200 bytes/sec
	//This is assuming we don't use stop bits
	//This means we can send ~115 bytes per loop without any risk of a buffer overflow
	//As mavlink can have max messages of 250+ bytes, we want to send as little as possible each loop to allow catchup
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

	sensors_init();

	communications_init();

	estimator_init(false, false, true);

	//Wait here for the first imu message (probably not really neaded)
	while(!imu_interrupt);
}

void loop(void)
{
	bool message_transmitted = false;

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
		//Check to see if a message has been sent this loop, then see if a message should be sent
		//Only allow this once per loop due to buffer risks (see serial define above)
		if(!message_transmitted)
			message_transmitted = communication_transmit(micros());

		//==-- Parameter Handilng

		//==-- Debug Information
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, micros(), "loop_ping", 0.0f);
	}

	//==-- Update Sensor Data
	sensors_update(micros());	//XXX: This takes ~230us with just IMU //TODO: Should double check this figure

	//==-- Calibrations
	if(_sensor_calibration != SENSOR_CAL_NONE)	//If any calibration is in progress
		sensors_calibrate();	//Run the rest of the calibration logic

	//==-- Timeout Checks
	//TODO: Set MAV_STATE in this function

	//==-- Update Estimator
    estimator_update(micros()); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)

	//==-- Update Controller
	controller_run();	//Apply the current commands and update the PID controllers

	//==-- Send Motor Commands
	mixer_output();	//Convert outputs to correct layout and send PWM

	//==-- Boot Control
	if(_system_operation_control != SYSTEM_OPERATION_RUN) {
		if(_system_operation_control == SYSTEM_OPERATION_REBOOT_BOOTLOADER)
			systemResetToBootloader();
		//Could be potentially more options here but just leave this as an error fallback
		systemReset();
	}

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
