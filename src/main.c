#include <stdio.h>
#include <stdarg.h>

#include "breezystm32.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "estimator.h"
#include "controller.h"
#include "mixer.h"

#include "mavlink/mavlink_types.h"
#include "mavlink_system.h"
#include "mavlink_receive.h"
#include "mavlink_transmit.h"

serialPort_t * Serial1;
serialPort_t * Serial2;
extern void SetSysClock(bool overclock);
uint8_t _system_operation_control;
volatile bool imu_interrupt;

system_status_t _system_status;
command_input_t _command_input;

int main(void) {
	_system_status.state = MAV_STATE_UNINIT;

    SetSysClock(false);
	imu_interrupt = false;

    systemInit();

	_system_status.state = MAV_STATE_BOOT;
	_system_status.mode = MAV_MODE_PREFLIGHT;

    setup();

	//We really need to consider how the baud affects Comms
	//A baud rate of 921600 will get us a data rate of 115200 bytes/sec
	//This is assuming we don't use stop bits
	//This means we can send ~115 bytes per loop without any risk of a buffer overflow
	//As mavlink can have max messages of 250+ bytes, we want to send as little as possible each loop to allow catchup
    Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX, SERIAL_NOT_INVERTED);
	Serial2 = uartOpen(USART2, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX, SERIAL_NOT_INVERTED);	//TODO: Allow this to have a difference baud rate through params

	_system_status.state = MAV_STATE_STANDBY;
	_system_status.mode = MAV_MODE_STABILIZE_DISARMED;

    while (true) {
        loop();
    }
}

void setup(void) {
	init_params();

	delay(500);

    i2cInit(I2CDEV);

	sensors_init();

	communications_init();

	estimator_init(true, true, true);

	safety_init();

	controller_init();

	mixer_init();

	//TODO: Could do motor calibration logic here
	pwm_init();

	//Wait here for the first imu message (probably not really neaded)
	while(!imu_interrupt);
}

void loop(void) {
	bool message_transmitted = false;

	//==-- Timing setup get loop time
	sensors_clock_ls_set( micros() );

	//==-- Read Sensors

	//Sensor Poll
	//Take a poll of any sensors (that aren't the IMU) that need to be updated
	//sensors_poll();	//TODO: Good time to check other sensors for more raw data
						//TODO: This should alert sensors_read() somhow to let it know there's more data to wait for

	//Keep busy until the sensor data is ready
	//Non-critical functions should be here, but we should also
	//do this loop multiple times, so it shouldn't lock the
	//thread for a long time
	while( !sensors_read() ) { //XXX: With no load, it takes ~557us to complete sensor_read()

		//==-- Check Serial
		communication_receive();

		//==-- Send Serial
		//Check to see if a message has been sent this loop, then see if a message should be sent
		//Only allow this once per loop due to buffer risks (see serial define above)
		if( !message_transmitted )
			message_transmitted = communication_transmit( micros() );

		//==-- Parameter Handilng

		//==-- Debug Information
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, micros(), "loop_ping", 0.0f);
	}

	//==-- Update Sensor Data
	sensors_update( sensors_clock_ls_get() );	//XXX: This takes ~230us with just IMU //TODO: Should double check this figure

	//==-- Calibrations
	if( _system_status.state == MAV_STATE_CALIBRATING ) {	//If any calibration is in progress
		if( sensors_calibrate() )	//Run the rest of the calibration logic
			_system_status.state = MAV_STATE_STANDBY;	//TODO: Make sure this is in state logic
	}

	//==-- Timeout Checks
	safety_run( micros() );
	//TODO: Set MAV_STATE in this function
	//TODO: Read for safety button here
	//TODO: Check timeouts for:
		//Accellerometer
		//Gyroscope
		//Compass
		//External Attitude Input
		//Command Input
		//External System Heartbeats
		//Anything else?

	//==-- Update Estimator
    estimator_update( sensors_clock_ls_get() ); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)

	//TODO: Make check to see if armed, else skip
		if( _system_status.mavlink.offboard_control.health != SYSTEM_HEALTH_OK ) {
			_command_input.r = 0;
			_command_input.p = 0;
			_command_input.y = 0;
			_command_input.q.a = 1;
			_command_input.q.b = 0;
			_command_input.q.c = 0;
			_command_input.q.d = 0;
			_command_input.T = 0;	//TODO: Throttle failsafe if timeout?
			_command_input.input_mask |= CMD_IN_IGNORE_ATTITUDE;	//Set it to just hold rpy rates (as this skips unnessessary computing during boot, and is possibly safer)
		}

		//==-- Update Controller
		controller_run( sensors_clock_ls_get() );	//Apply the current commands and update the PID controllers
		//TODO: Need to reset PIDs when armed
		//TODO: If there are any critical timeouts, set output to 0,0,0,throttle_emergency_land
		//TODO: Make check to see if armed, else skip

	//==-- Send Motor Commands
	mixer_output();	//Convert outputs to correct layout and send PWM (and considers failsafes)

	//==-- Boot Control	//TODO: Might be a better way to organize this
	if( _system_operation_control != SYSTEM_OPERATION_RUN ) {
		if( _system_operation_control == SYSTEM_OPERATION_REBOOT_BOOTLOADER )
			systemResetToBootloader();
		//Could be potentially more options here but just leave this as an error fallback
		systemReset();
	}

    //==-- loop time calculation

	//TODO: Move this elsewhere?
	sensors_clock_update( micros() );

	//==-- Waste remaining time
	while( !imu_interrupt );
}
