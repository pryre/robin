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

extern void SetSysClock(bool overclock);
uint8_t _system_operation_control;

system_status_t _system_status;
command_input_t _command_input;

int main(void) {
	_system_status.state = MAV_STATE_UNINIT;
	_system_status.mode = MAV_MODE_PREFLIGHT;

    SetSysClock(false);

    systemInit();

	safety_request_state( MAV_STATE_BOOT );

    setup();

	safety_request_state( MAV_STATE_STANDBY );

    while (true)
        loop();
}

void setup(void) {
	params_init();

	communications_system_init();

	delay(500);	//Wait for i2c devices to boot properly

    i2cInit(I2CDEV);

	sensors_init();

	estimator_init();

	safety_init();

	controller_init();

	mixer_init();

	pwm_init();

	//Wait here for the first imu message (probably not really neaded)
	while( !sensors_read() );
}


//XXX: Measured CPU load when armed and running: 51.4%
void loop(void) {
	//TODO: XXX: There will be a timer rollover at ~70 minutes of operation, will cause some issues

	//Take note of when this loop starts
	sensors_clock_ls_set( micros() );

	//Sensor Poll
	//Take a poll of any sensors (that aren't the IMU) that need to be updated
	//sensors_poll();	//TODO: Good time to check other sensors for more raw data
						//TODO: This should alert sensors_read() somhow to let it know there's more data to wait for

	//==-- Check Serial
	communication_receive();	//XXX: Could be moved to a UART callback, but may cause issues with async I2C

	//==-- Send Serial
	//Check to see if a message has been sent this loop, then see if a message should be sent
	//Only allow this once per loop due to buffer risks (see serial define above)
	communication_transmit( micros() );

	//==-- Update Sensor Data
	sensors_update( micros() );	//XXX: This takes ~230us with just IMU //TODO: Recalc

	//==-- Timeout Checks
	safety_run( micros() );

	//==-- Update Estimator
    estimator_update( sensors_clock_imu_int_get() ); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)

	//Only run the controller if the mav is armed
	if( ( safety_is_armed() ) && ( _system_status.state != MAV_STATE_EMERGENCY ) ) {
		//==-- Update Controller
		controller_run( sensors_clock_imu_int_get() );	//Apply the current commands and update the PID controllers
	} else {
		//==-- Reset Controller
		controller_reset();	//Reset the PIDs and output flat 0s for control
	}

	//==-- Send Motor Commands
	mixer_output();	//Convert outputs to correct layout and send PWM (and considers failsafes)

    //==-- loop time calculation
	sensors_clock_update( micros() );

	//==-- Read sensors for next loop
	// IMU interrupt can cappen halfway through the processing loop
	//  but the only things that can change are the raw imu/gyro values
	//  so it is safe to assume that we can let the async i2c read take place
	//  then continue when it is ready, as this still runs off the same IMU interrupt
	while( !sensors_read() );	//XXX: With no load, it takes ~557us to complete sensor_read()
}
