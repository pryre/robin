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
control_timing_t _control_timing;

system_status_t _system_status;

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

	safety_init();

	//XXX:This locks boot for a while if ESC cal is active, so do it before i2c devices
	mixer_init();
	pwm_init();

	delay(500);	//Wait for i2c devices to boot properly

    i2cInit(I2CDEV);

	sensors_init_internal();
	//XXX: Overrides settings made by pwm backend, so must be after pwm_init()
	sensors_init_external();

	estimator_init();

	controller_init();

	//==============================================================

	//Wait here for the first imu message (probably not really neaded)
	//while( !sensors_read() );
}

//XXX: Measured CPU load when armed and running: 51.4%
void loop(void) {
	//TODO: XXX: There will be a timer rollover at ~70 minutes of operation, will cause some issues

	//Take note of when this loop starts
	sensors_clock_ls_set( micros() );

	//Sensor Read
	//Check to see if any of the i2c sensors have been updated (mainly the imu)
	// and if so, update the sensor states and estimator
	sensors_poll( micros() );

	if( sensors_read() ) {

		sensors_update( micros() );

		estimator_update( sensors_clock_imu_int_get() ); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
	}

	//==-- Check Serial
	communication_receive();	//XXX: Could be moved to a UART callback, but may cause issues with async I2C

	//==-- Transmit Serial
	//Check to see if a message has been sent this loop, then see if a message should be sent
	//Only allow this once per loop due to buffer risks (see serial define above)
	communication_transmit( micros() );

	//==-- Timeout Checks
	safety_run( micros() );

	//==-- Control Process
	//Run the control loop at a slower frequency so it is more resilient against noise
	if( (micros() - _control_timing.time_last) > _control_timing.period_update) {
		if( ( safety_is_armed() ) && ( _system_status.state != MAV_STATE_EMERGENCY ) ) {
			//==-- Update Controller
			controller_run( sensors_clock_imu_int_get() );	//Apply the current commands and update the PID controllers
		} else {
			//==-- Reset Controller
			controller_reset();	//Reset the PIDs and output flat 0s for control
		}

		//==-- Send Motor Commands
		mixer_output();	//Convert outputs to correct layout and send PWM (and considers failsafes)

		_control_timing.time_last = micros();
	}

    //==-- loop time calculation
	sensors_clock_update( micros() );
}
