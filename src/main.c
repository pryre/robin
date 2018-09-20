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

	mixer_init();	//XXX: Must be called before pwm_init()
	pwm_init();		//XXX: This locks boot for a while if ESC cal is active, so do it before i2c devices

	delay(500);	//Wait for i2c devices to boot properly

    i2cInit(I2CDEV);

	sensors_init_internal();
	//XXX: Overrides settings made by pwm backend, so must be after pwm_init()
	sensors_init_external();

	estimator_init();

	control_init();
}

//XXX: Measured CPU load when armed and running: 51.4%
void loop(void) {
	//TODO: XXX: There will be a timer rollover at ~70 minutes of operation, will cause some issues

	//Take note of when this loop starts
	sensors_clock_ls_set( micros() );

	//Sensor Read
	//Check to see if any of the i2c sensors have been updated (mainly the imu)
	// and if so, update the sensor states and estimator
	//If we're not in HIL mode
	if(!_sensors.hil.status.present) {
		sensors_poll( micros() );

		if( sensors_read() ) {
			sensors_update( micros() );

			estimator_update_sensors( sensors_clock_imu_int_get() );
		}
	} else {
		if(_sensors.hil.status.new_data)
			estimator_update_hil( micros() );
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
	control_run( micros() );

	//==-- Send Motor Commands
	//Convert outputs to correct layout and send PWM (and considers failsafes)
	pwm_output();

    //==-- loop time calculation
	sensors_clock_update( micros() );
}
