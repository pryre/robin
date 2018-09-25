#include <stdio.h>
//#include <stdarg.h>

#include "safety.h"
#include "params.h"
#include "calibration.h"
#include "sensors.h"
#include "estimator.h"
#include "controller.h"
#include "mixer.h"

#include "drivers/drv_system.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"

#include "mavlink/mavlink_types.h"
#include "mavlink_system.h"
#include "mavlink_receive.h"
#include "mavlink_transmit.h"

system_status_t _system_status;


void setup(void);
void loop(void);

int main(void) {
	_system_status.state = MAV_STATE_UNINIT;
	_system_status.mode = MAV_MODE_PREFLIGHT;

	system_init();
	safety_request_state( MAV_STATE_BOOT );

    setup();

	safety_request_state( MAV_STATE_STANDBY );

	while (true)
		loop();
}

void setup(void) {
	status_devices_init();

	params_init();

	communications_system_init();

	safety_init();

	mixer_init();	//XXX: Must be called before pwm_init()
	pwm_init();		//XXX: This locks boot for a while if ESC cal is active, so do it before i2c devices

	//XXX: Overrides settings made by pwm backend, so must be after pwm_init()
	sensors_init();

	calibration_init();

	estimator_init();

	control_init();
}

//XXX: Measured CPU load when armed and running: 51.4%
void loop(void) {
	//TODO: XXX: There will be a timer rollover at ~70 minutes of operation, will cause some issues

	//Take note of when this loop starts
	sensors_clock_ls_set( system_micros() );

	//Sensor Read
	//Check to see if any of the i2c sensors have been updated (mainly the imu)
	// and if so, update the sensor states and estimator
	//If we're not in HIL mode
	if(!_sensors.hil.status.present) {
		if( sensors_read( system_micros() ) ) {
			//Handle any calibration is in progress
			if( _system_status.state == MAV_STATE_CALIBRATING ) {
				//Run the rest of the calibration logic
				calibration_run();
			}

			estimator_update_sensors( sensors_clock_imu_int_get() );
		}
	} else {
		if(_sensors.hil.status.new_data)
			estimator_update_hil( system_micros() );
	}

	//==-- Check Serial
	communication_receive();	//XXX: Could be moved to a UART callback, but may cause issues with async I2C

	//==-- Transmit Serial
	//Check to see if a message has been sent this loop, then see if a message should be sent
	//Only allow this once per loop due to buffer risks (see serial define above)
	communication_transmit( system_micros() );

	//==-- Timeout Checks
	safety_run( system_micros() );

	status_devices_run( system_micros() );

	//==-- Control Process
	control_run( system_micros() );

	//==-- Send Motor Commands
	//Convert outputs to correct layout and send PWM (and considers failsafes)
	pwm_output( system_micros() );

    //==-- loop time calculation
	sensors_clock_update( system_micros() );

    //==-- Loop rate limiting (if required)
	system_rate_limit();
}
