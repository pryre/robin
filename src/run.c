#ifdef __cplusplus
extern "C" {
#endif

#include "run.h"

#include "calibration.h"
#include "controller.h"
#include "estimator.h"
#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "profiler.h"

#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"

#include "mavlink/mavlink_types.h"
#include "mavlink_receive.h"
#include "mavlink_system.h"
#include "mavlink_transmit.h"

system_status_t _system_status;

void setup( void ) {
	// Preset status flags
	_system_status.state = MAV_STATE_UNINIT;
	_system_status.mode = MAV_MODE_PREFLIGHT;

	profiler_init();
	profiler_set_start( PROFILER_ID_SETUP, system_micros() );

	// System-specific abstraction initializers
	system_init();

	status_devices_init();

	// Initialize parameters
	params_init();

	// Bring up communications
	communications_system_init();

	// Bring up safety backend
	safety_init();
	// Critical systems are up, now we're into normal boot phase
	safety_request_state( MAV_STATE_BOOT );

	// Initialize the rest of the systems
	// XXX: This locks boot for a while if ESC cal is active, so do it
	// before i2c devices
	mixer_init();

	// XXX: May overrides settings made by pwm backend, so must be
	// after mixer_init()
	sensors_init();

	calibration_init();

	estimator_init();

	control_init();

	// Initialization complete, fall into STANDBY state
	safety_request_state( MAV_STATE_STANDBY );

	profiler_set_end( PROFILER_ID_SETUP, system_micros() );
}

// XXX: Measured CPU load when armed and running: 51.4%
void loop( void ) {
	// TODO: XXX: There will be a timer rollover at ~70 minutes of operation, will
	// cause some issues

	// Take note of when this loop starts
	profiler_set_start( PROFILER_ID_LOOP, system_micros() );

	//TODO: HERE!: Do sensor profiling

	// Sensor Read
	// Check to see if any of the i2c sensors have been updated (mainly the imu)
	// and if so, update the sensor states and estimator
	// If we're not in HIL mode
	if ( !_sensors.hil.status.present ) {
		if ( sensors_read( system_micros() ) ) {
			// Handle any calibration is in progress
			if ( _system_status.state == MAV_STATE_CALIBRATING ) {
				// Run the rest of the calibration logic
				calibration_run();
			}

			profiler_run( PROFILER_ID_ESTIMATOR, &estimator_update_sensors );
		}
	} else {
		if ( _sensors.hil.status.new_data )
			profiler_run( PROFILER_ID_ESTIMATOR, &estimator_update_hil );
	}

	//==-- Communications
	//==-- Check Serial
	profiler_run( PROFILER_ID_COMMS_RX, &communication_receive );

	//==-- Transmit Serial
	// Check to see if a message has been sent this loop, then see if a message
	// should be sent
	// Only allow this once per loop due to buffer risks (see serial define above)
	profiler_run( PROFILER_ID_COMMS_TX, &communication_transmit );

	//==-- Timeout Checks
	profiler_run( PROFILER_ID_SAFETY, &safety_run );

	profiler_run( PROFILER_ID_STATUS, &status_devices_run );

	//==-- Control Process
	profiler_run( PROFILER_ID_CONTROL, &control_run );

	//==-- Send Motor Commands
	// Convert outputs to correct layout and send PWM (and considers failsafes)
	profiler_run( PROFILER_ID_MIXER, &mixer_output );

	//==-- loop time calculation
	profiler_set_end( PROFILER_ID_LOOP, system_micros() );

	//TODO: HERE!: Add parameter for profiler data collection (turn on/off)
	//TODO: HERE!: periodically dump profiler data to comms

	//==-- Loop rate limiting (if required)
	system_rate_limit();
}

#ifdef __cplusplus
}
#endif
