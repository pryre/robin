#ifdef __cplusplus
extern "C" {
#endif

#include "calibration.h"
#include "mavlink_system.h"

#include "drivers/drv_pwm.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"
#include "mixer.h"
#include "params.h"

bool calibrate_esc( void ) {
	bool failed = false;

	if ( get_param_uint( PARAM_DO_ESC_CAL ) ) {
		mavlink_send_broadcast_statustext( MAV_SEVERITY_NOTICE, "[MIXER] Performing ESC calibration" );

		for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ )
			if ( _mixer_to_use->output_type[i] == IO_TYPE_OM )
				drv_pwm_write( i, get_param_uint( PARAM_MOTOR_PWM_MAX ) );

		status_led_heart_set( false );
		bool led_toggle = false;
		for ( uint8_t i = 0; i < 20; i++ ) {
			status_led_arm_set( led_toggle );
			system_pause_ms( 100 );
		}

		for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ )
			if ( _mixer_to_use->output_type[i] == IO_TYPE_OM )
				drv_pwm_write( i, get_param_uint( PARAM_MOTOR_PWM_MIN ) );

		status_led_arm_set( false );
		status_led_heart_set( false );

		status_buzzer_success();
		mavlink_send_broadcast_statustext( MAV_SEVERITY_INFO, "[MIXER] ESC calibration complete!" );

		set_param_uint( PARAM_DO_ESC_CAL, 0 );

		write_params();
	}

	return !failed;
}

#ifdef __cplusplus
}
#endif
