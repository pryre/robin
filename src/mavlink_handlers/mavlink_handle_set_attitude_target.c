#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "controller.h"
#include "params.h"
#include "safety.h"

#include "fix16.h"
#include "fixextra.h"

// command_input_t _cmd_ob_input;
static bool attempted_ob_default_mode_change_ = false;

void mavlink_handle_set_attitude_target( mavlink_channel_t chan,
										 mavlink_message_t* msg,
										 mavlink_status_t* status ) {
	if ( ( mavlink_msg_set_attitude_target_get_target_system( msg ) == mavlink_system.sysid ) && ( mavlink_msg_set_attitude_target_get_target_component( msg ) == mavlink_system.compid ) ) {

		// TODO: Check timestamp was recent before accepting

		// Input Mask
		_cmd_ob_input.input_mask = mavlink_msg_set_attitude_target_get_type_mask( msg );

		// Rates
		_cmd_ob_input.r = fix16_from_float(
			mavlink_msg_set_attitude_target_get_body_roll_rate( msg ) );
		_cmd_ob_input.p = fix16_from_float(
			mavlink_msg_set_attitude_target_get_body_pitch_rate( msg ) );
		_cmd_ob_input.y = fix16_from_float(
			mavlink_msg_set_attitude_target_get_body_yaw_rate( msg ) );

		// Attitude
		float qt_float[4];
		qf16 qt_fix;
		mavlink_msg_set_attitude_target_get_q( msg, &qt_float[0] );

		qt_fix.a = fix16_from_float( qt_float[0] );
		qt_fix.b = fix16_from_float( qt_float[1] );
		qt_fix.c = fix16_from_float( qt_float[2] );
		qt_fix.d = fix16_from_float( qt_float[3] );

		qf16_normalize_to_unit( &_cmd_ob_input.q, &qt_fix );

		// Trottle
		_cmd_ob_input.T = fix16_from_float( mavlink_msg_set_attitude_target_get_thrust( msg ) );

		// Update Sensor
		safety_update_sensor( &_system_status.sensors.offboard_control );

		//==-- Offboard Control Default Mode
		//	Have an option to fall into OFFBOARD mode if
		//	we detect a connection similar to the RC
		//	default mode. Allows any mode to be requested.
		if( !attempted_ob_default_mode_change_ &&
			( _system_status.control_mode == MAIN_MODE_UNSET ) &&
			get_param_uint( PARAM_OB_MODE_DEFAULT ) &&
			( _system_status.sensors.offboard_control.health == SYSTEM_HEALTH_OK ) &&
			( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) ) {
			// RC connection should trigger a default mode change
			// We only allow this to occur once after boot
			// in case RC drops and reconnects mid-flight
			// This also only occurs if the mode is unset,
			// and the default is valid, for the same reason
			if ( !safety_request_control_mode( MAIN_MODE_OFFBOARD ) ) {
				mavlink_queue_broadcast_error(
					"[SENSOR] Error setting OB default mode" );
			}

			attempted_ob_default_mode_change_ = true;
		}
	}
}

#ifdef __cplusplus
}
#endif
