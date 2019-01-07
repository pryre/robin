#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "drivers/drv_status_io.h"

void mavlink_handle_command_long( mavlink_channel_t chan, mavlink_message_t* msg,
								  mavlink_status_t* status ) {
	// A command should always have an acknowledge
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;
	uint16_t command = mavlink_msg_command_long_get_command( msg );

	float params[CMD_LONG_NUM_PARAMS];
	params[0] = mavlink_msg_command_long_get_param1( msg );
	params[1] = mavlink_msg_command_long_get_param2( msg );
	params[2] = mavlink_msg_command_long_get_param3( msg );
	params[3] = mavlink_msg_command_long_get_param4( msg );
	params[4] = mavlink_msg_command_long_get_param5( msg );
	params[5] = mavlink_msg_command_long_get_param6( msg );
	params[6] = mavlink_msg_command_long_get_param7( msg );

	switch ( command ) {
	case MAV_CMD_PREFLIGHT_CALIBRATION: {
		command_result = mavlink_handle_command_long_preflight_calibration(
			chan, params, msg->sysid, msg->compid );
		break;
	}
	case MAV_CMD_DO_SET_MODE: {
		command_result = mavlink_handle_command_long_set_mode( chan, params );
		break;
	}
	case MAV_CMD_DO_MOTOR_TEST: {
		command_result = mavlink_handle_command_long_do_motor_test( chan, params );
		break;
	}
	case MAV_CMD_REQUEST_PROTOCOL_VERSION: {
		command_result = mavlink_handle_command_long_request_protocol_version( chan, params );
		break;
	}
	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
		command_result = mavlink_handle_command_long_request_autopilot_capabilities(
			chan, params );
		break;
	}
	case MAV_CMD_GET_HOME_POSITION: {
		command_result = mavlink_handle_command_long_get_home_position( chan, params );
		break;
	}
	case MAV_CMD_PREFLIGHT_STORAGE: {
		command_result = mavlink_handle_command_long_preflight_storage( chan, params );
		break;
	}
	case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
		command_result = mavlink_handle_command_long_reboot_shutdown(
			chan, params, msg->sysid, msg->compid );
		break;
	}
	case MAV_CMD_COMPONENT_ARM_DISARM: {
		command_result = mavlink_handle_command_long_component_arm_disarm( chan, params );
		break;
	}
	default: {
		command_result = MAV_RESULT_UNSUPPORTED;
		break;
	}
	}

	if ( command_result < MAV_RESULT_ENUM_END ) {
		mavlink_message_t msg_out;
		mavlink_prepare_command_ack( &msg_out, command, command_result, msg->sysid,
									 msg->compid, 0xFF );
		lpq_queue_msg( chan, &msg_out );

		if ( command_result == MAV_RESULT_ACCEPTED ) {
			status_buzzer_success();
		} else {
			status_buzzer_failure();
		}
	}
}

#ifdef __cplusplus
}
#endif
