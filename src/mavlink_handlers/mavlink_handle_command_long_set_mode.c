#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"

MAV_RESULT mavlink_handle_command_long_set_mode( uint8_t port, float *params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	//XXX: We only use custom mode
	uint8_t base_mode = (int)params[0];
	uint8_t custom_mode = (int)params[1];

	if( base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ) {
		if( safety_request_control_mode(compat_decode_px4_main_mode(custom_mode)) ) {
			command_result = MAV_RESULT_ACCEPTED;
		} else {
			mavlink_queue_broadcast_error("[SAFETY] Rejecting mode switch");
			command_result = MAV_RESULT_DENIED;
		}
	} else {
		mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR, "[SAFETY] Unsupported mode" );
		command_result = MAV_RESULT_FAILED;
	}

	return command_result;
}
