#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

MAV_RESULT mavlink_handle_command_long_request_autopilot_capabilities(
	mavlink_channel_t chan, float* params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	mavlink_message_t msg_out;
	mavlink_prepare_autopilot_version( &msg_out );
	lpq_queue_msg( chan, &msg_out );

	return command_result;
}

#ifdef __cplusplus
}
#endif
