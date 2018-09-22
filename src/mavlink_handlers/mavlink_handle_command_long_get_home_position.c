#include "mavlink_system.h"
#include "mavlink_receive.h"

MAV_RESULT mavlink_handle_command_long_get_home_position( uint8_t port, float *params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	//XXX: Just give a false message to have it handled if it is requested
	mavlink_message_t msg_out;
	mavlink_prepare_home_position(&msg_out);
	lpq_queue_msg(port, &msg_out);

	return command_result;
}
