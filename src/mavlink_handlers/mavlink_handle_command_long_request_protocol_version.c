#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

MAV_RESULT mavlink_handle_command_long_request_protocol_version( uint8_t port, float *params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	uint8_t ver = mavlink_get_proto_version(port);
	mavlink_set_proto_version(port, 2);	//Switch to v2

	const uint8_t blank_array[8] = {0,0,0,0,0,0,0,0};
	mavlink_msg_protocol_version_send(port,
									  MAVLINK_VERSION_MAX,
									  MAVLINK_VERSION_MIN,
									  MAVLINK_VERSION_MAX,
									  &blank_array[0],
									  (uint8_t*)GIT_VERSION_MAVLINK_STR);

	mavlink_set_proto_version(port, ver); //Switch back

	return command_result;
}

#ifdef __cplusplus
}
#endif
