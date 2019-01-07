#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "safety.h"

MAV_RESULT
mavlink_handle_command_long_component_arm_disarm( mavlink_channel_t chan,
												  float* params ) {
	MAV_RESULT command_result = MAV_RESULT_DENIED;

	if ( (bool)params[0] ) { // ARM
		if ( safety_request_arm() )
			command_result = MAV_RESULT_ACCEPTED;
	} else { // DISARM
		if ( safety_request_disarm() )
			command_result = MAV_RESULT_ACCEPTED;
	}

	return command_result;
}

#ifdef __cplusplus
}
#endif
