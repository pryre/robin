#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "params.h"

mavlink_queue_t _lpq;

void mavlink_handle_param_request_list( mavlink_channel_t chan, mavlink_message_t *msg, mavlink_status_t *status ) {
	if( (mavlink_msg_param_request_list_get_target_system(msg) == mavlink_system.sysid) &&
		( (mavlink_msg_param_request_list_get_target_component(msg) == mavlink_system.compid) ||
		  (mavlink_msg_param_request_list_get_target_component(msg) == MAV_COMP_ID_ALL) ) ) {
		//Set the new request flag
		if(chan == MAVLINK_COMM_0) {
			_lpq.request_all_params_port0 = 0;
		} else if(chan == MAVLINK_COMM_1) {
			_lpq.request_all_params_port1 = 0;
		}

		mavlink_queue_broadcast_notice("[PARAM] Caution: Broadcasting param list!");
	} //Else this message is for someone else
}

#ifdef __cplusplus
}
#endif
