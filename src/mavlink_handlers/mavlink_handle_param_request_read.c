#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "params.h"

void mavlink_handle_param_request_read( mavlink_channel_t chan,
										mavlink_message_t* msg,
										mavlink_status_t* status ) {
	if ( ( mavlink_msg_param_request_read_get_target_system( msg ) == mavlink_system.sysid ) && ( mavlink_msg_param_request_read_get_target_component( msg ) == mavlink_system.compid ) ) {

		int16_t index = mavlink_msg_param_request_read_get_param_index( msg );

		if ( index < PARAMS_COUNT ) {
			if ( index == -1 ) { // Parameter is specified with name
				char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
				mavlink_msg_param_request_read_get_param_id( msg, param_id );
				index = lookup_param_id( param_id );
			}

			mavlink_message_t msg_out;
			mavlink_prepare_param_value( &msg_out, index );
			lpq_queue_msg( chan, &msg_out );
		}
	} // Else this message is for someone else
}

#ifdef __cplusplus
}
#endif
