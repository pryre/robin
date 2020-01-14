#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

void mavlink_handle_ping( mavlink_channel_t chan,
						  mavlink_message_t* msg,
						  mavlink_status_t* status ) {

	if( ( mavlink_msg_ping_get_target_system( msg ) == 0 ) &&
		( mavlink_msg_ping_get_target_component( msg ) == 0 ) ) {

		mavlink_message_t msg_out;
		mavlink_msg_ping_pack(mavlink_system.sysid,
							  mavlink_system.compid,
							  &msg_out,
							  mavlink_msg_ping_get_time_usec(msg),
							  mavlink_msg_ping_get_seq(msg),
							  msg->sysid,
							  msg->compid);

		lpq_queue_msg( chan, &msg_out );
	} // Else this message is a return ping
}

#ifdef __cplusplus
}
#endif
