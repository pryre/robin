#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "params.h"
#include "safety.h"

void mavlink_handle_set_mode( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status ) {
	uint8_t base_mode = mavlink_msg_set_mode_get_base_mode( msg );
	uint32_t custom_mode = mavlink_msg_set_mode_get_custom_mode( msg );

	if ( base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ) {
		if ( !safety_request_control_mode( compat_decode_px4_main_mode( custom_mode ) ) ) {
			mavlink_queue_broadcast_error( "[SAFETY] Rejecting mode switch" );
		}
	} else {
		mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR, "[SAFETY] Unsupported mode" );
	}
}

#ifdef __cplusplus
}
#endif
