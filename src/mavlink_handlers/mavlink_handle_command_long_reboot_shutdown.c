#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"
#include "drivers/drv_system.h"

MAV_RESULT mavlink_handle_command_long_reboot_shutdown( mavlink_channel_t chan, float *params, uint8_t sysid, uint8_t compid ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	if( safety_request_state( MAV_STATE_POWEROFF ) ) {
		switch( (int)params[0] ) {
			case 1:
				mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[SAFETY] Performing system reset!");
				mavlink_msg_command_ack_pack(mavlink_system.sysid,
											 mavlink_system.compid,
											 get_channel_buf(chan),
											 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
											 MAV_RESULT_ACCEPTED,
											 0xFF,
											 0xFF,
											 sysid,
											 compid);
				comms_send_msg(chan);

				safety_prepare_graceful_shutdown();

				system_reset();

				break;
			case 3:
				mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[SAFETY] Entering bootloader mode!");
				mavlink_msg_command_ack_pack(mavlink_system.sysid,
											 mavlink_system.compid,
											 get_channel_buf(chan),
											 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
											 MAV_RESULT_ACCEPTED,
											 0xFF,
											 0xFF,
											 sysid,
											 compid);
				comms_send_msg(chan);
	
				safety_prepare_graceful_shutdown();

				system_bootloader();

				break;
			default:
				command_result = MAV_RESULT_UNSUPPORTED;

				break;
		}
	} else {
		mavlink_queue_broadcast_error("[SAFETY] Unable to enter poweroff state!");
	}

	return command_result;
}

#ifdef __cplusplus
}
#endif
