#include "mavlink_receive.h"
#include "mavlink/common/mavlink.h"
#include "breezystm32.h"

void communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status;

	while( serialTotalRxBytesWaiting( Serial1 ) ) {
		uint8_t c = serialRead(Serial1);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
					LED0_TOGGLE;
					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
					break;
				case MAVLINK_MSG_ID_COMMAND_LONG:
					//mavlink_command_long_t command_long;
					//mavlink_msg_command_long_decode(&msg, &command_long);
					//switch command_long.command
						//switch command_long.param1

					switch(mavlink_msg_command_long_get_command(&msg)) {
						case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
							//TODO: Make sure mav is in preflight mode
							switch((int)mavlink_msg_command_long_get_param1(&msg)) {
								case 1:
									systemReset();
									break;
								case 2:
									systemResetToBootloader();
									break;
								default:
									//TODO: Error?
									break;
							}

							break;
							//TODO: Handle other cases?
						default:
							//TODO: Error?
							break;
					}

					break;
				default:
					//TODO: Error?
					//Do nothing
					break;
			}
		}

		// And get the next one
	}

	// Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
}
