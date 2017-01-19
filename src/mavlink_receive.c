#include "mavlink_receive.h"
#include "mavlink/common/mavlink.h"

static void communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status;

	while(1)//uart0_char_available())
	{
		uint8_t c ;//= uart0_get_char();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message

			switch(msg.msgid)
			{
			        case MAVLINK_MSG_ID_HEARTBEAT:
			        {
				  // E.g. read GCS heartbeat and go into
                                  // comm lost mode if timer times out
			        }
			        break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				// EXECUTE ACTION
				break;
			default:
				//Do nothing
				break;
			}
		}

		// And get the next one
	}

	// Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
}
