#include "mavlink_receive.h"
#include "mavlink_system.h"
#include "safety.h"
#include "sensors.h"
#include "breezystm32.h"

int32_t _request_all_params;
uint8_t _system_operation_control;
uint8_t _sensor_calibration;
mavlink_queue_t _low_priority_queue;

void communication_receive(void) {
	mavlink_message_t msg;
	mavlink_status_t status;

	while( serialTotalRxBytesWaiting( Serial1 ) ) {
		uint8_t c = serialRead(Serial1);

		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT: {
					//LED0_TOGGLE;
					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					if((mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_param_request_list_get_target_component(&msg) == mavlink_system.compid)) {
						//Set the new request flag
						_request_all_params = 0;
					} //Else this message is for someone else

					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
					if((mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_param_request_read_get_target_component(&msg) == mavlink_system.compid)) {

						int16_t index = mavlink_msg_param_request_read_get_param_index(&msg);

						if(index < PARAMS_COUNT) {
							if(index == -1) {	//Parameter is specified with name
								char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
								mavlink_msg_param_request_read_get_param_id(&msg, param_id);
								index = lookup_param_id(param_id); //TODO: UNTESTED
							}

							if(check_lpq_space_free()) {
								uint8_t i = get_lpq_next_slot();
								_low_priority_queue.buffer_len[i] = mavlink_prepare_param_value(_low_priority_queue.buffer[i], index);
								_low_priority_queue.queued_message_count++;
							}
						}
					} //Else this message is for someone else

					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG: {
					//A command should always have an acknowledge
					bool need_ack = false;
					uint16_t command = mavlink_msg_command_long_get_command(&msg);
					uint8_t command_result = MAV_RESULT_FAILED;

					switch(command) {
						case MAV_CMD_PREFLIGHT_CALIBRATION:
							if(_system_status.state == MAV_STATE_STANDBY) {
								if((int)mavlink_msg_command_long_get_param1(&msg))
									_sensor_calibration |= SENSOR_CAL_GYRO;

								if((int)mavlink_msg_command_long_get_param2(&msg))
									_sensor_calibration |= SENSOR_CAL_MAG;

								if((int)mavlink_msg_command_long_get_param3(&msg))
									_sensor_calibration |= SENSOR_CAL_BARO;

								if((int)mavlink_msg_command_long_get_param4(&msg))
									_sensor_calibration |= SENSOR_CAL_RC;

								if((int)mavlink_msg_command_long_get_param5(&msg))
									_sensor_calibration |= SENSOR_CAL_ACCEL;

								if((int)mavlink_msg_command_long_get_param6(&msg))
									_sensor_calibration |= SENSOR_CAL_INTER;
							} else {	//We send the denied immidiately if we can't do it now
								command_result = MAV_RESULT_DENIED;
								need_ack = true;
							}


							break;
						case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
							if(check_lpq_space_free()) {
								uint8_t i = get_lpq_next_slot();
								_low_priority_queue.buffer_len[i] = mavlink_prepare_autopilot_version(_low_priority_queue.buffer[i]);
								_low_priority_queue.queued_message_count++;
							}
							break;
						case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
							need_ack = true;
							//TODO: Make sure mav is in preflight mode
							switch((int)mavlink_msg_command_long_get_param1(&msg)) {
								case 1:
									_system_operation_control = SYSTEM_OPERATION_REBOOT;
									command_result = MAV_RESULT_ACCEPTED;
									break;
								case 3:
									_system_operation_control = SYSTEM_OPERATION_REBOOT_BOOTLOADER;
									command_result = MAV_RESULT_ACCEPTED;
									break;
								default:
									command_result = MAV_RESULT_UNSUPPORTED;
									break;
							}

							break;
							//TODO: Handle other cases?
						default:
							need_ack = true;
							command_result = MAV_RESULT_UNSUPPORTED;
							break;
					}

					if(need_ack) {
						if(check_lpq_space_free()) {
							uint8_t i = get_lpq_next_slot();
							_low_priority_queue.buffer_len[i] = mavlink_prepare_command_ack(_low_priority_queue.buffer[i], command, command_result);
							_low_priority_queue.queued_message_count++;
						}
					}

					break;
				}
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
