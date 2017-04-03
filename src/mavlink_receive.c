#include "mavlink_receive.h"
#include "mavlink_system.h"
#include "safety.h"
#include "sensors.h"
#include "controller.h"
#include "breezystm32.h"

int32_t _request_all_params;
uint8_t _system_operation_control;
uint8_t _sensor_calibration;
mavlink_queue_t _low_priority_queue;

command_input_t _command_input;

void communication_receive(void) {
	mavlink_message_t msg;
	mavlink_status_t status;

	//TODO: Have a check on Serial 0 for... all the same messages?
	//TODO: That would mean there is only a need to have 1 parse function, and pass the right port and buffer.
	while( serialTotalRxBytesWaiting( Serial1 ) ) {
		uint8_t c = serialRead(Serial1);

		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT: {
					//TODO: Log to make keep track of connected systems status'
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
				case MAVLINK_MSG_ID_PARAM_SET: {
					if((mavlink_msg_param_set_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_param_set_get_target_component(&msg) == mavlink_system.compid)) {

						bool set_complete = false;

						char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						mavlink_msg_param_set_get_param_id(&msg, param_id);
						param_id_t index = lookup_param_id(param_id);

						//TODO: Remember to mention in docs that the sent dat is entered as the type, regardless of what type it should be
						//TODO: Should probably have a safetly check for this though
						if(index < PARAMS_COUNT) { //If the ID is valid
							switch(mavlink_msg_param_set_get_param_type(&msg)) {
								case MAV_PARAM_TYPE_INT32: {
									union {
										float f;
										uint32_t i;
									} u;

									u.f = mavlink_msg_param_set_get_param_value(&msg);
									set_complete = set_param_by_name_int(param_id, u.i);

									break;
								}
								case MAV_PARAM_TYPE_REAL32: {
									float value = mavlink_msg_param_set_get_param_value(&msg);
									set_complete = set_param_fix16(index, fix16_from_float(value));

									break;
								}
								default:
									break;
							}

							if(set_complete) {
								if(check_lpq_space_free()) {
									uint8_t i = get_lpq_next_slot();
									_low_priority_queue.buffer_len[i] = mavlink_prepare_param_value(_low_priority_queue.buffer[i], index);
									_low_priority_queue.queued_message_count++;
								}
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
								_system_status.state = MAV_STATE_CALIBRATING;

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
						case MAV_CMD_PREFLIGHT_STORAGE:
							need_ack = true;

							switch((int)mavlink_msg_command_long_get_param1(&msg)) {
								case 0:	//Read from flash
									if(read_params()) {
										command_result = MAV_RESULT_ACCEPTED;
									} else {
										command_result = MAV_RESULT_FAILED;
									}

									break;

								//TODO: Writing to EEPROM doesn't really work after boot
								/*
								case 1:	//Write to flash

									if(write_params()) {
										command_result = MAV_RESULT_ACCEPTED;
									} else {
										command_result = MAV_RESULT_FAILED;
									}

									break;
								*/
								case 2:	//Reset to defaults
									set_param_defaults();
									command_result = MAV_RESULT_ACCEPTED;

									break;
								default://Not supported
									command_result = MAV_RESULT_UNSUPPORTED;
									break;
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
				case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
					//TODO: Check timestamp was recent
					//TODO: Check target system
					//TODO: Check target component
					_command_input.input_mask = mavlink_msg_set_attitude_target_get_type_mask(&msg);

					_command_input.r = fix16_from_float(mavlink_msg_set_attitude_target_get_body_roll_rate(&msg));
					_command_input.p = fix16_from_float(mavlink_msg_set_attitude_target_get_body_pitch_rate(&msg));
					_command_input.y = fix16_from_float(mavlink_msg_set_attitude_target_get_body_yaw_rate(&msg));

					//TODO: Check this is correct
					float qt[4];
					if(mavlink_msg_set_attitude_target_get_q(&msg, qt) == 4) {
						_command_input.q.a = fix16_from_float(qt[0]);
						_command_input.q.b = fix16_from_float(qt[1]);
						_command_input.q.c = fix16_from_float(qt[2]);
						_command_input.q.d = fix16_from_float(qt[3]);
					}

					_command_input.T = fix16_from_float(mavlink_msg_set_attitude_target_get_thrust(&msg));

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
