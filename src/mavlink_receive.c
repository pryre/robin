#include "breezystm32.h"

#include "mavlink_receive.h"
#include "mavlink_system.h"
#include <mavlink/common/common.h>
#include "safety.h"
#include "sensors.h"
#include "controller.h"
#include "mixer.h"

#include "fixextra.h"

#include <stdio.h>

uint8_t _system_operation_control;
sensor_calibration_t _sensor_calibration;
mavlink_queue_t _lpq_port_0;
//XXX: mavlink_queue_t _lpq_port_1;

command_input_t _command_input;
system_status_t _system_status;
mavlink_system_t _mavlink_gcs;

static void communication_decode(uint8_t port, uint8_t c) {
	mavlink_message_t msg;
	mavlink_status_t status;

	// Try to get a new message
	if(mavlink_parse_char(port, c, &msg, &status)) {
		if( ( !get_param_uint(PARAM_STRICT_GCS_MATCH) ) ||
		    ( (msg.sysid == _mavlink_gcs.sysid) && (msg.compid == _mavlink_gcs.compid) ) ) {
			//XXX: This may happen automatically in the MAVLINK backend
			//If we detected a mavlink v2 status from GCS, and we're still in v1, switch
			if( ( !(mavlink_get_channel_status(port)->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) ) &&
				( mavlink_get_channel_status(port)->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1 ) ) {
				mavlink_set_proto_version(port, 2);
				mavlink_queue_broadcast_notice("[COMMS] Switching to MAVLINKv2");
			}

			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT: {
					safety_update_sensor(&_system_status.sensors.offboard_heartbeat);

					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					if( (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) &&
						( (mavlink_msg_param_request_list_get_target_component(&msg) == mavlink_system.compid) ||
						  (mavlink_msg_param_request_list_get_target_component(&msg) == MAV_COMP_ID_ALL) ) ) {
						//Set the new request flag
						if(port == MAVLINK_COMM_0) {
							_lpq_port_0.request_all_params = 0;
						//XXX: } else if(port == MAVLINK_COMM_1) {
						//XXX: 	_lpq_port_1.request_all_params = 0;
						}

						mavlink_queue_broadcast_notice("[PARAM] Caution: Broadcasting param list!");
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
								index = lookup_param_id(param_id);
							}

							mavlink_message_t msg_out;
							mavlink_prepare_param_value(&msg_out, index);
							lpq_queue_msg(port, &msg_out);
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

						if(index < PARAMS_COUNT) { //If the ID is valid
							MAV_PARAM_TYPE send_type = mavlink_msg_param_set_get_param_type(&msg);
							MAV_PARAM_TYPE known_type = get_param_type(index);

							bool int_uint_conversion = false;

							//XXX: Some GCS won't send correct parameter types, so we allow relaxed setting
							if( get_param_uint( PARAM_RELAXED_PARAM_SET ) ) {
								if( (send_type == MAV_PARAM_TYPE_INT32 ) && (known_type == MAV_PARAM_TYPE_UINT32 ) ) {
									int_uint_conversion = true;
									known_type = send_type;

									mavlink_queue_broadcast_notice("[PARAM] Using relaxed parameter saving");
								}

							}

							if( send_type == known_type ) {
								switch( send_type ) {
									case MAV_PARAM_TYPE_UINT32: {
										union {
											float f;
											uint32_t u;
										} u;

										u.f = mavlink_msg_param_set_get_param_value(&msg);
										set_complete = set_param_by_name_uint(param_id, u.u);

										break;
									}
									case MAV_PARAM_TYPE_INT32: {
										union {
											float f;
											int32_t i;
										} u;

										u.f = mavlink_msg_param_set_get_param_value(&msg);

										if(!int_uint_conversion) {
											set_complete = set_param_by_name_int(param_id, u.i);
										} else {
											set_complete = set_param_by_name_uint(param_id, (uint32_t)u.i);
										}

										break;
									}
									case MAV_PARAM_TYPE_REAL32: {
										float value = mavlink_msg_param_set_get_param_value(&msg);
										set_complete = set_param_fix16(index, fix16_from_float(value));

										break;
									}
									default:
										mavlink_queue_broadcast_notice("[PARAM] Do not know how to handle read paramater type!");

										break;
								}
							} else {
								//XXX: This may be caused if a GCS sends a UINT32 param as INT32
								mavlink_queue_broadcast_error("[PARAM] Paramater type mismatch!");
							}
						}

						if(set_complete) {
							mavlink_message_t msg_out;
							mavlink_prepare_param_value(&msg_out, index);

							lpq_queue_broadcast_msg(&msg_out);
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
						case MAV_CMD_PREFLIGHT_CALIBRATION: {
							if(_system_status.state == MAV_STATE_CALIBRATING) {	//XXX: Only allow one calibration request at a time
								if( (int)mavlink_msg_command_long_get_param5(&msg) && ( _sensor_calibration.type == SENSOR_CAL_ACCEL ) ) {
									_sensor_calibration.data.accel.waiting = false;
									command_result = MAV_RESULT_ACCEPTED;
								} else {
									command_result = MAV_RESULT_TEMPORARILY_REJECTED;
								}
							} else if( safety_request_state( MAV_STATE_CALIBRATING ) && ( _sensor_calibration.type == SENSOR_CAL_NONE ) ) { //TODO: Note about only doing 1 calibration at a time
								_sensor_calibration.req_sysid = msg.sysid;
								_sensor_calibration.req_compid = msg.compid;

								if( (int)mavlink_msg_command_long_get_param1(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_GYRO;
								} else 	if( (int)mavlink_msg_command_long_get_param2(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_MAG;
								} else if( (int)mavlink_msg_command_long_get_param3(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_BARO;
								} else if( (int)mavlink_msg_command_long_get_param4(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_RC;
								} else if( (int)mavlink_msg_command_long_get_param5(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_ACCEL;
								} else if( (int)mavlink_msg_command_long_get_param6(&msg) ) {
									_sensor_calibration.type |= SENSOR_CAL_INTER;
								}	//TODO: Make sure these are all up to date (some have different values now!)

								command_result = MAV_RESULT_ACCEPTED;
							} else {	//We send the denied immidiately if we can't do it now
								command_result = MAV_RESULT_DENIED;
							}

							need_ack = true;

							break;
						}
						case MAV_CMD_REQUEST_PROTOCOL_VERSION: {
							uint8_t ver = mavlink_get_proto_version(port);
							mavlink_set_proto_version(port, 2);	//Switch to v2

							const uint8_t blank_array[8] = {0,0,0,0,0,0,0,0};
							mavlink_msg_protocol_version_send(port,
															  MAVLINK_VERSION_MAX,
															  MAVLINK_VERSION_MIN,
															  MAVLINK_VERSION_MAX,
															  &blank_array[0],
															  (uint8_t*)GIT_VERSION_MAVLINK_STR);

							mavlink_set_proto_version(port, ver); //Switch back

							break;
						}
						case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
							mavlink_message_t msg_out;
							mavlink_prepare_autopilot_version(&msg_out);
							lpq_queue_msg(port, &msg_out);

							break;
						}
						case MAV_CMD_PREFLIGHT_STORAGE: {
							need_ack = true;

							if( _system_status.state == MAV_STATE_STANDBY) {
								switch( (int)mavlink_msg_command_long_get_param1(&msg) ) {
									case 0:		//Read from flash
										if( read_params() ) {
											command_result = MAV_RESULT_ACCEPTED;
										} else {
											command_result = MAV_RESULT_FAILED;
										}

										break;
									case 1:	//Write to flash
										if( write_params() ) {
											command_result = MAV_RESULT_ACCEPTED;
										} else {
											command_result = MAV_RESULT_FAILED;
										}

										//XXX
										//Reinit imu as writing EEPROM messes up the callback
										sensors_init_imu();
										//XXX

										break;
									case 2:		//Reset to defaults
										set_param_defaults();
										command_result = MAV_RESULT_ACCEPTED;

										break;
									default:	//Not supported
										command_result = MAV_RESULT_UNSUPPORTED;
										break;
								}
							} else {
								command_result = MAV_RESULT_TEMPORARILY_REJECTED;
							}

							break;
						}
						case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
							need_ack = true;

							if( safety_request_state( MAV_STATE_POWEROFF ) ) {
								switch( (int)mavlink_msg_command_long_get_param1(&msg) ) {
									case 1:
										mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[SAFETY] Performing system reset!");
										mavlink_msg_command_ack_send(port, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED, 0xFF, 0xFF, msg.sysid, msg.compid);
										delay(500);	//XXX: Give a few moments for the comms to send

										systemReset();

										break;
									case 3:
										mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[SAFETY] Entering bootloader mode!");
										mavlink_msg_command_ack_send(port, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED, 0xFF, 0xFF, msg.sysid, msg.compid);
										delay(500);	//XXX: Give a few moments for the comms to send

										systemResetToBootloader();

										break;
									default:
										command_result = MAV_RESULT_UNSUPPORTED;

										break;
								}
							} else {
								mavlink_queue_broadcast_error("[SAFETY] Unable to enter poweroff state!");
							}

							break;
						}
						case MAV_CMD_COMPONENT_ARM_DISARM: {	//TODO: For some reason this doesn't return an acceptable result
							need_ack = true;
							command_result = MAV_RESULT_DENIED;

							if( (bool)mavlink_msg_command_long_get_param1(&msg) ) { //ARM
								if( safety_request_arm() )
									command_result = MAV_RESULT_ACCEPTED;
							} else { //DISARM
								if( safety_request_disarm() )
									command_result = MAV_RESULT_ACCEPTED;
							}

							break;
						}
						//TODO: Handle other cases?
						default: {
							need_ack = true;
							command_result = MAV_RESULT_UNSUPPORTED;
							break;
						}
					}

					if(need_ack) {
						mavlink_message_t msg_out;
						mavlink_prepare_command_ack(&msg_out, command, command_result, msg.sysid, msg.compid, 0xFF);
						lpq_queue_msg(port, &msg_out);

						if( command_result == MAV_RESULT_ACCEPTED ) {
							status_buzzer_success();
						} else {
							status_buzzer_failure();
						}
					}

					break;
				}
				case MAVLINK_MSG_ID_SET_MODE: {
					uint8_t base_mode = mavlink_msg_set_mode_get_base_mode(&msg);
					uint32_t custom_mode = mavlink_msg_set_mode_get_custom_mode(&msg);

					if( ( base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ) &&
						( compat_decode_px4_main_mode( custom_mode ) == MAIN_MODE_OFFBOARD ) ) {
						mavlink_send_broadcast_statustext( MAV_SEVERITY_NOTICE, "[SAFETY] Mav already in OFFBOARD mode" );
					} else {
						mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR, "[SAFETY] Unsupported mode" );
					}

					break;
				}
				case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
					if( (mavlink_msg_set_attitude_target_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_set_attitude_target_get_target_component(&msg) == mavlink_system.compid) ) {

						//TODO: Check timestamp was recent before accepting

						//Input Mask
						_command_input.input_mask = mavlink_msg_set_attitude_target_get_type_mask(&msg);

						//Rates
						_command_input.r = fix16_from_float(mavlink_msg_set_attitude_target_get_body_roll_rate(&msg));
						_command_input.p = fix16_from_float(mavlink_msg_set_attitude_target_get_body_pitch_rate(&msg));
						_command_input.y = fix16_from_float(mavlink_msg_set_attitude_target_get_body_yaw_rate(&msg));

						//Attitude
						float qt_float[4];
						qf16 qt_fix;
						mavlink_msg_set_attitude_target_get_q(&msg, &qt_float[0]);

						qt_fix.a = fix16_from_float(qt_float[0]);
						qt_fix.b = fix16_from_float(qt_float[1]);
						qt_fix.c = fix16_from_float(qt_float[2]);
						qt_fix.d = fix16_from_float(qt_float[3]);

						qf16_normalize_to_unit(&_command_input.q, &qt_fix);

						//Trottle
						_command_input.T = fix16_from_float(mavlink_msg_set_attitude_target_get_thrust(&msg));

						//Update Sensor
						safety_update_sensor(&_system_status.sensors.offboard_control);
					}

					break;
				}
				case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
					if( (mavlink_msg_rc_channels_override_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_rc_channels_override_get_target_component(&msg) == mavlink_system.compid) ) {

						_pwm_control[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
						_pwm_control[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
						_pwm_control[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
						_pwm_control[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
						_pwm_control[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
						_pwm_control[5] = mavlink_msg_rc_channels_override_get_chan6_raw(&msg);
						_pwm_control[6] = mavlink_msg_rc_channels_override_get_chan7_raw(&msg);
						_pwm_control[7] = mavlink_msg_rc_channels_override_get_chan8_raw(&msg);

						//Update Sensor
						safety_update_sensor(&_system_status.sensors.pwm_control);
					}

					break;
				}
				case MAVLINK_MSG_ID_ATT_POS_MOCAP: {
					_sensors.ext_pose.status.present = true;

					//TODO: Check timestamp was recent before accepting
					_sensors.ext_pose.status.time_read = micros();

					//Position
					_sensors.ext_pose.p.x = fix16_from_float(mavlink_msg_att_pos_mocap_get_x(&msg));
					_sensors.ext_pose.p.y = fix16_from_float(mavlink_msg_att_pos_mocap_get_y(&msg));
					_sensors.ext_pose.p.z = fix16_from_float(mavlink_msg_att_pos_mocap_get_z(&msg));

					//Attitude
					float qt_float[4];
					qf16 qt_fix;
					mavlink_msg_att_pos_mocap_get_q(&msg, &qt_float[0]);

					qt_fix.a = fix16_from_float(qt_float[0]);
					qt_fix.b = fix16_from_float(qt_float[1]);
					qt_fix.c = fix16_from_float(qt_float[2]);
					qt_fix.d = fix16_from_float(qt_float[3]);

					qf16_normalize_to_unit(&_sensors.ext_pose.q, &qt_fix);

					//Update Sensor
					safety_update_sensor(&_system_status.sensors.ext_pose);

					if( _system_status.sensors.ext_pose.health == SYSTEM_HEALTH_OK )
						_sensors.ext_pose.status.new_data = true;

					break;
				}
				/*
				case MAVLINK_MSG_ID_TIMESYNC: {
					mavlink_timesync_t tsync;
					mavlink_msg_timesync_decode(&msg, &tsync);

					uint32_t now_ms = micros();

					//TODO: Should be in safety_check()?
					if( (now_ms - _sensors.clock.rt_sync_last) > 500000) {	//There hasn't been a sync in a while
						_sensors.clock.rt_offset_ns = 0;
						_sensors.clock.rt_drift = 1.0;
						_sensors.clock.rt_ts_last = 0;
						_sensors.clock.rt_tc_last = 0;
					}

					//Pulled from px4 firmware
					uint64_t now_ns = now_ms * 1000LL;
					uint64_t now_ns_corrected = now_ns * _sensors.clock.rt_drift;

					int64_t time_offset_new = _sensors.clock.rt_offset_ns;

					if (tsync.tc1 == 0) {
						mavlink_send_timesync(port, now_ns_corrected, tsync.ts1);
					} else if (tsync.tc1 > 0) {
						if( (_sensors.clock.rt_ts_last != 0) && (_sensors.clock.rt_ts_last != 0) ) {
							float drift = (float)(tsync.tc1 - _sensors.clock.rt_tc_last) / (float)(tsync.ts1 - _sensors.clock.rt_ts_last);
							_sensors.clock.rt_drift = sensors_clock_smooth_time_drift(_sensors.clock.rt_drift, drift);
						}

						_sensors.clock.rt_ts_last = tsync.ts1;
						_sensors.clock.rt_tc_last = tsync.tc1;

						int64_t offset_ns = (int64_t)(tsync.ts1 + now_ns_corrected - tsync.tc1 * 2) / 2;
						int64_t dt = _sensors.clock.rt_offset_ns - offset_ns;

						if ( abs(dt) > 10000000LL ) { // 10 millisecond skew
							time_offset_new = offset_ns;

							char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
							snprintf(text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "[SENSOR] Hard setting skew: %0.9f", dt / 1e9);
							mavlink_queue_notice_broadcast( &text[0] );
						} else {
							//Filter the new time offset
							time_offset_new = sensors_clock_smooth_time_skew(_sensors.clock.rt_offset_ns, offset_ns);
						}
					}

					_sensors.clock.rt_offset_ns = time_offset_new;
					_sensors.clock.rt_sync_last = now_ms;

					break;
				}
				*/
				default:
					//TODO: Error?
					//Do nothing
					break;
			}
		}
	}
}

void communication_receive(void) {
	const uint32_t time_read_max = 250;	//XXX: Make sure the read step doesn't last more that 250us (means we might drop packets, but it won't lock the system)
	uint32_t time_start_read = micros();

	if( comm_is_open( COMM_CH_0 ) )
		while( serialTotalRxBytesWaiting( Serial1 ) && ( (micros() - time_start_read ) < time_read_max ) )
				communication_decode( MAVLINK_COMM_0, serialRead(Serial1) );

	//XXX: time_start_read = micros();

	//XXX: if( comm_is_open( COMM_CH_1 ) )
	//XXX: 	while( serialTotalRxBytesWaiting( Serial2 ) && ( (micros() - time_start_read ) < time_read_max ) )
	//XXX: 			communication_decode( MAVLINK_COMM_1, serialRead(Serial2) );

	//TODO: Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
}
