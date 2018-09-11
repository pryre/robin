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
mavlink_queue_t _lpq;
bool _ch_0_have_heartbeat;
bool _ch_1_have_heartbeat;

bool _actuator_apply_g2;
fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];

int32_t _pwm_control[MIXER_NUM_MOTORS];
mixer_motor_test_t _motor_test;
command_input_t _cmd_ob_input;
system_status_t _system_status;
mavlink_system_t _mavlink_gcs;

static bool communication_decode(uint8_t port, uint8_t c) {
	bool msg_parsed = false;
	mavlink_message_t msg;
	mavlink_status_t status;

	// Try to get a new message
	if(mavlink_parse_char(port, c, &msg, &status)) {
		msg_parsed = true;

		if( ( !get_param_uint(PARAM_STRICT_GCS_MATCH) ) ||
		    ( (msg.sysid == _mavlink_gcs.sysid) && (msg.compid == _mavlink_gcs.compid) ) ) {
			//XXX: This may happen automatically in the MAVLINK backend
			//If we detected a mavlink v2 status from GCS, and we're still in v1, switch
			if( ( !(mavlink_get_channel_status(port)->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) ) &&
				( mavlink_get_proto_version(port) == 1) ) {
				mavlink_set_proto_version(port, 2);
				mavlink_queue_broadcast_notice("[COMMS] Switching to MAVLINKv2");
			}

			// Handle message
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT: {
					safety_update_sensor(&_system_status.sensors.offboard_heartbeat);

					//XXX: This won't operate each independently, which is probably not a good thing
					if(_system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK) {
						if(port == MAVLINK_COMM_0) {
							_ch_0_have_heartbeat = true;
						} else if(port == MAVLINK_COMM_1) {
							_ch_1_have_heartbeat = true;
						}
					} else {
						if(port == MAVLINK_COMM_0) {
							_ch_0_have_heartbeat = false;
						} else if(port == MAVLINK_COMM_1) {
							_ch_1_have_heartbeat = false;
						}
					}

					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					if( (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) &&
						( (mavlink_msg_param_request_list_get_target_component(&msg) == mavlink_system.compid) ||
						  (mavlink_msg_param_request_list_get_target_component(&msg) == MAV_COMP_ID_ALL) ) ) {
						//Set the new request flag
						if(port == MAVLINK_COMM_0) {
							_lpq.request_all_params_port0 = 0;
						} else if(port == MAVLINK_COMM_1) {
							_lpq.request_all_params_port1 = 0;
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

						char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						mavlink_msg_param_set_get_param_id(&msg, param_id);
						param_id_t index = lookup_param_id(param_id);

						if(index < PARAMS_COUNT) { //If the ID is valid
							MAV_PARAM_TYPE send_type = mavlink_msg_param_set_get_param_type(&msg);
							MAV_PARAM_TYPE known_type = get_param_type(index);

							bool right_type = (known_type == send_type);
							if( (!right_type) && get_param_uint( PARAM_RELAXED_PARAM_SET ) ) {
								right_type = true;
								mavlink_queue_broadcast_notice("[PARAM] Using relaxed parameter saving");
							}

							if(right_type) {
								bool set_failed = false;

								union {
									float f;
									int32_t i;
									uint32_t u;
								} u;

								u.f = mavlink_msg_param_set_get_param_value(&msg);

								switch( send_type ) {
									case MAV_PARAM_TYPE_UINT32: {
										uint32_t val = 0;

										if(known_type == MAV_PARAM_TYPE_UINT32) {
											val = u.u;
										} else if(known_type == MAV_PARAM_TYPE_INT32) {
											val = (uint32_t)u.i;
										} else if(known_type == MAV_PARAM_TYPE_REAL32) {
											val = (uint32_t)u.f;
										} else { //Must be real
											set_failed = true;
										}

										if(!set_failed)
											set_param_by_name_uint(param_id, val);

										break;
									}
									case MAV_PARAM_TYPE_INT32: {
										int32_t val = 0;

										if(known_type == MAV_PARAM_TYPE_UINT32) {
											val = u.u;
										} else if(known_type == MAV_PARAM_TYPE_INT32) {
											val = u.i;
										} else if(known_type == MAV_PARAM_TYPE_REAL32) {
											val = u.f;
										} else { //Must be real
											set_failed = true;
										}

										if(!set_failed)
											set_param_by_name_int(param_id, val);

										break;
									}
									case MAV_PARAM_TYPE_REAL32: {
										float val = 0;

										if(known_type == MAV_PARAM_TYPE_UINT32) {
											val = u.u;
										} else if(known_type == MAV_PARAM_TYPE_INT32) {
											val = u.i;
										} else if(known_type == MAV_PARAM_TYPE_REAL32) {
											val = u.f;
										} else { //Must be real
											set_failed = true;
										}

										if(!set_failed)
											set_param_by_name_fix16(param_id, fix16_from_float(val));

										break;
									}
									default: {
										mavlink_queue_broadcast_notice("[PARAM] Do not know how to handle sent paramater type!");

										break;
									}
								}

								if(set_failed)
									mavlink_queue_broadcast_error("[PARAM] Failed to determine known parameter type!");
							} else {
								//XXX: This may be caused if a GCS sends a UINT32 param as INT32
								mavlink_queue_broadcast_error("[PARAM] Paramater type mismatch!");
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
						case MAV_CMD_PREFLIGHT_CALIBRATION: {
							if(_system_status.state == MAV_STATE_CALIBRATING) {	//XXX: Only allow one calibration request at a time
								if( ( (int)mavlink_msg_command_long_get_param5(&msg) == SENSOR_CAL_CMD_ACCEL) && ( _sensor_calibration.type == SENSOR_CAL_ACCEL ) ) {
									_sensor_calibration.data.accel.waiting = false;
									command_result = MAV_RESULT_ACCEPTED;
								} else if( ( (int)mavlink_msg_command_long_get_param4(&msg) == SENSOR_CAL_CMD_RC) && ( _sensor_calibration.type == SENSOR_CAL_RC ) ) {
									if(_sensor_calibration.data.rc.waiting) {
										_sensor_calibration.data.rc.waiting = false;
									} else if(_sensor_calibration.data.rc.step == SENSOR_CAL_RC_RANGE_EXTREMES) {
										_sensor_calibration.data.rc.step = SENSOR_CAL_RC_RANGE_DONE;
									}

									command_result = MAV_RESULT_ACCEPTED;
								} else {
									mavlink_queue_broadcast_error("[SENSOR] Automatic calibration in progress");
									command_result = MAV_RESULT_TEMPORARILY_REJECTED;
								}
							} else if ( _sensor_calibration.type == SENSOR_CAL_NONE ) {
								_sensor_calibration.req_sysid = msg.sysid;
								_sensor_calibration.req_compid = msg.compid;

								command_result = MAV_RESULT_DENIED;

								if( (int)mavlink_msg_command_long_get_param1(&msg) == SENSOR_CAL_CMD_GYRO ) {
									command_result = sensors_request_cal( SENSOR_CAL_GYRO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								//XXX: } else if( (int)mavlink_msg_command_long_get_param1(&msg) == SENSOR_CAL_CMD_GYRO_TEMP ) {
								//XXX: TODO: GYRO TEMP
								} else if( (int)mavlink_msg_command_long_get_param2(&msg) == SENSOR_CAL_CMD_MAG ) {
									command_result = sensors_request_cal( SENSOR_CAL_MAG ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								} else if( (int)mavlink_msg_command_long_get_param3(&msg) == SENSOR_CAL_CMD_PRESSURE_GND) {
									command_result = sensors_request_cal( SENSOR_CAL_GND_PRESSURE ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								} else if( (int)mavlink_msg_command_long_get_param4(&msg) == SENSOR_CAL_CMD_RC ) {
									command_result = sensors_request_cal( SENSOR_CAL_RC ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								//: } else if( (int)mavlink_msg_command_long_get_param4(&msg) == SENSOR_CAL_CMD_RC_TRIM ) {
								//XXX: RC is done during normal RC cal, maybe it shouldn't?
								} else if( (int)mavlink_msg_command_long_get_param5(&msg) == SENSOR_CAL_CMD_ACCEL ) {
									command_result = sensors_request_cal( SENSOR_CAL_ACCEL ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								} else if( (int)mavlink_msg_command_long_get_param5(&msg) == SENSOR_CAL_CMD_ACCEL_LEVEL ) {
									command_result = sensors_request_cal( SENSOR_CAL_LEVEL_HORIZON ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								//XXX: } else if( (int)mavlink_msg_command_long_get_param5(&msg) == SENSOR_CAL_CMD_ACCEL_TEMP ) {
								//XXX: TODO: ACCEL TEMP
								} else if( (int)mavlink_msg_command_long_get_param6(&msg) == SENSOR_CAL_CMD_COMPASS_MOTOR ) {
									command_result = sensors_request_cal( SENSOR_CAL_INTER ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								//XXX:} else if( (int)mavlink_msg_command_long_get_param6(&msg) == SENSOR_CAL_CMD_AIRPSEED ) {
								//XXX: TODO: Airpspeed?
								} else if( (int)mavlink_msg_command_long_get_param7(&msg) == SENSOR_CAL_CMD_ESC ) {
									//Manually reset calibration mode
									if( set_param_uint( PARAM_DO_ESC_CAL, 1 ) ) {
										write_params();

										command_result = MAV_RESULT_ACCEPTED;
										mavlink_queue_broadcast_notice("[SENSOR] ESC cal will be run next reboot");
									} else {
										mavlink_queue_broadcast_error("[SENSOR] Failed to configure ESC cal!");
									}
								} else if( (int)mavlink_msg_command_long_get_param7(&msg) == SENSOR_CAL_CMD_BAROMETER ) {
									command_result = sensors_request_cal( SENSOR_CAL_BARO ) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
								}
							} else {
								mavlink_queue_broadcast_error("[SENSOR] Calibration already in progress");
							}

							need_ack = true;

							break;
						}
						case MAV_CMD_DO_SET_MODE: {

							//XXX: We only use custom mode
							uint8_t base_mode = (int)mavlink_msg_command_long_get_param1(&msg);
							uint8_t custom_mode = (int)mavlink_msg_command_long_get_param2(&msg);

							if( base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ) {
								if( !safety_request_control_mode(compat_decode_px4_main_mode(custom_mode)) ) {
									mavlink_queue_broadcast_error("[SAFETY] Rejecting mode switch");
								}
							} else {
								mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR, "[SAFETY] Unsupported mode" );
							}

							need_ack = true;

							break;
						}
						case MAV_CMD_DO_MOTOR_TEST: {
							need_ack = true;

							if( _system_status.sensors.pwm_control.health == SYSTEM_HEALTH_OK ) {
								mavlink_queue_broadcast_error("[MIXER] Cannot run motor test, PWM control is active!");

								command_result = MAV_RESULT_TEMPORARILY_REJECTED;
							} else if(!safety_is_armed()) {
								mavlink_queue_broadcast_error("[MIXER] Cannot run motor test unless armed!");

								command_result = MAV_RESULT_TEMPORARILY_REJECTED;
							} else {
								uint8_t motor_test_number = (int)mavlink_msg_command_long_get_param1(&msg);
								//fix16_t motor_test_type = fix16_from_float(mavlink_msg_command_long_get_param2(&msg));
								fix16_t motor_test_throttle = fix16_from_float(mavlink_msg_command_long_get_param3(&msg));
								fix16_t motor_test_timeout = fix16_from_float(mavlink_msg_command_long_get_param4(&msg));
								//uint8_t motor_test_count = (int)mavlink_msg_command_long_get_param5(&msg);
								//uint8_t motor_test_order = (int)mavlink_msg_command_long_get_param6(&msg);

								if( ( (motor_test_number < MIXER_NUM_MOTORS) ||
									  (motor_test_number == MIXER_TEST_MOTORS_ALL) ) &&
									( (motor_test_throttle > 0) && (motor_test_throttle < _fc_1) ) &&
									( motor_test_timeout > 0 ) ) {

									//Configure motor test
									if(motor_test_number == MIXER_TEST_MOTORS_ALL) {
										_motor_test.test_all = true;
										_motor_test.motor_step = 0;
									} else {
										_motor_test.motor_step = motor_test_number;
									}

									_motor_test.start = micros();
									_motor_test.duration = 10000*fix16_to_int(fix16_mul(motor_test_timeout, _fc_100)); //XXX: Motor test should be accurate to 0.01 seconds, and max out at ~1min
									_motor_test.throttle = motor_test_throttle;

									//XXX: Override sensor health as autopilot is in control
									_system_status.sensors.pwm_control.count = get_param_uint(_system_status.sensors.pwm_control.param_stream_count);
									safety_update_sensor(&_system_status.sensors.pwm_control);

									char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[MIXER] Testing motor: ";
									char mchar[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
									itoa(_motor_test.motor_step, mchar, 10);
									strncat(text, mchar, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
									mavlink_queue_broadcast_notice(text);

									command_result = MAV_RESULT_ACCEPTED;
								} else {
									mavlink_queue_broadcast_error("[MIXER] Cannot run motor test, bad test variables!");

									command_result = MAV_RESULT_FAILED;
								}
							}

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
						case MAV_CMD_GET_HOME_POSITION: {
							//XXX: Just give a false message to have it handled if it is requested
							mavlink_message_t msg_out;
							mavlink_prepare_home_position(&msg_out);
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

										//XXX: Graceful Shutdown
										sensors_clear_i2c();
										//while( i2c_job_queued() ); //Wait for jobs to finish

										systemReset();

										break;
									case 3:
										mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[SAFETY] Entering bootloader mode!");
										mavlink_msg_command_ack_send(port, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED, 0xFF, 0xFF, msg.sysid, msg.compid);
										delay(500);	//XXX: Give a few moments for the comms to send

										//XXX: Graceful Shutdown
										sensors_clear_i2c();
										//while( i2c_job_queued() ); //Wait for jobs to finish

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

					if( base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ) {
						if( !safety_request_control_mode(compat_decode_px4_main_mode(custom_mode)) ) {
							mavlink_queue_broadcast_error("[SAFETY] Rejecting mode switch");
						}
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
						_cmd_ob_input.input_mask = mavlink_msg_set_attitude_target_get_type_mask(&msg);

						//Rates
						_cmd_ob_input.r = fix16_from_float(mavlink_msg_set_attitude_target_get_body_roll_rate(&msg));
						_cmd_ob_input.p = fix16_from_float(mavlink_msg_set_attitude_target_get_body_pitch_rate(&msg));
						_cmd_ob_input.y = fix16_from_float(mavlink_msg_set_attitude_target_get_body_yaw_rate(&msg));

						//Attitude
						float qt_float[4];
						qf16 qt_fix;
						mavlink_msg_set_attitude_target_get_q(&msg, &qt_float[0]);

						qt_fix.a = fix16_from_float(qt_float[0]);
						qt_fix.b = fix16_from_float(qt_float[1]);
						qt_fix.c = fix16_from_float(qt_float[2]);
						qt_fix.d = fix16_from_float(qt_float[3]);

						qf16_normalize_to_unit(&_cmd_ob_input.q, &qt_fix);

						//Trottle
						_cmd_ob_input.T = fix16_from_float(mavlink_msg_set_attitude_target_get_thrust(&msg));

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
					_sensors.ext_pose.status.new_data = true;

					break;
				}
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
					//XXX: We don't support missions, so just send back 0
					mavlink_message_t msg_out;
					mavlink_msg_mission_count_pack(mavlink_system.sysid,
												   mavlink_system.compid,
												   &msg_out,
												   mavlink_system.sysid,
												   mavlink_system.compid,
												   0, 0);
					lpq_queue_msg(port, &msg_out);

					break;
				}
				case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET: {
					if( (mavlink_msg_set_actuator_control_target_get_target_system(&msg) == mavlink_system.sysid) &&
						(mavlink_msg_set_actuator_control_target_get_target_component(&msg) == mavlink_system.compid) ) {

						float act_float[8];
						mavlink_msg_set_actuator_control_target_get_controls(&msg, &act_float[0]);

						if( mavlink_msg_set_actuator_control_target_get_group_mlx(&msg) == 0) {
							//XXX: Use this instead of PWM Overrides
							//for(int i=0; i<8; i++)
							//	_actuator_control_g0[i] = fix16_from_float(act_float[i]);
						} else if( mavlink_msg_set_actuator_control_target_get_group_mlx(&msg) == 2) {
							if(!_actuator_apply_g2) {
								mavlink_queue_broadcast_info("[MIXER] Accepted offboard aux. actuator control");
								_actuator_apply_g2 = true;
							}

							for(int i=0; i<8; i++)
								_actuator_control_g2[i] = fix16_from_float(act_float[i]);
						}
					}

					break;
				}
				case MAVLINK_MSG_ID_HIL_SENSOR: {
					if(_sensors.hil.status.present) {
						//Accelerometer
						//XXX: TODO: THIS SHOULDN'T NEED TO BE INVERSED
						_sensors.hil.accel.x = -fix16_from_int(mavlink_msg_hil_sensor_get_xacc(&msg));
						_sensors.hil.accel.y = -fix16_from_int(mavlink_msg_hil_sensor_get_yacc(&msg));
						_sensors.hil.accel.z = -fix16_from_int(mavlink_msg_hil_sensor_get_zacc(&msg));
						//Gyroscope
						_sensors.hil.gyro.x = fix16_from_float(mavlink_msg_hil_sensor_get_xgyro(&msg));
						_sensors.hil.gyro.y = fix16_from_float(mavlink_msg_hil_sensor_get_ygyro(&msg));
						_sensors.hil.gyro.z = fix16_from_float(mavlink_msg_hil_sensor_get_zgyro(&msg));
						//Magnetometer
						_sensors.hil.mag.x = fix16_from_float(mavlink_msg_hil_sensor_get_xmag(&msg));
						_sensors.hil.mag.y = fix16_from_float(mavlink_msg_hil_sensor_get_ymag(&msg));
						_sensors.hil.mag.z = fix16_from_float(mavlink_msg_hil_sensor_get_zmag(&msg));
						//Differential Pressure
						_sensors.hil.pressure_abs = fix16_from_float(mavlink_msg_hil_sensor_get_abs_pressure(&msg));
						_sensors.hil.pressure_diff = fix16_from_float(mavlink_msg_hil_sensor_get_diff_pressure(&msg));
						_sensors.hil.pressure_alt = fix16_from_float(mavlink_msg_hil_sensor_get_pressure_alt(&msg));
						//Temperature
						_sensors.hil.temperature = fix16_from_float(mavlink_msg_hil_sensor_get_temperature(&msg));

						safety_update_sensor(&_system_status.sensors.hil);
						_sensors.hil.status.new_data = true;
					}	//else no in HIL mode, discard

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

	return msg_parsed;
}

void communication_receive(void) {
	//XXX: Make sure the read step doesn't last more that 250us
	//	   (means we might drop packets, but it won't lock the system)
	const uint32_t time_read_max = 250;
	uint32_t time_start_read = micros();

	//Read in as many byts as we can until either
	//both ports are empty, have both read messages,
	//or the time_read_max is hit
	while( (micros() - time_start_read ) < time_read_max ) {
		bool port0_done = !comm_is_open( COMM_PORT_0 );
		bool port1_done = !comm_is_open( COMM_PORT_1 );

		if( !port0_done ) {
			if( serialTotalRxBytesWaiting( Serial1 ) ) {
					port0_done = communication_decode( MAVLINK_COMM_0, serialRead(Serial1) );
			} else {
				port0_done = true;
			}
		}

		if( !port1_done ) {
			if( serialTotalRxBytesWaiting( Serial2 ) ) {
					port1_done = communication_decode( MAVLINK_COMM_1, serialRead(Serial2) );
			} else {
				port1_done = true;
			}
		}

		if(port0_done && port1_done)
			break;
	}
	//TODO: Update global packet drops counter
	//packet_drops += status.packet_rx_drop_count;
}
