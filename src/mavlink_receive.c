#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "params.h"
#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include <stdio.h>
//#include "fixextra.h"

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
					mavlink_handle_heartbeat( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					mavlink_handle_param_request_list( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
					mavlink_handle_param_request_read( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_PARAM_SET: {
					mavlink_handle_param_set( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG: {
					mavlink_handle_command_long( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_SET_MODE: {
					mavlink_handle_set_mode( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
					mavlink_handle_set_attitude_target( port, &msg, &status );
					break;
				}
				/*
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
				*/
				case MAVLINK_MSG_ID_ATT_POS_MOCAP: {
					mavlink_handle_att_pos_mocap( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
					mavlink_handle_mission_request_list( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET: {
					mavlink_handle_set_actuator_control_target( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_HIL_SENSOR: {
					mavlink_handle_hil_sensor( port, &msg, &status );
					break;
				}
				case MAVLINK_MSG_ID_TIMESYNC: {
					mavlink_handle_timesync( port, &msg, &status );
					break;
				}
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
	uint32_t time_start_read = system_micros();

	//Read in as many byts as we can until either
	//both ports are empty, have both read messages,
	//or the time_read_max is hit
	while( (system_micros() - time_start_read ) < time_read_max ) {
		bool port0_done = !comms_is_open( COMM_PORT_0 );
		bool port1_done = !comms_is_open( COMM_PORT_1 );

		if( !port0_done ) {
			if( comms_waiting( COMM_PORT_0 ) ) {
					//port0_done = communication_decode( MAVLINK_COMM_0, serialRead(Serial1) );
					port0_done = communication_decode( MAVLINK_COMM_0, comms_recv(COMM_PORT_0) );
			} else {
				port0_done = true;
			}
		}

		if( !port1_done ) {
			if( comms_waiting( COMM_PORT_1 ) ) {
					//port1_done = communication_decode( MAVLINK_COMM_1, serialRead(Serial2) );
					port1_done = communication_decode( MAVLINK_COMM_1, comms_recv(COMM_PORT_1) );
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
