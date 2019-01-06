#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"

//This will contain functions to receive and parse mavlink messages
//and should also hanlde any unsupported commands

void communication_receive( void );

void mavlink_handle_att_pos_mocap( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_heartbeat( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_hil_sensor( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_command_long( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_mission_request_list( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_param_request_list( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_param_request_read( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_param_set( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_set_actuator_control_target( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_set_attitude_target( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_set_mode( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );
void mavlink_handle_timesync( mavlink_channel_t chan, mavlink_message_t* msg, mavlink_status_t* status );

//Command handlers
#define MAV_RESULT_NO_ACK 0xFF
#define CMD_LONG_NUM_PARAMS 7

MAV_RESULT mavlink_handle_command_long_preflight_calibration( mavlink_channel_t chan, float* params, uint8_t sysid, uint8_t compid );
MAV_RESULT mavlink_handle_command_long_set_mode( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_do_motor_test( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_request_protocol_version( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_request_autopilot_capabilities( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_get_home_position( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_preflight_storage( mavlink_channel_t chan, float* params );
MAV_RESULT mavlink_handle_command_long_reboot_shutdown( mavlink_channel_t chan, float* params, uint8_t sysid, uint8_t compid );
MAV_RESULT mavlink_handle_command_long_component_arm_disarm( mavlink_channel_t chan, float* params );

#ifdef __cplusplus
}
#endif
