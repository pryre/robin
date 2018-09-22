#pragma once

#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

//This will contain functions to receive and parse mavlink messages
//and should also hanlde any unsupported commands

void communication_receive(void);

void mavlink_handle_att_pos_mocap( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_heartbeat( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_hil_sensor( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_command_long( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_mission_request_list( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_param_request_list( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_param_request_read( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_param_set( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_set_actuator_control_target( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_set_attitude_target( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_set_mode( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );
void mavlink_handle_timesync( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status );

//Command handlers
#define MAV_RESULT_NO_ACK	0xFF
#define CMD_LONG_NUM_PARAMS	7

MAV_RESULT mavlink_handle_command_long_preflight_calibration( uint8_t port, float *params, uint8_t sysid, uint8_t compid );
MAV_RESULT mavlink_handle_command_long_set_mode( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_do_motor_test( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_request_protocol_version( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_request_autopilot_capabilities( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_get_home_position( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_preflight_storage( uint8_t port, float *params );
MAV_RESULT mavlink_handle_command_long_reboot_shutdown( uint8_t port, float *params, uint8_t sysid, uint8_t compid );
MAV_RESULT mavlink_handle_command_long_component_arm_disarm( uint8_t port, float *params );
