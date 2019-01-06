#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

//XXX: LPQ was 12
#define LOW_PRIORITY_QUEUE_SIZE 8
#define LOW_PRIORITY_QUEUE_PARAMS_SIZE 32

//#include "breezystm32.h"
//#include "serial.h"
//#include "serial_uart.h"

#include "drivers/drv_comms.h"
#include "mavlink/mavlink_types.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

#define MAVLINK_VERSION_MIN 100
#define MAVLINK_VERSION_MAX 200

extern mavlink_system_t mavlink_system;
extern mavlink_system_t mavlink_gcs;
extern bool _ch_0_have_heartbeat;
extern bool _ch_1_have_heartbeat;

void communications_system_init( void );

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void comm_send_ch( mavlink_channel_t chan, uint8_t ch );

mavlink_message_t* get_channel_buf( mavlink_channel_t chan );
void comms_send_msg( mavlink_channel_t chan );

#include "mavlink/common/mavlink.h"

//==-- On-demand messages
void mavlink_send_statustext( mavlink_channel_t chan, uint8_t severity, char* text );
void mavlink_send_broadcast_statustext( uint8_t severity, char* text );
void mavlink_send_timesync( mavlink_channel_t chan, uint64_t tc1, uint64_t ts1 );

//==-- Low priority message queue
typedef struct {
	uint8_t buffer[LOW_PRIORITY_QUEUE_SIZE][MAVLINK_MAX_PACKET_LEN]; //List of buffered messages
	uint16_t buffer_len[LOW_PRIORITY_QUEUE_SIZE];					 //Lengths of buffered messages
	uint8_t buffer_port[LOW_PRIORITY_QUEUE_SIZE];
	uint16_t position; //Current position in the queue
	uint16_t length;   //Current length of the queue						//List representing which comm port to send message to

	uint32_t param_buffer[LOW_PRIORITY_QUEUE_PARAMS_SIZE];
	uint8_t param_buffer_port[LOW_PRIORITY_QUEUE_PARAMS_SIZE];
	uint16_t param_position; //Current position in the queue
	uint16_t param_length;   //Current length of the queue
	int32_t request_all_params_port0;
	int32_t request_all_params_port1;

	uint32_t timer_warn_full;
	uint32_t timer_param_warn_full;
} mavlink_queue_t;

extern mavlink_queue_t _lpq;

bool lpq_queue_msg( mavlink_channel_t chan, mavlink_message_t* msg );
bool lpq_queue_param_broadcast( uint32_t index );
void lpq_queue_broadcast_msg( mavlink_message_t* msg );

//==-- Streams
bool mavlink_stream_ready( mavlink_channel_t chan );
void mavlink_stream_low_priority( mavlink_channel_t chan );
void mavlink_stream_heartbeat( mavlink_channel_t chan );
void mavlink_stream_status_io( mavlink_channel_t chan );
void mavlink_stream_sys_status( mavlink_channel_t chan );
void mavlink_stream_highres_imu( mavlink_channel_t chan );
void mavlink_stream_attitude( mavlink_channel_t chan );
void mavlink_stream_attitude_quaternion( mavlink_channel_t chan );
void mavlink_stream_attitude_target( mavlink_channel_t chan );
void mavlink_stream_rc_channels_raw( mavlink_channel_t chan );
void mavlink_stream_servo_output_raw( mavlink_channel_t chan );
void mavlink_stream_timesync( mavlink_channel_t chan );
void mavlink_stream_battery_status( mavlink_channel_t chan );

void mavlink_stream_broadcast_param_value( uint32_t index );

//==-- Low Priority Messages
void mavlink_queue_broadcast_info( char* text );
void mavlink_queue_broadcast_notice( char* text );
void mavlink_queue_broadcast_error( char* text );

void mavlink_prepare_protocol_version( mavlink_message_t* msg );
void mavlink_prepare_autopilot_version( mavlink_message_t* msg );

void mavlink_prepare_home_position( mavlink_message_t* msg );
//void mavlink_prepare_scaled_pressure(mavlink_message_t *msg);

void mavlink_prepare_command_ack( mavlink_message_t* msg, uint16_t command, uint8_t result, uint8_t sender_sysid, uint8_t sender_compid, uint8_t progress );
void mavlink_prepare_param_value( mavlink_message_t* msg, uint32_t index );
void mavlink_prepare_statustext( mavlink_message_t* msg, uint8_t severity, char* text );
void mavlink_prepare_debug( mavlink_message_t* msg, uint32_t stamp, uint8_t index, uint32_t value );

#ifdef __cplusplus
}
#endif
