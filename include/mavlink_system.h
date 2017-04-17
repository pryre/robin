#pragma once

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define LOW_PRIORITY_QUEUE_SIZE 16

#include "breezystm32.h"
#include "serial.h"
#include "serial_uart.h"

#include "mavlink/mavlink_types.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

extern serialPort_t * Serial2;
extern mavlink_system_t mavlink_system;

void communications_init(void);

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#include "mavlink/common/mavlink.h"

//==-- On-demand messages
void mavlink_send_statustext_notice(uint8_t port, uint8_t severity, char* text);
void mavlink_send_timesync(uint8_t port, uint64_t tc1, uint64_t ts1);

//==-- Low priority message queue
typedef struct {
	uint8_t port;
	int32_t request_all_params;

	uint8_t buffer[LOW_PRIORITY_QUEUE_SIZE][MAVLINK_MAX_PACKET_LEN];
	uint16_t buffer_len[LOW_PRIORITY_QUEUE_SIZE];
	uint16_t position;	//Current position in the queue
	uint16_t length;	//Current length of the queue

	uint32_t timer_warn_full;
} mavlink_queue_t;

extern mavlink_queue_t _lpq_port_0;
extern mavlink_queue_t _lpq_port_1;

bool lpq_queue_msg(uint8_t port, mavlink_message_t *msg);

//==-- Streams
void mavlink_stream_low_priority(uint8_t port);
void mavlink_stream_heartbeat(uint8_t port);
void mavlink_stream_sys_status(uint8_t port);
void mavlink_stream_highres_imu(uint8_t port);
void mavlink_stream_attitude(uint8_t port);
void mavlink_stream_attitude_quaternion(uint8_t port);
void mavlink_stream_attitude_target(uint8_t port);
void mavlink_stream_servo_output_raw(uint8_t port);
void mavlink_stream_timesync(uint8_t port);

//==-- Low Priority Messages
void mavlink_prepare_autopilot_version(mavlink_message_t *msg);
void mavlink_prepare_command_ack(mavlink_message_t *msg, uint16_t command, uint8_t result);
void mavlink_prepare_param_value(mavlink_message_t *msg, uint32_t index);
void mavlink_prepare_statustext(mavlink_message_t *msg, uint8_t severity, char* text);
void mavlink_prepare_debug(mavlink_message_t *msg, uint32_t stamp, uint8_t index, uint32_t value);

//==-- Utility Messages
bool mavlink_queue_notice_broadcast(char* text);

//==-- List of supported mavlink messages
	//o Optional
	//- Needed
	//+ Done!

	//Receive
	//- command_int.h
	//- command_long.h
	//- heartbeat.h
	//- manual_setpoint.h
	//- message_interval.h
	//+ param_set.h
	//+ param_request_list.h
	//+ param_request_read.h
	//- ping.h
	//- raw_imu.h
	//- raw_pressure.h
	//- set_attitude_target.h
	//- timesync.h
	//o att_pos_mocap.h
	//o power_status.h
	//o radio_status.h

	//Transmit
		//Stream
			//+ attitude_target.h
			//- attitude_quaternion_cov.h
			//o battery_status.h
			//- distance_sensor.h
			//+ heartbeat.h
			//+ highres_imu.h
			//- scaled_pressure.h
			//+ servo_output_raw.h
			//+ sys_status.h
			//+ attitude.h
			//+ attitude_quaternion.h
			//o optical_flow.h
			//o optical_flow_rad.h

		//High Priority
			//- ping.h
			//- timesync.h
			//- command_ack.h

		//Low Priority
			//+ debug.h					--	3 params	--	9 bytes
			//- debug_vect.h			--	14 params	--	30 bytes
			//+ autopilot_version.h		--	16 params	--	60 bytes
			//- named_value_float.h		--	12 params	--	18 bytes
			//- named_value_int.h		--	12 params	--	18 bytes
			//+ param_value.h			--	20 params	--	25 bytes

