#include "mavlink_transmit.h"
#include "mavlink_system.h"
#include "breezystm32.h"

#include <stdbool.h>
#include <stdint.h>

#include "params.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// local variable definitions
int32_t _request_all_params_port_0;
int32_t _request_all_params_port_1;
mavlink_queue_t _low_priority_queue;

static void lpq_queue_all_params(uint8_t port, int32_t request_all_params) {
	if( request_all_params >= 0 ) {
		mavlink_message_t msg;
		mavlink_prepare_param_value(&msg, request_all_params);

		//Don't flood the buffer
		if(lpq_queue_msg(port, &msg)) {
			request_all_params++;

			if(request_all_params >= PARAMS_COUNT) {
				request_all_params = -1;
			}
		}
	}
}

static void mavlink_transmit_low_priority(uint8_t port) {

	if(port == MAVLINK_COMM_0)
		lpq_queue_all_params(port, _request_all_params_port_0);

	if(port == MAVLINK_COMM_1)
		lpq_queue_all_params(port, _request_all_params_port_1);

	//If there are messages in the queue
	if(_low_priority_queue.queued_message_count > 0) {
		//Transmit the message
		uint16_t buffer_len = _low_priority_queue.buffer_len[_low_priority_queue.queue_position];
		uint8_t buffer_port = _low_priority_queue.buffer_port[_low_priority_queue.queue_position];

		if( port == buffer_port ) {
			for(uint16_t i = 0; i < buffer_len; i++) {
				comm_send_ch(buffer_port, _low_priority_queue.buffer[_low_priority_queue.queue_position][i]);
			}

			//Move the queue along
			remove_current_lpq_message();
		}
	}
}

//Stream rate in microseconds: 1s = 1,000,000ms
static mavlink_stream_t mavlink_stream_comm_0[MAVLINK_STREAM_COUNT] = {
	{ .period_param = PARAM_STREAM_RATE_HEARTBEAT_0,			.last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .period_param = PARAM_STREAM_RATE_SYS_STATUS_0,			.last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .period_param = PARAM_STREAM_RATE_HIGHRES_IMU_0,			.last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_0,				.last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0,	.last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_TARGET_0,		.last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .period_param = PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0,		.last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .period_param = PARAM_STREAM_RATE_TIMESYNC_0,				.last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .period_param = PARAM_STREAM_RATE_LOW_PRIORITY_0,			.last_time_us = 0, .send_function = mavlink_transmit_low_priority }
};

static mavlink_stream_t mavlink_stream_comm_1[MAVLINK_STREAM_COUNT] = {
	{ .period_param = PARAM_STREAM_RATE_HEARTBEAT_1,			.last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .period_param = PARAM_STREAM_RATE_SYS_STATUS_1,			.last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .period_param = PARAM_STREAM_RATE_HIGHRES_IMU_1,			.last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_1,				.last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1,	.last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .period_param = PARAM_STREAM_RATE_ATTITUDE_TARGET_1,		.last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .period_param = PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1,		.last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .period_param = PARAM_STREAM_RATE_TIMESYNC_1,				.last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .period_param = PARAM_STREAM_RATE_LOW_PRIORITY_1,			.last_time_us = 0, .send_function = mavlink_transmit_low_priority }
};

static bool transmit_stream(uint32_t time_us, uint8_t port, mavlink_stream_t *stream) {
	bool sent_message = false;

	if( ( get_param_int(stream->period_param) > 0) && ( ( time_us - stream->last_time_us) > (uint32_t)get_param_int(stream->period_param) ) ) {
		stream->send_function(port);
		stream->last_time_us = time_us;

		sent_message = true;
	}

	return sent_message;
}

void communication_transmit(uint32_t time_us) {
	//We only want to send 1 message each loop (per port),
	// otherwise we risk overloading the serial buffer. This
	// will also offset the message streams so they are all staggered
	//Disable checking for outputs if port disabled
	bool message_sent_comm_0 = ( get_param_int(PARAM_BAUD_RATE_0) != 0 );
	bool message_sent_comm_1 = ( get_param_int(PARAM_BAUD_RATE_1) != 0 );

	for (int i = 0; i < MAVLINK_STREAM_COUNT; i++) {

		if( !message_sent_comm_0 )
			message_sent_comm_0 = transmit_stream(time_us, MAVLINK_COMM_0, &(mavlink_stream_comm_0[i]));

		if( !message_sent_comm_1 )
			message_sent_comm_1 = transmit_stream(time_us, MAVLINK_COMM_1, &(mavlink_stream_comm_1[i]));

		if(message_sent_comm_0 && message_sent_comm_1)
			break;
	}
}

/*
void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate)
{
  mavlink_streams[stream_id].period_us = (rate == 0 ? 0 : 1000000/rate);
}

void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us)
{
  mavlink_streams[stream_id].period_us = period_us;
}
*/
