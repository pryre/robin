#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_transmit.h"
#include "mavlink_system.h"
#include "params.h"
#include "fixextra.h"
#include "safety.h"

#include <stdbool.h>
#include <stdint.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

system_status_t _system_status;

//Stream rate in microseconds: 1s = 1,000,000ms
static mavlink_stream_t mavlink_stream_comm_0[MAVLINK_STREAM_COUNT] = {
	{ .param_rate = PARAM_STREAM_RATE_HEARTBEAT_0,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .param_rate = PARAM_STREAM_RATE_STATUS_IO_0,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_status_io },
	{ .param_rate = PARAM_STREAM_RATE_SYS_STATUS_0,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .param_rate = PARAM_STREAM_RATE_HIGHRES_IMU_0,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_0,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0,	.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_TARGET_0,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .param_rate = PARAM_STREAM_RATE_RC_CHANNELS_RAW_0,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_rc_channels_raw },
	{ .param_rate = PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .param_rate = PARAM_STREAM_RATE_TIMESYNC_0,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .param_rate = PARAM_STREAM_RATE_BATTERY_STATUS_0,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_battery_status },
	{ .param_rate = PARAM_STREAM_RATE_LOW_PRIORITY_0,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_low_priority }
};
static mavlink_stream_t mavlink_stream_comm_1[MAVLINK_STREAM_COUNT] = {
	{ .param_rate = PARAM_STREAM_RATE_HEARTBEAT_1,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .param_rate = PARAM_STREAM_RATE_STATUS_IO_1,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_status_io },
	{ .param_rate = PARAM_STREAM_RATE_SYS_STATUS_1,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .param_rate = PARAM_STREAM_RATE_HIGHRES_IMU_1,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_1,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1,	.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .param_rate = PARAM_STREAM_RATE_ATTITUDE_TARGET_1,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .param_rate = PARAM_STREAM_RATE_RC_CHANNELS_RAW_1,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_rc_channels_raw },
	{ .param_rate = PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1,		.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .param_rate = PARAM_STREAM_RATE_TIMESYNC_1,				.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .param_rate = PARAM_STREAM_RATE_BATTERY_STATUS_1,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_battery_status },
	{ .param_rate = PARAM_STREAM_RATE_LOW_PRIORITY_1,			.period_update = 0,	.last_time_us = 0, .send_function = mavlink_stream_low_priority }
};

void communication_calc_period_update(uint8_t comm_port, mavlink_stream_id_t stream_id) {
	if(comm_port == COMM_PORT_0) {
		fix16_t rate = get_param_fix16(mavlink_stream_comm_0[stream_id].param_rate);

		if(rate == 0) {
			mavlink_stream_comm_0[stream_id].period_update = 0;
		} else {
			uint32_t val = fix16_to_int( fix16_div(_fc_1000, rate));
			mavlink_stream_comm_0[stream_id].period_update = 1000 * val;
		}
	} else if(comm_port == COMM_PORT_1) {
		fix16_t rate = get_param_fix16(mavlink_stream_comm_1[stream_id].param_rate);

		if(rate == 0) {
			mavlink_stream_comm_1[stream_id].period_update = 0;
		} else {
			uint32_t val = fix16_to_int( fix16_div(_fc_1000, rate));
			mavlink_stream_comm_1[stream_id].period_update = 1000 * val;
		}
	}
}

static bool transmit_stream(uint32_t time_us, uint8_t port, mavlink_stream_t *stream) {
	bool sent_message = false;

	if( ( stream->period_update > 0) && ( ( time_us - stream->last_time_us) > stream->period_update ) ) {
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
	bool message_sent_comm_0 = !comms_is_open( COMM_PORT_0 );
	bool message_sent_comm_1 = !comms_is_open( COMM_PORT_1 );

	for (int i = 0; i < MAVLINK_STREAM_COUNT; i++) {

		if( !message_sent_comm_0 )
			message_sent_comm_0 = transmit_stream(time_us, MAVLINK_COMM_0, &(mavlink_stream_comm_0[i]));

		if( !message_sent_comm_1 )
			message_sent_comm_1 = transmit_stream(time_us, MAVLINK_COMM_1, &(mavlink_stream_comm_1[i]));

		//Break early if neither device will transmit again
		if(message_sent_comm_0 && message_sent_comm_1)
			break;
	}
}

#ifdef __cplusplus
}
#endif
