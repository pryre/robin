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

//Default stream update rates, same layout as defined in mavlink_stream_id_t
static const float default_stream_rate_lookup_b921600[MAVLINK_STREAM_COUNT] = {
	1.0,	//MAVLINK_STREAM_ID_HEARTBEAT
	0.0,	//MAVLINK_STREAM_ID_STATUS_IO
	0.2,	//MAVLINK_STREAM_ID_SYS_STATUS
	50.0,	//MAVLINK_STREAM_ID_HIGHRES_IMU
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE
	50.0,	//MAVLINK_STREAM_ID_ATTITUDE_QUATERNION
	10.0,	//MAVLINK_STREAM_ID_ATTITUDE_TARGET
	20.0,	//MAVLINK_STREAM_ID_RC_CHANNELS_RAW
	10.0,	//MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW
	0.0,	//MAVLINK_STREAM_ID_TIMESYNC
	2.0,	//MAVLINK_STREAM_ID_BATTERY_STATUS
	100.0	//MAVLINK_STREAM_ID_LOW_PRIORITY
};

static const float default_stream_rate_lookup_b115200[MAVLINK_STREAM_COUNT] = {
	1.0,	//MAVLINK_STREAM_ID_HEARTBEAT
	0.0,	//MAVLINK_STREAM_ID_STATUS_IO
	0.2,	//MAVLINK_STREAM_ID_SYS_STATUS
	10.0,	//MAVLINK_STREAM_ID_HIGHRES_IMU
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE
	10.0,	//MAVLINK_STREAM_ID_ATTITUDE_QUATERNION
	5.0,	//MAVLINK_STREAM_ID_ATTITUDE_TARGET
	5.0,	//MAVLINK_STREAM_ID_RC_CHANNELS_RAW
	5.0,	//MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW
	0.0,	//MAVLINK_STREAM_ID_TIMESYNC
	2.0,	//MAVLINK_STREAM_ID_BATTERY_STATUS
	40.0	//MAVLINK_STREAM_ID_LOW_PRIORITY
};

static const float default_stream_rate_lookup_b57600[MAVLINK_STREAM_COUNT] = {
	1.0,	//MAVLINK_STREAM_ID_HEARTBEAT
	0.0,	//MAVLINK_STREAM_ID_STATUS_IO
	0.2,	//MAVLINK_STREAM_ID_SYS_STATUS
	5.0,	//MAVLINK_STREAM_ID_HIGHRES_IMU
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE
	5.0,	//MAVLINK_STREAM_ID_ATTITUDE_QUATERNION
	5.0,	//MAVLINK_STREAM_ID_ATTITUDE_TARGET
	5.0,	//MAVLINK_STREAM_ID_RC_CHANNELS_RAW
	5.0,	//MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW
	0.0,	//MAVLINK_STREAM_ID_TIMESYNC
	2.0,	//MAVLINK_STREAM_ID_BATTERY_STATUS
	20.0	//MAVLINK_STREAM_ID_LOW_PRIORITY
};

static const float default_stream_rate_lookup_minimal[MAVLINK_STREAM_COUNT] = {
	1.0,	//MAVLINK_STREAM_ID_HEARTBEAT
	0.0,	//MAVLINK_STREAM_ID_STATUS_IO
	0.2,	//MAVLINK_STREAM_ID_SYS_STATUS
	0.0,	//MAVLINK_STREAM_ID_HIGHRES_IMU
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE_QUATERNION
	0.0,	//MAVLINK_STREAM_ID_ATTITUDE_TARGET
	0.0,	//MAVLINK_STREAM_ID_RC_CHANNELS_RAW
	0.0,	//MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW
	0.0,	//MAVLINK_STREAM_ID_TIMESYNC
	0.0,	//MAVLINK_STREAM_ID_BATTERY_STATUS
	10.0	//MAVLINK_STREAM_ID_LOW_PRIORITY
};

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

static uint32_t calc_period(fix16_t rate) {
	uint32_t period = 0;

	//If we were passed a valid rate
	if(rate > 0)
		period = 1000 * fix16_to_int( fix16_div(_fc_1000, rate));	//XXX: Split the multiply/divs of 1000 to make sure we keep precision

	return period;
}

void communication_calc_period_update(uint8_t comm_port, mavlink_stream_id_t stream_id) {
	uint32_t baud = 0;
	mavlink_stream_t (*lookup)[];
	lookup = NULL;

	//Determine the comm port map to use
	//and lookup details for auto-rate settings
	if(comm_port == COMM_PORT_0) {
		baud = get_param_uint(PARAM_BAUD_RATE_0);
		lookup = &mavlink_stream_comm_0;
	} else if(comm_port == COMM_PORT_1) {
		baud = get_param_uint(PARAM_BAUD_RATE_1);
		lookup = &mavlink_stream_comm_1;
	}

	//If we were passed a valid comm port and slight bit of protection
	if( (lookup != NULL) && (stream_id < MAVLINK_STREAM_COUNT) ) {
		//Check the parameter to see what the user wants
		fix16_t rate = get_param_fix16((*lookup)[stream_id].param_rate);

		if(rate == 0) {
			//First see if we should disable the stream
			(*lookup)[stream_id].period_update = 0;
		} else if(rate == -_fc_1) {
			//Next we see if we should use the predefined "automatic" settings
			fix16_t auto_rate = 0;

			//Skim through out options from fastest to slowest
			//to find the best match we can
			if(baud >= 921600) {
				auto_rate = fix16_from_float( default_stream_rate_lookup_b921600[stream_id] );
			} else if(baud >= 115200) {
				auto_rate = fix16_from_float( default_stream_rate_lookup_b115200[stream_id] );
			} else if(baud >= 57600) {
				auto_rate = fix16_from_float( default_stream_rate_lookup_b57600[stream_id] );
			} else {
				//In this case, just get a set of params for core functionallity
				auto_rate = fix16_from_float( default_stream_rate_lookup_minimal[stream_id] );
			}

			(*lookup)[stream_id].period_update = calc_period(auto_rate);
		} else {
			//Otherwise the user has specified a rate, so lets use that
			(*lookup)[stream_id].period_update = calc_period(rate);
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
