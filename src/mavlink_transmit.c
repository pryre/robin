#include "mavlink_transmit.h"
#include "mavlink_system.h"
#include "breezystm32.h"

#include <stdbool.h>
#include <stdint.h>

#include "params.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

//This will contain functions to build and transmit mavlink messages
//as well as a listing of utility functions to send each supported message type

// local function definitions
/*
void mavlink_send_heartbeat(void) {
  MAV_MODE armed_mode = MAV_MODE_ENUM_END; // used for failsafe
  if(_armed_state == ARMED)
    armed_mode = MAV_MODE_MANUAL_ARMED;
  else if(_armed_state == DISARMED)
    armed_mode = MAV_MODE_MANUAL_DISARMED;

  uint8_t control_mode = 0;
  if(get_param_int(PARAM_FIXED_WING))
  {
    control_mode = MODE_PASS_THROUGH;
  }
  else if(rc_switch(get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL)))
  {
    control_mode = MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
  }
  else
  {
    control_mode = rc_switch(get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? MODE_ROLL_PITCH_YAWRATE_THROTTLE : MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  }

  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             get_param_int(PARAM_FIXED_WING) ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_GENERIC,
                             armed_mode,
                             control_mode,
                             MAV_STATE_STANDBY);
}

static void mavlink_send_attitude(void)
{
  mavlink_msg_attitude_send(MAVLINK_COMM_0,
                            millis(),
                            _current_state.phi,
                            _current_state.theta,
                            _current_state.psi,
                            _current_state.p,
                            _current_state.q,
                            _current_state.r);
}

static void mavlink_send_imu(void)
{
  if (get_param_int(PARAM_STREAM_ADJUSTED_GYRO))
  {
    mavlink_msg_small_imu_send(MAVLINK_COMM_0,
                               _imu_time,
                               _accel.x,
                               _accel.y,
                               _accel.z,
                               _gyro.x - _adaptive_gyro_bias.x,
                               _gyro.y - _adaptive_gyro_bias.y,
                               _gyro.z - _adaptive_gyro_bias.z,
                               _imu_temperature);
  }
  else
  {
    mavlink_msg_small_imu_send(MAVLINK_COMM_0,
                               _imu_time,
                               _accel.x,
                               _accel.y,
                               _accel.z,
                               _gyro.x,
                               _gyro.y,
                               _gyro.z,
                               _imu_temperature);
  }
}

static void mavlink_send_servo_output_raw(void)
{
  mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0,
                                    micros(),
                                    0,
                                    _outputs[0],
                                    _outputs[1],
                                    _outputs[2],
                                    _outputs[3],
                                    _outputs[4],
                                    _outputs[5],
                                    _outputs[6],
                                    _outputs[7]);
}

static void mavlink_send_rc_raw(void)
{
  mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
                               millis(),
                               0,
                               pwmRead(0),
                               pwmRead(1),
                               pwmRead(2),
                               pwmRead(3),
                               pwmRead(4),
                               pwmRead(5),
                               pwmRead(6),
                               pwmRead(7),
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
}

static void mavlink_send_diff_pressure(void)
{
  if (_diff_pressure_present)
  {
    mavlink_msg_diff_pressure_send(MAVLINK_COMM_0, _diff_pressure, _temperature);
  }
}

static void mavlink_send_baro(void)
{
  if (_baro_present)
  {
    mavlink_msg_small_baro_send(MAVLINK_COMM_0, _baro_pressure, _temperature);
  }
}

static void mavlink_send_sonar(void)
{
  if (_sonar_present)
  {
    mavlink_msg_distance_sensor_send(MAVLINK_COMM_0,
                                     _sonar_time,
                                     24,
                                     822,
                                     _sonar_range,
                                     MAV_DISTANCE_SENSOR_ULTRASOUND,
                                     1,
                                     MAV_SENSOR_ROTATION_PITCH_180,
                                     1);
  }
}

static void mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}*/

// local variable definitions
int32_t _request_all_params;
mavlink_queue_t _low_priority_queue;

static void mavlink_transmit_low_priority(uint8_t port) {
	//TODO: Should be a better place for this
	if(_request_all_params >= 0) {
		mavlink_message_t msg;
		mavlink_prepare_param_value(&msg, _request_all_params);

		//Don't flood the buffer
		if(lpq_queue_msg(MAVLINK_COMM_0, &msg)) {	//TODO: should be for the port that requested params, not this port
			_request_all_params++;

			if(_request_all_params >= PARAMS_COUNT) {
				_request_all_params = -1;
			}
		}
	}

	//If there are messages in the queue
	if(_low_priority_queue.queued_message_count > 0) {
		//Transmit the message
		uint16_t i;
		uint16_t buffer_len = _low_priority_queue.buffer_len[_low_priority_queue.queue_position];
		uint8_t buffer_port = _low_priority_queue.buffer_port[_low_priority_queue.queue_position];

		for (i = 0; i < buffer_len; i++) {
			comm_send_ch(buffer_port, _low_priority_queue.buffer[_low_priority_queue.queue_position][i]);
		}

		//Move the queue along
		remove_current_lpq_message();
	}
}

//TODO: Individual LPQ streams
//Stream rate in microseconds: 1s = 1,000,000ms
static mavlink_stream_t mavlink_stream_comm_0[MAVLINK_STREAM_COUNT] = {
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_transmit_low_priority }
};

static mavlink_stream_t mavlink_stream_comm_1[MAVLINK_STREAM_COUNT] = {
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_highres_imu },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude_quaternion },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_attitude_target },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_servo_output_raw },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_stream_timesync },
	{ .period_us = 0, .last_time_us = 0, .send_function = mavlink_transmit_low_priority }
};

void communication_streams_init(void) {
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_HEARTBEAT].period_us = get_param_int(PARAM_STREAM_RATE_HEARTBEAT_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_SYS_STATUS].period_us = get_param_int(PARAM_STREAM_RATE_SYS_STATUS_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_HIGHRES_IMU].period_us = get_param_int(PARAM_STREAM_RATE_HIGHRES_IMU_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_ATTITUDE].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_ATTITUDE_QUATERNION].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_ATTITUDE_TARGET].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_TARGET_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW].period_us = get_param_int(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_TIMESYNC].period_us = get_param_int(PARAM_STREAM_RATE_TIMESYNC_0);
	mavlink_stream_comm_0[MAVLINK_STREAM_ID_LOW_PRIORITY].period_us = get_param_int(PARAM_STREAM_RATE_LOW_PRIORITY_0);

	mavlink_stream_comm_1[MAVLINK_STREAM_ID_HEARTBEAT].period_us = get_param_int(PARAM_STREAM_RATE_HEARTBEAT_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_SYS_STATUS].period_us = get_param_int(PARAM_STREAM_RATE_SYS_STATUS_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_HIGHRES_IMU].period_us = get_param_int(PARAM_STREAM_RATE_HIGHRES_IMU_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_ATTITUDE].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_ATTITUDE_QUATERNION].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_ATTITUDE_TARGET].period_us = get_param_int(PARAM_STREAM_RATE_ATTITUDE_TARGET_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW].period_us = get_param_int(PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_TIMESYNC].period_us = get_param_int(PARAM_STREAM_RATE_TIMESYNC_1);
	mavlink_stream_comm_1[MAVLINK_STREAM_ID_LOW_PRIORITY].period_us = get_param_int(PARAM_STREAM_RATE_LOW_PRIORITY_1);
}

static bool transmit_stream(uint32_t time_us, uint8_t port, mavlink_stream_t *stream) {
	bool sent_message = false;

	if( (stream->period_us > 0) && (time_us >= ( stream->last_time_us + stream->period_us ) ) ) {
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
	bool message_sent_comm_1 = ( get_param_int(PARAM_BAUD_RATE_0) != 0 );

	for (int i = 0; i < MAVLINK_STREAM_COUNT; i++) {

		if (!message_sent_comm_0)
			message_sent_comm_0 = transmit_stream(time_us, MAVLINK_COMM_0, &(mavlink_stream_comm_0[i]));

		if (!message_sent_comm_1)
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
