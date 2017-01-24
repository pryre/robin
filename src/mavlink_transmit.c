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
bool request_all_params;
int32_t param_to_send;

static void mavlink_transmit_low_priority() {
	//bool sent_something = false; //TODO: Flag to set so only one message gets sent per loop

	//Params
	if(param_to_send >= 0) {
		mavlink_stream_param_value(param_to_send);

		if(request_all_params) {
			param_to_send++;

			if(param_to_send >= PARAMS_COUNT) {	//Kind of doubles up, but is faster if only 1 is requested
				request_all_params = false;
				param_to_send = -1;
			}
		} else {
			param_to_send = -1;
		}
	}
}

//TODO: Should IMU be sending this fast?
static mavlink_stream_t mavlink_streams[MAVLINK_STREAM_COUNT] = {
	{ .period_us = 1000000, .last_time_us = 0, .send_function = mavlink_stream_heartbeat },
	{ .period_us = 5000000, .last_time_us = 0, .send_function = mavlink_stream_sys_status },
	{ .period_us = 900, .last_time_us = 0, .send_function = mavlink_stream_highres_imu },	//Set at 900, not 1000 as we want it to do it as fast as possible, but not multiple times per loop
	/*
	{ .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_attitude },

	{ .period_us = 1000,    .last_time_us = 0, .send_function = mavlink_send_imu },
	{ .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_diff_pressure },
	{ .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_baro },
	{ .period_us = 100000,  .last_time_us = 0, .send_function = mavlink_send_sonar },

	{ .period_us = 0,       .last_time_us = 0, .send_function = mavlink_send_servo_output_raw },
	{ .period_us = 0,       .last_time_us = 0, .send_function = mavlink_send_rc_raw },*/
	{ .period_us = 10000,   .last_time_us = 0, .send_function = mavlink_transmit_low_priority }
};

//static void mavlink_send_stream(uint32_t time_us) {

//}

//This function will send out 1 packet of requested data at a time
//This is for data that isn't time-sensitive or may need a lot of processing per packet

// function definitions
void communication_transmit(uint32_t time_us) {
	for (int i = 0; i < MAVLINK_STREAM_COUNT; i++) {
		if (mavlink_streams[i].period_us && (time_us >= mavlink_streams[i].last_time_us + mavlink_streams[i].period_us)) {
			mavlink_streams[i].last_time_us = time_us;
			mavlink_streams[i].send_function();
		}
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
