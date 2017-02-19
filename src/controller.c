#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "breezystm32/breezystm32.h"
#include "fix16.h"

#include "params.h"
#include "mixer.h"
#include "estimator.h"
#include "safety.h"
#include "controller.h"

#include "mavlink_system.h"

static uint32_t counter = 0;

void controller_init() {
  pid_init(&pid_roll,
           PARAM_PID_ROLL_ANGLE_P,
           PARAM_PID_ROLL_ANGLE_I,
           PARAM_PID_ROLL_ANGLE_D,
           _current_state.phi,
           _current_state.p,
           _combined_control.x.value,
           _command.x,
           get_param_int(PARAM_MAX_COMMAND)/2.0f,
           -1.0f*get_param_int(PARAM_MAX_COMMAND)/2.0f);

  pid_init(&pid_pitch,
           PARAM_PID_PITCH_ANGLE_P,
           PARAM_PID_PITCH_ANGLE_I,
           PARAM_PID_PITCH_ANGLE_D,
           &_current_state.theta,
           &_current_state.q,
           &_combined_control.y.value,
           &_command.y,
           get_param_int(PARAM_MAX_COMMAND)/2.0f,
           -1.0f*get_param_int(PARAM_MAX_COMMAND)/2.0f);

  pid_init(&pid_roll_rate,
           PARAM_PID_ROLL_RATE_P,
           PARAM_PID_ROLL_RATE_I,
           PARAMS_COUNT,
           &_current_state.p,
           NULL,
           &_combined_control.x.value,
           &_command.x,
           get_param_int(PARAM_MAX_COMMAND)/2.0f,
           -1.0f*get_param_int(PARAM_MAX_COMMAND)/2.0f);

  pid_init(&pid_pitch_rate,
           PARAM_PID_PITCH_RATE_P,
           PARAM_PID_PITCH_RATE_I,
           PARAMS_COUNT,
           &_current_state.q,
           NULL,
           &_combined_control.y.value,
           &_command.y,
           get_param_int(PARAM_MAX_COMMAND)/2.0f,
           -1.0f*get_param_int(PARAM_MAX_COMMAND)/2.0f);

  pid_init(&pid_yaw_rate,
           PARAM_PID_YAW_RATE_P,
           PARAM_PID_YAW_RATE_I,
           PARAMS_COUNT,
           &_current_state.r,
           NULL,
           &_combined_control.z.value,
           &_command.z,
           get_param_int(PARAM_MAX_COMMAND)/2.0f,
           -1.0f*get_param_int(PARAM_MAX_COMMAND)/2.0f);

  pid_init(&pid_altitude,
           PARAM_PID_ALT_P,
           PARAM_PID_ALT_I,
           PARAM_PID_ALT_D,
           &_current_state.altitude,
           NULL,
           &_combined_control.F.value,
           &_command.F,
           get_param_int(PARAM_MAX_COMMAND),
           0.0f);
}


void controller_run()
{
	//Roll
	if(_combined_control.x.type == RATE) {
		run_pid(&pid_roll_rate);
	} else if(_combined_control.x.type == ANGLE) {
		run_pid(&pid_roll);
	} else {	//Passthrough
		_command.x = _combined_control.x.value;
	}

	//Pitch
	if(_combined_control.y.type == RATE) {
		run_pid(&pid_pitch_rate);
	} else if(_combined_control.y.type == ANGLE) {
		run_pid(&pid_pitch);
	} else {	//Passthrough
		_command.y = _combined_control.y.value;
	}

	//Yaw
	if(_combined_control.z.type == RATE) {
		run_pid(&pid_yaw_rate);
	} else {	//Passthrough
		_command.z = _combined_control.z.value;
	}

	//Throttle
	if(_combined_control.F.type == ALTITUDE) {
		run_pid(&pid_altitude);
	} else {	//Passthrough
		_command.F = _combined_control.F.value;
	}

	if(counter > 100) {
		mavlink_send_named_command_struct("RC", _rc_control);
		mavlink_send_named_command_struct("offboard", _offboard_control);
		mavlink_send_named_command_struct("combined", _combined_control);
		mavlink_send_named_value_float("command_F", _command.F);
		mavlink_send_named_value_float("command_x", _command.x);
		mavlink_send_named_value_float("command_y", _command.y);
		mavlink_send_named_value_float("command_z", _command.z);
		counter = 0;
	}

	counter++;
}
