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

state_t _state_estimator;
command_input_t _command_input;
control_output_t _control_output;

pid_t pid_roll_rate;
pid_t pid_pitch_rate;
pid_t pid_yaw_rate;
pid_t pid_roll;
pid_t pid_pitch;
pid_t pid_yaw;
//pid_t pid_altitude;

void controller_init() {
/*
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
*/
}


void controller_run(uint32_t time_now) {
	//Variables that store the computed attitude goal rates
	fix16_t goal_att_r = 0;
	fix16_t goal_att_p = 0;
	fix16_t goal_att_y = 0;
	fix16_t goal_auto_throttle = 0;

	_control_output.r = 0;
	_control_output.p = 0;
	_control_output.y = 0;
	_control_output.T = 0;

	//TODO: Could do something here for altitude hold mode if enabled

	//If we should listen to attitude input
	if( !(_command_input.input_mask & CMD_IN_IGNORE_ATTITUDE) ) {
		//METHOD:
			//DCM from _command_input.q
			//DCM from _state_estimator.q

			//Calculate shortest path to goal rotation without yaw (as it's slow)
			//Calculate remaining yaw error
			//Use different methods depending on how dramatic the change is

			//Calculate rate setpoints
			//Constrain rates to set params
			//Apply a yaw feed forward(?)

/*
//==== PX4 METHOD ====//
_thrust_sp = _v_att_sp.thrust;

// construct attitude setpoint rotation matrix
math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
math::Matrix<3, 3> R_sp = q_sp.to_dcm();

// get current rotation matrix from control state quaternions
math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
math::Matrix<3, 3> R = q_att.to_dcm();

// all input data is ready, run controller itself

// try to move thrust vector shortest way, because yaw response is slower than roll/pitch
math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

// axis and sin(angle) of desired rotation
math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

// calculate angle error
float e_R_z_sin = e_R.length();
float e_R_z_cos = R_z * R_sp_z;

// calculate weight for yaw control
float yaw_w = R_sp(2, 2) * R_sp(2, 2);

// calculate rotation matrix after roll/pitch only rotation
math::Matrix<3, 3> R_rp;

if (e_R_z_sin > 0.0f) {
	// get axis-angle representation
	float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
	math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

	e_R = e_R_z_axis * e_R_z_angle;

	// cross product matrix for e_R_axis
	math::Matrix<3, 3> e_R_cp;
	e_R_cp.zero();
	e_R_cp(0, 1) = -e_R_z_axis(2);
	e_R_cp(0, 2) = e_R_z_axis(1);
	e_R_cp(1, 0) = e_R_z_axis(2);
	e_R_cp(1, 2) = -e_R_z_axis(0);
	e_R_cp(2, 0) = -e_R_z_axis(1);
	e_R_cp(2, 1) = e_R_z_axis(0);

	// rotation matrix for roll/pitch only rotation
	R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

} else {
	// zero roll/pitch rotation
	R_rp = R;
}

//R_rp and R_sp has the same Z axis, calculate yaw error
math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

if (e_R_z_cos < 0.0f) {
	//for large thrust vector rotations use another rotation method:
	// calculate angle and axis for R -> R_sp rotation directly
	math::Quaternion q_error;
	q_error.from_dcm(R.transposed() * R_sp);
	math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

	// use fusion of Z axis based rotation and direct rotation
	float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
	e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
}

//calculate angular rates setpoint
_rates_sp = _params.att_p.emult(e_R);

//limit rates
for (int i = 0; i < 3; i++) {
	_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
}

//feed forward yaw setpoint rate
_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

//==== PX4 METHOD ====//
*/

		//Run PIDs for attitude
		//goal_att_r = ...();
		//goal_att_p = ...();
		//goal_att_y = ...();
	}

	//Roll Rate
	if( !(_command_input.input_mask & CMD_IN_IGNORE_ROLL_RATE) ) {
		//Run PID for roll rate with _command_input.r
	} else {	//Else use the computed roll rate goal
		//_control_output.r = ...(goal_att_r);
	}

	//Pitch Rate
	if( !(_command_input.input_mask & CMD_IN_IGNORE_PITCH_RATE) ) {
		//Run PID for pitch rate with _command_input.p
	} else {	//Else use the computed pitch rate goal
		//_control_output.p = ...(goal_att_p);
	}

	//Yaw Rate
	if( !(_command_input.input_mask & CMD_IN_IGNORE_YAW_RATE) ) {
		//Run PID for pitch rate with _command_input.y
	} else {	//Else use the computed yaw rate goal
		//_control_output.y = ...(goal_att_y);
	}

	//Trottle
	if( !(_command_input.input_mask & CMD_IN_IGNORE_THROTTLE) ) {
		_control_output.T = _command_input.T;
	} else {	//Else use the computed throttle goal
		_control_output.T = goal_auto_throttle;
	}
}
