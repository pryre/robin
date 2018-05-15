#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "breezystm32.h"
#include "pwm.h"
#include "fix16.h"
#include "fixvector3d.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixextra.h"

#include "params.h"
#include "safety.h"
#include "estimator.h"
#include "controller.h"
#include "pid_controller.h"

#include "mavlink_system.h"
#include "stdio.h"

system_status_t _system_status;
state_t _state_estimator;
command_input_t _cmd_ob_input;
command_input_t _cmd_rc_input;
command_input_t _control_input;
control_output_t _control_output;

pid_controller_t _pid_roll_rate;
pid_controller_t _pid_pitch_rate;
pid_controller_t _pid_yaw_rate;

void controller_reset(void) {
	pid_reset(&_pid_roll_rate, _state_estimator.p);
	pid_reset(&_pid_pitch_rate, _state_estimator.q);
	pid_reset(&_pid_yaw_rate, _state_estimator.r);


	_control_input.r = 0;
	_control_input.p = 0;
	_control_input.y = 0;
	_control_input.q.a = _fc_1;
	_control_input.q.b = 0;
	_control_input.q.c = 0;
	_control_input.q.d = 0;
	_control_input.T = 0;
	_cmd_ob_input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

	_control_output.r = 0;
	_control_output.p = 0;
	_control_output.y = 0;
	_control_output.T = 0;
}

void controller_init(void) {
	pid_init(&_pid_roll_rate,
			 get_param_fix16(PARAM_PID_ROLL_RATE_P),
			 get_param_fix16(PARAM_PID_ROLL_RATE_I),
			 get_param_fix16(PARAM_PID_ROLL_RATE_D),
			 get_param_fix16(PARAM_PID_TAU),
			 _state_estimator.p,
			 0, 0, 0, -_fc_1, _fc_1);	//XXX: Mixer input is normalized from -1 to 1

	pid_init(&_pid_pitch_rate,
			 get_param_fix16(PARAM_PID_PITCH_RATE_P),
			 get_param_fix16(PARAM_PID_PITCH_RATE_I),
			 get_param_fix16(PARAM_PID_PITCH_RATE_D),
			 get_param_fix16(PARAM_PID_TAU),
			 _state_estimator.q,
			 0, 0, 0, -_fc_1, _fc_1);	//XXX: Mixer input is normalized from -1 to 1

	pid_init(&_pid_yaw_rate,
			 get_param_fix16(PARAM_PID_YAW_RATE_P),
			 get_param_fix16(PARAM_PID_YAW_RATE_I),
			 get_param_fix16(PARAM_PID_YAW_RATE_D),
			 get_param_fix16(PARAM_PID_TAU),
			 _state_estimator.r,
			 0, 0, 0, -_fc_1, _fc_1);	//XXX: Mixer input is normalized from -1 to 1

	_cmd_ob_input.r = 0;
	_cmd_ob_input.p = 0;
	_cmd_ob_input.y = 0;
	_cmd_ob_input.q.a = _fc_1;
	_cmd_ob_input.q.b = 0;
	_cmd_ob_input.q.c = 0;
	_cmd_ob_input.q.d = 0;
	_cmd_ob_input.T = 0;
	_cmd_ob_input.input_mask |= CMD_IN_IGNORE_ATTITUDE;	//Set it to just hold rpy rates (as this skips unnessessary computing during boot, and is possibly safer)

	_control_output.r = 0;
	_control_output.p = 0;
	_control_output.y = 0;
	_control_output.T = 0;
}

static void controller_set_input_failsafe(command_input_t *input) {
	input->r = 0;
	input->p = 0;
	input->y = 0;
	input->q.a = _fc_1;
	input->q.b = 0;
	input->q.c = 0;
	input->q.d = 0;
	input->T = get_param_fix16(PARAM_FAILSAFE_THROTTLE);
	input->input_mask |= CMD_IN_IGNORE_ATTITUDE;	//Set it to just hold rpy rates (as this skips unnessessary computing during boot, and is possibly safer)
}

static v3d rate_goals_from_attitude(const qf16 *q_sp, const qf16 *q_current) {
		mf16 I;
		I.rows = 3;
		I.columns = 3;
		I.data[0][0] = _fc_1;
		I.data[1][1] = _fc_1;
		I.data[2][2] = _fc_1;

		//Method derived from px4 attitude controller:
		//DCM from for state and setpoint
		mf16 R_sp;
		qf16_to_matrix(&R_sp, q_sp);
		mf16 R;
		qf16_to_matrix(&R, q_current);

		//Calculate shortest path to goal rotation without yaw (as it's slower than roll/pitch)
		v3d R_z;
		v3d R_sp_z;
		R_z.x = R.data[0][2];
		R_z.y = R.data[1][2];
		R_z.z = R.data[2][2];
		R_sp_z.x = R_sp.data[0][2];
		R_sp_z.y = R_sp.data[1][2];
		R_sp_z.z = R_sp.data[2][2];

		//px4: axis and sin(angle) of desired rotation
		//px4: math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);
		mf16 Rt;
		mf16_transpose(&Rt, &R);

		v3d R_z_c;
		v3d_cross(&R_z_c, &R_z, &R_sp_z);
		mf16 R_z_c_m;
		R_z_c_m.rows = 3;
		R_z_c_m.columns = 1;
		R_z_c_m.data[0][0] = R_z_c.x;
		R_z_c_m.data[1][0] = R_z_c.y;
		R_z_c_m.data[2][0] = R_z_c.z;

		mf16 e_R_mat;
		v3d e_R;
		mf16_mul(&e_R_mat, &Rt, &R_z_c_m);
		e_R.x = e_R_mat.data[0][0];
		e_R.y = e_R_mat.data[1][0];
		e_R.z = e_R_mat.data[2][0];

		fix16_t e_R_z_sin = v3d_norm(&e_R);
		fix16_t e_R_z_cos = v3d_dot(&R_z, &R_sp_z);	//XXX: "float e_R_z_cos = R_z * R_sp_z;" means dot product(?)

		//px4: calculate weight for yaw control
		fix16_t yaw_w = fix16_sq(R_sp.data[2][2]);

		//px4: calculate rotation matrix after roll/pitch only rotation
		mf16 R_rp;
		R_rp.rows = 3;
		R_rp.columns = 3;

		e_R_z_sin = 0;	//TODO: XXX: Seems to fix issues somewhere

		if(e_R_z_sin > 0) {
			//px4: get axis-angle representation
			fix16_t e_R_z_angle = fix16_atan2(e_R_z_sin, e_R_z_cos);
			v3d e_R_z_axis;
			v3d_div_s(&e_R_z_axis, &e_R, e_R_z_sin);

			v3d_mul_s(&e_R, &e_R_z_axis, e_R_z_angle);

			//px4: cross product matrix for e_R_axis
			mf16 e_R_cp;
			e_R_cp.rows = 3;
			e_R_cp.columns = 3;
			//mf16_fill(&e_R_cp, 0);
			e_R_cp.data[0][1] = -e_R_z_axis.z;
			e_R_cp.data[0][2] = e_R_z_axis.y;
			e_R_cp.data[1][0] = e_R_z_axis.z;
			e_R_cp.data[1][2] = -e_R_z_axis.x;
			e_R_cp.data[2][0] = -e_R_z_axis.y;
			e_R_cp.data[2][1] = e_R_z_axis.x;

			//px4: rotation matrix for roll/pitch only rotation
			mf16 e_R_cp_2;
			mf16 e_R_cp_z_sin;
			mf16 e_R_add_temp;

			mf16_mul_s(&e_R_cp_z_sin, &e_R_cp, e_R_z_sin);
			mf16_mul(&e_R_cp_2, &e_R_cp, &e_R_cp);
			mf16_mul_s(&e_R_cp_2, &e_R_cp_2, fix16_sub(_fc_1, e_R_z_cos));

			mf16_add(&e_R_add_temp, &I, &e_R_cp_z_sin);
			mf16_add(&e_R_add_temp, &e_R_add_temp, &e_R_cp_2);

			mf16_mul(&R_rp, &R, &e_R_add_temp);
		} else {
			//px4: zero roll/pitch rotation
			R_rp = R;
		}

		//XXX: R_rp = R;

		//px4: R_rp and R_sp has the same Z axis, calculate yaw error
		v3d R_sp_x;
		R_sp_x.x = R_sp.data[0][0];
		R_sp_x.y = R_sp.data[1][0];
		R_sp_x.z = R_sp.data[2][0];

		v3d R_rp_x;
		R_rp_x.x = R_rp.data[0][0];
		R_rp_x.y = R_rp.data[1][0];
		R_rp_x.z = R_rp.data[2][0];

		v3d R_rp_c_sp;
		v3d_cross(&R_rp_c_sp, &R_rp_x, &R_sp_x);

		//e_R(2) = atan2f(R_rp_c_sp * R_sp_z, R_rp_x * R_sp_x) * yaw_w;
		e_R.z = fix16_mul(fix16_atan2(v3d_dot(&R_rp_c_sp, &R_sp_z), v3d_dot(&R_rp_x, &R_sp_x)), yaw_w);

		if(e_R_z_cos < 0) {
			//px4: for large thrust vector rotations use another rotation method:
			//px4: calculate angle and axis for R -> R_sp rotation directly
			qf16 q_error;
			mf16 Rt_R_sp;
			mf16_mul(&Rt_R_sp, &Rt, &R_sp);
			matrix_to_qf16(&q_error, &Rt_R_sp);
			v3d e_R_d;

			if(q_error.a >= 0) {
				v3d temp_vec;
				temp_vec.x = q_error.b;
				temp_vec.y = q_error.c;
				temp_vec.z = q_error.d;

				v3d_mul_s(&e_R_d, &temp_vec, _fc_2);
			} else {
				v3d temp_vec;
				temp_vec.x = -q_error.b;
				temp_vec.y = -q_error.c;
				temp_vec.z = -q_error.d;

				v3d_mul_s(&e_R_d, &temp_vec, _fc_2);
			}

			//px4: use fusion of Z axis based rotation and direct rotation
			fix16_t direct_w = fix16_mul(fix16_mul(e_R_z_cos, e_R_z_cos), yaw_w);

			//px4: e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
			v3d_mul_s(&e_R, &e_R, fix16_sub(_fc_1, direct_w));
			v3d_mul_s(&e_R_d, &e_R_d, direct_w);
			v3d_add(&e_R, &e_R, &e_R_d);
		}

		//px4: calculate angular rates setpoint
		v3d rates_sp;
		rates_sp.x = fix16_mul(get_param_fix16(PARAM_PID_ROLL_ANGLE_P), e_R.x);
		rates_sp.y = fix16_mul(get_param_fix16(PARAM_PID_PITCH_ANGLE_P), e_R.y);
		rates_sp.z = fix16_mul(get_param_fix16(PARAM_PID_YAW_ANGLE_P), e_R.z);

		//px4: feed forward yaw setpoint rate	//TODO:?
		//rates_sp.z += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

		return rates_sp;
}

void controller_run( uint32_t time_now ) {
	//Variables that store the computed attitude goal rates
	fix16_t goal_r = 0;
	fix16_t goal_p = 0;
	fix16_t goal_y = 0;
	fix16_t goal_throttle = 0;

	command_input_t input;
	controller_set_input_failsafe(&input); //Failsafe input to be safe

	//Handle controller input
	if(_system_status.state != MAV_STATE_CRITICAL) {
		if(true) {
			//Offboard mode
			input = _cmd_ob_input;
		} else if(false) {
			//Manual stabilize mode
			//Roll/pitch angle and yaw rate
			input.input_mask |= CMD_IN_IGNORE_ROLL_RATE;
			input.input_mask |= CMD_IN_IGNORE_PITCH_RATE;
			/*
			uint16_t pwm_roll = pwmRead(get_param_uint(RC_MAP_ROLL));
			uint16_t pwm_pitch = pwmRead(get_param_uint(RC_MAP_PITCH));
			uint16_t pwm_yaw = pwmRead(get_param_uint(RC_MAP_YAW));
			uint16_t pwm_throttle = pwmRead(get_param_uint(RC_MAP_THROTTLE));

			fix16_t rc_roll = normalized_input(pwm_roll);
			*/
		} else if(false) {
			//Manual acro mode
			//Roll/pitch/yaw rate
			input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

		} //Else: keep failsafe
	}

	//Get the control input mask to use
	_control_input.input_mask = input.input_mask;

	//Save intermittent goals
	_control_input.q = input.q;

	//==-- Attitude Control
	//If we should listen to attitude input
	if( !(_control_input.input_mask & CMD_IN_IGNORE_ATTITUDE) ) {
		//======== Body Frame Lock ========//
		//XXX: Very important!
		//TODO: Make a very clear note about this
		//The control method use here (px4 method) uses a method that allows
		// say a pitch of 20 Deg, and a yaw of 180 Deg, which will cause the
		// mav to pitch first, then rotate around to meet that angle, adjusting
		// for a pitch of -20 Deg and at first while the yaw rotates to the
		// correct angle.
		//Because of this, if there is a control request to override a specic
		// rotation body, and tell it that
		// we assume that the attitude message sent up was in the body frame
		qf16 q_control_lock;
		q_control_lock = _control_input.q;

		//Yaw control will be overriden
		if( !(_control_input.input_mask & CMD_IN_IGNORE_YAW_RATE) ) {
			qf16_align_to_axis(&q_control_lock, &q_control_lock, &_state_estimator.attitude, AXIS_LOCK_Z);
		}

		//Pitch control will be overriden
		if( !(_control_input.input_mask & CMD_IN_IGNORE_PITCH_RATE) ) {
			qf16_align_to_axis(&q_control_lock, &q_control_lock, &_state_estimator.attitude, AXIS_LOCK_Y);
		}

		//Roll control will be overriden
		if( !(_control_input.input_mask & CMD_IN_IGNORE_ROLL_RATE) ) {
			qf16_align_to_axis(&q_control_lock, &q_control_lock, &_state_estimator.attitude, AXIS_LOCK_X);
		}

		//Save intermittent goals
		_control_input.q = q_control_lock;
		//======== End Body Frame Lock ========//

		//Caclulate goal body rotations
		//XXX: v3d rates_sp = rate_goals_from_attitude(&q_control_lock, &q_body_lock);
		v3d rates_sp = rate_goals_from_attitude(&q_control_lock, &_state_estimator.attitude);

		//Set goal rates
		goal_r = rates_sp.x;
		goal_p = rates_sp.y;
		goal_y = rates_sp.z;
	}

	//==-- Rate Control PIDs
	//Roll Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_ROLL_RATE) ) {
		//Use the commanded roll rate goal
		goal_r = _cmd_ob_input.r;
	}

	//Pitch Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_PITCH_RATE) ) {
		//Use the commanded pitch rate goal
		goal_p = _cmd_ob_input.p;
	}

	//Yaw Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_YAW_RATE) ) {
		//Use the commanded yaw rate goal
		goal_y = _cmd_ob_input.y;
	}

	//Constrain rates to set params
	goal_r = fix16_constrain(goal_r, -get_param_fix16(PARAM_MAX_ROLL_RATE), get_param_fix16(PARAM_MAX_ROLL_RATE));
	goal_p = fix16_constrain(goal_p, -get_param_fix16(PARAM_MAX_PITCH_RATE), get_param_fix16(PARAM_MAX_PITCH_RATE));
	goal_y = fix16_constrain(goal_y, -get_param_fix16(PARAM_MAX_YAW_RATE), get_param_fix16(PARAM_MAX_YAW_RATE));

	//Save intermittent goals
	_control_input.r = goal_r;
	_control_input.p = goal_p;
	_control_input.y = goal_y;

	//Rate PID Controllers
	_control_output.r = pid_step(&_pid_roll_rate, time_now, goal_r, _state_estimator.p, 0, false);
	_control_output.p = pid_step(&_pid_pitch_rate, time_now, goal_p, _state_estimator.q, 0, false);
	_control_output.y = pid_step(&_pid_yaw_rate, time_now, goal_y, _state_estimator.r, 0, false);

	//==-- Throttle Control

	//Trottle
	if( !(_control_input.input_mask & CMD_IN_IGNORE_THROTTLE) ) {
		//Use the commanded throttle
		goal_throttle = _cmd_ob_input.T;
	}

	_control_output.T = goal_throttle;

	//Save intermittent goals
	_control_input.T = goal_throttle;
}

#ifdef __cplusplus
}
#endif
