#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "fixvector3d.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixextra.h"

#include "mavlink_system.h"

#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "estimator.h"
#include "controller.h"
#include "mixer.h"
#include "pid_controller.h"


system_status_t _system_status;
sensor_readings_t _sensors;
state_t _state_estimator;

control_timing_t _control_timing;
command_input_t _cmd_ob_input;
command_input_t _control_input;
control_output_t _control_output;

pid_controller_t _pid_roll_rate;
pid_controller_t _pid_pitch_rate;
pid_controller_t _pid_yaw_rate;

static void controller_reset(void) {
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
	_control_input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

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

//Technique adapted from the Pixhawk multirotor control scheme (~December 2018)
static void rates_from_attitude(v3d *rates, const qf16 *q_sp, const qf16 *q, const fix16_t yaw_w) {
	/*
	mf16 R;
	mf16 Rd;
	qf16_to_matrix(&R, q);
	qf16_to_matrix(&Rd, q_sp);

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	v3d e_z;
	v3d e_z_d;
	dcm_to_basis_z(&e_z, &R);
	dcm_to_basis_z(&e_z_d, &Rd);

	qf16 qd_red;
	qf16_from_shortest_path(&qd_red, &e_z, &e_z_d);

	//Handle co-linear vectoring cases
	if ( fix16_abs(qd_red.a) >= _fc_epsilon ) {
		// There is little or no difference in thrust vector direction, so they are parrallel
		// If they are parallel, we only need yaw motion, but we still need want to use qd_red
		// to ensure that the yaw is scaled correctly by the gain/weight
		qd_red.a = _fc_1;
		qd_red.b = 0;
		qd_red.c = 0;
		qd_red.d = 0;
	} else if ( fix16_abs(qd_red.b) >= _fc_epsilon ) {
		// Otherwise they are in opposite directions which presents an ambiguous solution
		// The best we can momenterily is to just accept bad weightings from the mixing and
		// do the 'best' movement possible for 1 time step until things can be calculated
		qd_red = *q_sp;
	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qf16_mul(&qd_red, &qd_red, q);
	}

	// mix full and reduced desired attitude
	qf16 q_mix;
	qf16_inverse(&qd_red, &qd_red);
	qf16_mul(&q_mix, &qd_red, q_sp);
	qf16_mul_s(&q_mix, &q_mix, fix16_sign_no_zero(q_mix.a));
	// catch numerical problems with the domain of acosf and asinf
	fix16_t q_mix_w = fix16_constrain(q_mix.a, -1.0, 1.0);
	fix16_t q_mix_z = fix16_constrain(q_mix.d, -1.0, 1.0);
	q_mix.a = fix16_cos( fix16_mul( yaw_w, fix16_acos(q_mix_w) ) );
	q_mix.b = 0;
	q_mix.c = 0;
	q_mix.d = fix16_sin( fix16_mul( yaw_w, fix16_asin(q_mix_z) ) );
	qf16_normalize_to_unit(&q_mix, &q_mix);

	qf16 qd;
	qf16_mul(&qd, &qd_red, &q_mix);

	// quaternion attitude control law, qe is rotation from q to qd
	qf16 qe;
	qf16_inverse(&qe, q);
	qf16_mul(&qe, &qe, &qd);
	qf16_normalize_to_unit(&qe, &qe);
	*/
	//XXX: HERE!
	qf16 qe;
	qf16_inverse(&qe, q);
	qf16_mul(&qe, &qe, q_sp);
	qf16_normalize_to_unit(&qe, &qe);

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	v3d e_R;
	qf16_to_v3d(&e_R, &qe);
	v3d_mul_s(&e_R, &e_R, fix16_mul( _fc_2, fix16_sign_no_zero(qe.a) ) );

	v3d_mul_s( rates, &e_R, get_param_fix16(PARAM_MC_ANGLE_P) );
}

static void controller_run( uint32_t time_now ) {
	//Variables that store the computed attitude goal rates
	fix16_t goal_r = 0;
	fix16_t goal_p = 0;
	fix16_t goal_y = 0;
	fix16_t goal_throttle = 0;

	command_input_t input;
	controller_set_input_failsafe(&input); //Failsafe input to be safe


	//Handle controller input
	if(_system_status.state != MAV_STATE_CRITICAL) {
		//_system_status.mode |= MAV_MODE_FLAG_MANUAL_ENABLED
		switch(_system_status.control_mode) {
			case MAIN_MODE_OFFBOARD: {
				//Offboard mode
				input = _cmd_ob_input;

				break;
			}
			case MAIN_MODE_STABILIZED: {
				//Manual stabilize mode
				//Roll/pitch angle and yaw rate
				input.input_mask = 0;
				input.input_mask |= CMD_IN_IGNORE_ROLL_RATE;
				input.input_mask |= CMD_IN_IGNORE_PITCH_RATE;

				fix16_t man_roll = fix16_mul(_sensors.rc_input.c_r, get_param_fix16(PARAM_MAX_ROLL_ANGLE));
				fix16_t man_pitch = fix16_mul(_sensors.rc_input.c_p, get_param_fix16(PARAM_MAX_PITCH_ANGLE));

				quat_from_euler(&input.q, man_roll, man_pitch, 0);

				input.r = 0;
				input.p = 0;
				input.y = fix16_mul(_sensors.rc_input.c_y, get_param_fix16(PARAM_MAX_YAW_RATE));

				input.T = _sensors.rc_input.c_T;

				break;
			}
			case MAIN_MODE_ACRO: {
				//Manual acro mode
				//Roll/pitch/yaw rate
				input.input_mask = 0;
				input.input_mask |= CMD_IN_IGNORE_ATTITUDE;

				input.q.a = 1.0;
				input.q.b = 0;
				input.q.c = 0;
				input.q.d = 0;

				input.r = fix16_mul(_sensors.rc_input.c_r, get_param_fix16(PARAM_MAX_ROLL_RATE));
				input.p = fix16_mul(_sensors.rc_input.c_p, get_param_fix16(PARAM_MAX_PITCH_RATE));
				input.y = fix16_mul(_sensors.rc_input.c_y, get_param_fix16(PARAM_MAX_YAW_RATE));

				input.T = _sensors.rc_input.c_T;

				break;
			}
			default: {
				//Keep failsafe
				break;
			}
		}
	}

	fix16_t dt = fix16_from_float(1e-6 * (float)(time_now - _control_timing.time_last));	//Delta time in seconds

	if( (dt == 0) || (time_now - _control_timing.time_last > _control_timing.period_stale) ) {
		dt = 0;
		controller_reset();
	}

	//Get the control input mask to use
	_control_input.input_mask = input.input_mask;

	//==-- Throttle Control
	//Trottle
	if( !(_control_input.input_mask & CMD_IN_IGNORE_THROTTLE) ) {
		//Use the commanded throttle
		goal_throttle = input.T;
	}

	_control_output.T = goal_throttle;

	//Save intermittent goals
	_control_input.T = goal_throttle;

	//Don't allow for derivative or integral calculations if
	// there is a very low throttle
	if(goal_throttle < _fc_0_05) {
		dt = 0;
	}

	//==-- Attitude Control
	//Save intermittent goals
	qf16_normalize_to_unit(&_control_input.q, &input.q);

	//If we should listen to attitude input
	if( !(_control_input.input_mask & CMD_IN_IGNORE_ATTITUDE) ) {
		//If we are going to override the calculated yaw rate, just ignore it now
		fix16_t yaw_w = get_param_fix16(PARAM_MC_ANGLE_YAW_W);

		if( !(_control_input.input_mask & CMD_IN_IGNORE_YAW_RATE) ) {
			yaw_w = 0;
		}

		v3d rates_sp;
		rates_from_attitude(&rates_sp, &_control_input.q, &_state_estimator.attitude, yaw_w);

		//Set goal rates
		goal_r = rates_sp.x;
		goal_p = rates_sp.y;
		goal_y = rates_sp.z;
	}

	//==-- Rate Control PIDs
	//Roll Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_ROLL_RATE) ) {
		//Use the commanded roll rate goal
		goal_r = input.r;
	}

	//Pitch Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_PITCH_RATE) ) {
		//Use the commanded pitch rate goal
		goal_p = input.p;
	}

	//Yaw Rate
	if( !(_control_input.input_mask & CMD_IN_IGNORE_YAW_RATE) ) {
		//Use the commanded yaw rate goal
		goal_y = input.y;
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
	fix16_t command_r = pid_step(&_pid_roll_rate, dt, goal_r, _state_estimator.p);
	fix16_t command_p = pid_step(&_pid_pitch_rate, dt, goal_p, _state_estimator.q);
	fix16_t command_y = pid_step(&_pid_yaw_rate, dt, goal_y, _state_estimator.r);

	//XXX:
	//"Post-Scale/Normalize" the commands to act within
	//a range that is approriate for the motors. This
	//allows us to have higher PID gains for the rates
	//(by a factor of 100), and avoids complications
	//of getting close to the fixed-point step size
	_control_output.r = fix16_div(command_r, _fc_100);
	_control_output.p = fix16_div(command_p, _fc_100);
	_control_output.y = fix16_div(command_y, _fc_100);
}

void control_init(void) {
	_system_status.control_mode = 0;
	_control_timing.time_last = 0;
	_control_timing.period_update = 1000*fix16_to_int( fix16_div( _fc_1000, get_param_fix16(PARAM_RATE_CONTROL) ) );
	_control_timing.period_stale = 4*_control_timing.period_update;	//Allow for some variance in update rate
	_control_timing.average_update = 0;

	pid_init(&_pid_roll_rate,
			 get_param_fix16(PARAM_PID_ROLL_RATE_P),
			 get_param_fix16(PARAM_PID_ROLL_RATE_I),
			 get_param_fix16(PARAM_PID_ROLL_RATE_D),
			 _state_estimator.p,
			 0, 0, -_fc_100, _fc_100);	//XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init(&_pid_pitch_rate,
			 get_param_fix16(PARAM_PID_PITCH_RATE_P),
			 get_param_fix16(PARAM_PID_PITCH_RATE_I),
			 get_param_fix16(PARAM_PID_PITCH_RATE_D),
			 _state_estimator.q,
			 0, 0, -_fc_100, _fc_100);	//XXX: Mixer input is normalized from 100/-100 to 1/-1

	pid_init(&_pid_yaw_rate,
			 get_param_fix16(PARAM_PID_YAW_RATE_P),
			 get_param_fix16(PARAM_PID_YAW_RATE_I),
			 get_param_fix16(PARAM_PID_YAW_RATE_D),
			 _state_estimator.r,
			 0, 0, -_fc_100, _fc_100);	//XXX: Mixer input is normalized from 100/-100 to 1/-1

	_cmd_ob_input.r = 0;
	_cmd_ob_input.p = 0;
	_cmd_ob_input.y = 0;
	_cmd_ob_input.q.a = _fc_1;
	_cmd_ob_input.q.b = 0;
	_cmd_ob_input.q.c = 0;
	_cmd_ob_input.q.d = 0;
	_cmd_ob_input.T = 0;
	_cmd_ob_input.input_mask = 0;;

	controller_reset();
}

void control_run( uint32_t now ) {
	//Run the control loop at a slower frequency so it is more resilient against noise
	if( ( now - _control_timing.time_last ) > _control_timing.period_update) {
		if( ( safety_is_armed() ) && ( _system_status.state != MAV_STATE_EMERGENCY ) ) {
			//==-- Update Controller
			controller_run( sensors_clock_imu_int_get() );	//Apply the current commands and update the PID controllers
		} else {
			//==-- Reset Controller
			controller_reset();	//Reset the PIDs and output flat 0s for control
		}

		calc_mixer_output();

		//Calculate timings for feedback
		_control_timing.average_update = fix16_div(_fc_1000, fix16_from_int( (now - _control_timing.time_last) / 1000 ) );

		_control_timing.time_last = now;
	}
}

#ifdef __cplusplus
}
#endif
