#include "fix16.h"

void pid_init(pid_t* pid, param_id_t kp_param_id, param_id_t ki_param_id, param_id_t kd_param_id, fix16_t current_x, fix16_t current_xdot, fix16_t commanded_x, fix16_t output, fix16_t max, fix16_t min) {
	pid->kp_param_id = kp_param_id;
	pid->ki_param_id = ki_param_id;
	pid->kd_param_id = kd_param_id;

	pid->current_x = current_x;
	pid->current_xdot = current_xdot;
	pid->commanded_x = commanded_x;
	pid->output = output;
	pid->max = max;
	pid->min = min;

	pid->integrator = 0.0;
	pid->prev_time = micros()*1e-6;
	pid->differentiator = 0.0;
	pid->prev_x = 0.0;
	pid->tau = get_param_fix16(PARAM_PID_TAU);
}


void pid_step(pid_t *pid) {
	// Time calculation
	float now = micros()*1e-6;
	float dt = now - pid->prev_time;
	pid->prev_time = now;

	if(dt > 0.010 || _armed_state == DISARMED) {
		// This means that this is a ''stale'' controller and needs to be reset.
		// This would happen if we have been operating in a different mode for a while
		// and will result in some enormous integrator.
		// Or, it means we are disarmed and shouldn't integrate
		// Setting dt for this loop will mean that the integrator and dirty derivative
		// doesn't do anything this time but will keep it from exploding.
		dt = 0.0;
		pid->differentiator = 0.0;
	}

	// Calculate Error (make sure to de-reference pointers)
	float error = (*pid->commanded_x) - (*pid->current_x);

	// Initialize Terms
	float p_term = error * get_param_float(pid->kp_param_id);
	float i_term = 0.0;
	float d_term = 0.0;

	// If there is a derivative term
	if(pid->kd_param_id < PARAMS_COUNT) {
		// calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
		// The dirty derivative is a sort of low-pass filtered version of the derivative.
		// (Be sure to de-refernce pointers)
		if(pid->current_xdot == NULL && dt > 0.0f) {
			pid->differentiator = (2.0f*pid->tau-dt)/(2.0f*pid->tau+dt)*pid->differentiator + 2.0f/(2.0f*pid->tau+dt)*((*pid->current_x) - pid->prev_x);
			pid->prev_x = *pid->current_x;
			d_term = get_param_float(pid->kd_param_id)*pid->differentiator;
		} else {
			d_term = get_param_float(pid->kd_param_id) * (*pid->current_xdot);
		}
	}

	// If there is an integrator, we are armed, and throttle is high
	/// TODO: better way to figure out if throttle is high
	if (pid->ki_param_id < PARAMS_COUNT && _armed_state == ARMED && pwmRead(get_param_int(PARAM_RC_F_CHANNEL) > 1200)) {
		if ( get_param_float(pid->ki_param_id) > 0.0 ) {
			// integrate
			pid->integrator += error*dt;
			// calculate I term (be sure to de-reference pointer to gain)
			i_term = get_param_float(pid->ki_param_id) * pid->integrator;
		}
	}

	// sum three terms
	float u = p_term + i_term - d_term;

	// Integrator anti-windup
	float u_sat = (u > pid->max) ? pid->max : (u < pid->min) ? pid->min : u;
	if(u != u_sat && fabs(i_term) > fabs(u - p_term + d_term))
	pid->integrator = (u - p_term + d_term)/get_param_float(pid->ki_param_id);

	// Set output
	(*pid->output) = u_sat;
}
