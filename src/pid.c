#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixextra.h"
#include "pid.h"

//By default, pass prev_x as 0, unless you know the first measurement
void pid_reset(pid_t *pid_, fix16_t prev_x) {
	pid_->integrator = 0;
	pid_->prev_time = 0;
	pid_->prev_x = prev_x;
}

void pid_set_gains(pid_t *pid_, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau) {
	pid_->kp = kp;
	pid_->ki = ki;
	pid_->kd = kd;
	pid_->tau = tau;
}

void pid_set_min_max(pid_t *pid_, fix16_t min, fix16_t max) {
	pid_->min = min;
	pid_->max = max;
}

void pid_init(pid_t *pid_, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau, fix16_t initial_x, fix16_t initial_x_dot, fix16_t initial_setpoint, fix16_t initial_output, fix16_t min, fix16_t max) {
	pid_set_gains(pid_, kp, ki, kd, tau);

	pid_->x = initial_x;
	pid_->x_dot = initial_x_dot;
	pid_->setpoint = initial_setpoint;
	pid_->output = initial_output;
	pid_set_min_max(pid_, min, max);

	pid_reset(pid_, initial_x);
}

//Steps the controller along,
// Accepts:
//	- The PID to use
//	- Current time
//	- Commanded setpoint
//	- Measured current state
//	- Measured derivative state (if available)
//	- Whether to use the measured derivative state (previous arg) or (PID) estimated derivative state
// Returns:
//	- Output Command
fix16_t pid_step(pid_t *pid_, uint32_t time_now, fix16_t sp, fix16_t x, fix16_t x_dot, bool use_x_dot) {
	fix16_t dt = fix16_from_float((float)(time_now - pid_->prev_time) * 1e-6);	//Delta time in milliseconds
	pid_->prev_time = time_now;

	if(dt > CONST_ZERO_ZERO_ONE) {
		// This means that this is a ''stale'' controller and needs to be reset.
		// This would happen if we have been operating in a different mode for a while
		// and will result in some enormous integrator.
		// Or, it means we are disarmed and shouldn't integrate
		// Setting dt for this loop will mean that the integrator and dirty derivative
		// doesn't do anything this time but will keep it from exploding.
		pid_reset(pid_, x);
		dt = 0;
	}

	pid_->x = x;
	pid_->setpoint = sp;

	//Calculate error
	fix16_t error = fix16_sub(pid_->setpoint, pid_->x);

	//Initialize Terms
	fix16_t p_term = fix16_mul(error, pid_->kp);
	fix16_t i_term = 0;
	fix16_t d_term = 0;

	//If it is a stale controller, just skip this section
	if(dt > 0) {
		//==-- Derivative
		if(pid_->kd > 0) {
			//Calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
			if(!use_x_dot) {
				//The dirty derivative is a sort of low-pass filtered version of the derivative.
				// pid.x_dot = ((2.0f * pid.tau - dt) / (2.0f * pid.tau + dt) * pid.x_dot) + (2.0f / (2.0f * pid.tau + dt) * (pid.x - pid.prev_x));
				pid_->x_dot = fix16_add(fix16_mul(fix16_div(fix16_sub(fix16_mul(CONST_TWO, pid_->tau), dt), fix16_add(fix16_mul(CONST_TWO, pid_->tau), dt)), pid_->x_dot), fix16_mul(fix16_div(CONST_TWO, fix16_add(fix16_mul(CONST_TWO, pid_->tau), dt)), fix16_sub(pid_->x, pid_->prev_x)));
			} else {
				pid_->x_dot = x_dot;
			}

			d_term = fix16_mul(pid_->kd, pid_->x_dot);
		}

		//==-- Integrator
		// If the integrator is valid (and if there is an integrator)
		if (pid_->ki > 0) {
			//Integrate over dt
			pid_->integrator = fix16_add( pid_->integrator, fix16_mul(error, dt) );
			//Calculate I term
			i_term = fix16_mul(pid_->ki, pid_->integrator);
		}
	}

	//TODO: May have to be "- d_term"
	//Sum three terms: u = p_term + i_term + d_term
	fix16_t u = fix16_add( p_term, fix16_add( i_term, d_term ) );

	//Output Saturation
	fix16_t u_sat = (u > pid_->max) ? pid_->max : (u < pid_->min) ? pid_->min : u;

	//Integrator anti-windup
	if( (pid_->ki > 0) && (u != u_sat) )	//If the pid controller has saturated, and there is an integrator active
		if( fix16_abs(i_term) > fix16_abs(fix16_sub(fix16_sub(u, p_term), d_term)) )	//And if the integrator is the cause
			pid_->integrator = fix16_div(fix16_sub(fix16_sub(u_sat, p_term), d_term), pid_->ki);	//Trim the integrator to what it should currently be to only just hit the maximum

	pid_->prev_x = pid_->x;
	//Set output
	pid_->output = u_sat;

	return pid_->output;
}

#ifdef __cplusplus
}
#endif
