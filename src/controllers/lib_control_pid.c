#ifdef __cplusplus
extern "C" {
#endif

#include "controllers/pid_controller.h"
#include "fix16.h"
#include "fixextra.h"

// By default, pass prev_x as 0, unless you know the first measurement
void pid_reset( pid_controller_t* pid, fix16_t prev_x ) {
	pid->integrator = 0;
	pid->prev_x = prev_x;
}

void pid_set_gain_p( pid_controller_t* pid, fix16_t kp ) {
	pid->kp = kp;
}

void pid_set_gain_i( pid_controller_t* pid, fix16_t ki ) {
	if ( ki == 0.0 ) {
		// Clear the integrator so it starts fresh
		pid->integrator = 0.0;
	} else if ( pid->ki != 0.0 ) {
		// Scale integrator so it has same weighting
		pid->integrator = fix16_mul( fix16_div( pid->ki, ki ), pid->integrator );
	}

	pid->ki = ki;
}

void pid_set_gain_d( pid_controller_t* pid, fix16_t kd ) {
	pid->kd = kd;
}

void pid_set_gains( pid_controller_t* pid, fix16_t kp, fix16_t ki, fix16_t kd ) {
	pid_set_gain_p( pid, kp );
	pid_set_gain_i( pid, ki );
	pid_set_gain_d( pid, kd );
}

void pid_set_min_max( pid_controller_t* pid, fix16_t min, fix16_t max ) {
	pid->min = min;
	pid->max = max;
}

void pid_init( pid_controller_t* pid, fix16_t kp, fix16_t ki, fix16_t kd,
			   fix16_t initial_x, fix16_t initial_setpoint,
			   fix16_t initial_output, fix16_t min, fix16_t max ) {
	pid_set_gains( pid, kp, ki, kd );

	pid->x = initial_x;
	pid->setpoint = initial_setpoint;
	pid->output = initial_output;
	pid_set_min_max( pid, min, max );

	pid_reset( pid, initial_x );
}

// Steps the controller along,
// Accepts:
//	- The PID to use
//	- Current time
//	- Commanded setpoint
//	- Measured current state
//	- Measured derivative state (if available)
//	- Whether to use the measured derivative state (previous arg) or (PID)
// estimated derivative state
// Returns:
//	- Output Command
fix16_t pid_step( pid_controller_t* pid, fix16_t dt, fix16_t sp, fix16_t x ) {
	pid->x = x;
	pid->setpoint = sp;

	// Calculate error
	fix16_t error = fix16_sub( pid->setpoint, pid->x );

	// Initialize Terms
	fix16_t p_term = fix16_mul( error, pid->kp );
	fix16_t i_term = 0;
	fix16_t d_term = 0;

	// If it is a stale controller, just skip this section
	if ( dt > 0 ) {
		//==-- Derivative
		if ( pid->kd > 0 ) {
			//Use dx instead of de to improve stability (removes noise due to changing reference)
			fix16_t dx = fix16_div( fix16_sub( x, pid->prev_x ), dt );

			d_term = -fix16_mul( pid->kd, dx );
		}

		//==-- Integrator
		// If the integrator is valid (and if there is an integrator)
		if ( pid->ki > 0 ) {
			// Integrate over dt
			pid->integrator = fix16_add( pid->integrator, fix16_mul( error, dt ) );
			// Calculate I term
			i_term = fix16_mul( pid->ki, pid->integrator );
		}
	}

	// Sum three terms: u = p_term + i_term + d_term
	fix16_t u = fix16_add( p_term, d_term );
	fix16_t ui = fix16_add( u, i_term );

	// Output Saturation
	fix16_t u_sat = ( ui > pid->max ) ? pid->max : ( ui < pid->min ) ? pid->min : ui;

	// Integrator anti-windup
	// If the pid controller has saturated and if the integrator is the cause
	if ( ( ui != u_sat ) && ( pid->ki > 0 ) && ( fix16_abs( i_term ) > fix16_abs( fix16_sub( u_sat, u ) ) ) )
		pid->integrator = fix16_div( fix16_sub( u_sat, u ), pid->ki ); // Trim the integrator to what
	// it should currently be to
	// only just hit the maximum

	// Set output
	pid->output = u_sat;

	// Save running values
	pid->prev_x = pid->x;

	return pid->output;
}

#ifdef __cplusplus
}
#endif
