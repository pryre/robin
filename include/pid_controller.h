#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float kp;
	float ki;
	float kd;
	float tau;

	float x;
	float x_dot;
	float setpoint;
	float output;

	float max;
	float min;

	float integrator;
	float prev_x;
	uint32_t prev_time;
} pid_controller_t;

void pid_reset(pid_controller_t *pid, float prev_x);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd, float tau);
void pid_set_min_max(pid_controller_t *pid, float min, float max);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float tau, float initial_x, float initial_x_dot, float initial_setpoint, float initial_output, float min, float max);
float pid_step(pid_controller_t *pid, uint32_t time_now, float sp, float x, float x_dot, bool use_x_dot);


#ifdef __cplusplus
}
#endif

