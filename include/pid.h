#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"

typedef struct {
	fix16_t kp;
	fix16_t ki;
	fix16_t kd;
	fix16_t tau;

	fix16_t x;
	fix16_t x_dot;
	fix16_t setpoint;
	fix16_t output;

	fix16_t max;
	fix16_t min;

	fix16_t integrator;
	fix16_t prev_x;
	uint32_t prev_time;
} pid_t;

void pid_reset(pid_t *pid_, fix16_t prev_x);
void pid_set_gains(pid_t *pid_, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau);
void pid_set_min_max(pid_t *pid_, fix16_t min, fix16_t max);
void pid_init(pid_t *pid_, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau, fix16_t initial_x, fix16_t initial_x_dot, fix16_t initial_setpoint, fix16_t initial_output, fix16_t min, fix16_t max);
fix16_t pid_step(pid_t *pid_, uint32_t time_now, fix16_t sp, fix16_t x, fix16_t x_dot, bool use_x_dot);


#ifdef __cplusplus
}
#endif

