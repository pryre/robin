#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"

typedef struct {
	fix16_t kp;
	fix16_t ki;
	fix16_t kd;
//	fix16_t tau;

	fix16_t x;
//	fix16_t x_dot;
	fix16_t setpoint;
	fix16_t output;

	fix16_t max;
	fix16_t min;

	fix16_t integrator;
	fix16_t prev_x;
	fix16_t prev_e;
} pid_controller_t;

void pid_reset(pid_controller_t *pid, fix16_t prev_x);

void pid_set_gain_p(pid_controller_t *pid, fix16_t kp);
void pid_set_gain_i(pid_controller_t *pid, fix16_t ki);
void pid_set_gain_d(pid_controller_t *pid, fix16_t kd);
//void pid_set_gain_tau(pid_controller_t *pid, fix16_t tau);
//void pid_set_gains(pid_controller_t *pid, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau);
void pid_set_gains(pid_controller_t *pid, fix16_t kp, fix16_t ki, fix16_t kd);

void pid_set_min_max(pid_controller_t *pid, fix16_t min, fix16_t max);

//void pid_init(pid_controller_t *pid, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t tau, fix16_t initial_x, fix16_t initial_x_dot, fix16_t initial_setpoint, fix16_t initial_output, fix16_t min, fix16_t max);
void pid_init(pid_controller_t *pid, fix16_t kp, fix16_t ki, fix16_t kd, fix16_t initial_x, fix16_t initial_setpoint, fix16_t initial_output, fix16_t min, fix16_t max);
//fix16_t pid_step(pid_controller_t *pid, uint32_t time_now, fix16_t sp, fix16_t x, fix16_t x_dot, bool use_x_dot);
fix16_t pid_step(pid_controller_t *pid, fix16_t dt, fix16_t sp, fix16_t x);


#ifdef __cplusplus
}
#endif

