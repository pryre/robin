#pragma once

#include "fix16.h"

typedef struct {
	param_id_t kp_param_id;
	param_id_t ki_param_id;
	param_id_t kd_param_id;

	fix16_t current_x;
	fix16_t current_xdot;
	fix16_t commanded_x;
	fix16_t output;

	fix16_t max;
	fix16_t min;

	fix16_t integrator;
	fix16_t prev_time;
	fix16_t prev_x;
	fix16_t differentiator;
	fix16_t tau;
} pid_t;

void pid_init(pid_t* pid, param_id_t kp_param_id, param_id_t ki_param_id, param_id_t kd_param_id, fix16_t current_x, fix16_t current_xdot, fix16_t commanded_x, fix16_t output, fix16_t max, fix16_t min);
void pid_step(pid_t* pid);
