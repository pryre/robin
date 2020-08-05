#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"

// input_mask defines
#define CMD_IN_IGNORE_ROLL_RATE (uint8_t)1  // 0b00000001
#define CMD_IN_IGNORE_PITCH_RATE (uint8_t)2 // 0b00000010
#define CMD_IN_IGNORE_YAW_RATE (uint8_t)4   // 0b00000100
#define CMD_IN_IGNORE_THROTTLE (uint8_t)64  // 0b01000000
#define CMD_IN_IGNORE_ATTITUDE (uint8_t)128 // 0b10000000

void pid_init( pid_controller_t* pid,
			   fix16_t kp,
			   fix16_t ki,
			   fix16_t kd,
			   fix16_t initial_x,
			   fix16_t initial_setpoint,
			   fix16_t initial_output,
			   fix16_t min,
			   fix16_t max );


void pid_set_gain_p( pid_controller_t* pid, fix16_t kp );
void pid_set_gain_i( pid_controller_t* pid, fix16_t ki );
void pid_set_gain_d( pid_controller_t* pid, fix16_t kd );
void pid_set_gains( pid_controller_t* pid, fix16_t kp, fix16_t ki, fix16_t kd );

void pid_set_min_max( pid_controller_t* pid, fix16_t min, fix16_t max );

void pid_reset( pid_controller_t* pid, fix16_t prev_x );

fix16_t pid_step( pid_controller_t* pid, fix16_t dt, fix16_t sp, fix16_t x );

#ifdef __cplusplus
}
#endif
