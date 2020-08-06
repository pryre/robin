#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixvector3d.h"
#include "control.h"
#include "controllers/lib_control_pid.h"

extern pid_controller_t _pid_roll_rate;
extern pid_controller_t _pid_pitch_rate;
extern pid_controller_t _pid_yaw_rate;

void controller_att_pid_reset( void );
void controller_att_pid_init( void );
void controller_att_pid_step( v3d* tau, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt ) {


#ifdef __cplusplus
}
#endif
