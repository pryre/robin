#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixvector3d.h"
#include "control.h"

void controller_nac_pid_reset( void );
void controller_nac_pid_init( void );
void controller_nac_pid_step( v3d* tau, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt ) {

#ifdef __cplusplus
}
#endif
