#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixvector3d.h"
#include "estimator.h"
#include "control.h"

void controller_att_nac_init( void );
void controller_att_nac_reset( void );
void controller_att_nac_save_parameters( void );
void controller_att_nac_step( v3d* c, v3d* rates_ref, const command_input_t* input, const state_t* state, const fix16_t dt );

#ifdef __cplusplus
}
#endif
