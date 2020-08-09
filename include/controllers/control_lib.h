#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fix16.h"
#include "fixmatrix.h"
#include "fixquat.h"
#include "fixvector3d.h"

#include "control.h"

// input_mask defines
#define CMD_IN_IGNORE_ROLL_RATE (uint8_t)1  // 0b00000001
#define CMD_IN_IGNORE_PITCH_RATE (uint8_t)2 // 0b00000010
#define CMD_IN_IGNORE_YAW_RATE (uint8_t)4   // 0b00000100
#define CMD_IN_IGNORE_THROTTLE (uint8_t)64  // 0b01000000
#define CMD_IN_IGNORE_ATTITUDE (uint8_t)128 // 0b10000000

void control_lib_set_input_zero( command_input_t* input );
void control_lib_set_input_from_mode( command_input_t* input );

void control_lib_q_att_error( v3d* e_R, qf16* qe, const qf16* q_sp, const qf16* q, const fix16_t yaw_w );

#ifdef __cplusplus
}
#endif
