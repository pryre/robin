#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "fixquat.h"

#include "params.h"
//#include "pid.h"

//input_mask defines
#define CMD_IN_IGNORE_ROLL_RATE		0b00000001
#define CMD_IN_IGNORE_PITCH_RATE	0b00000010
#define CMD_IN_IGNORE_YAW_RATE		0b00000100
#define CMD_IN_IGNORE_THROTTLE		0b01000000
#define CMD_IN_IGNORE_ATTITUDE		0b10000000

typedef struct {
	fix16_t r;
	fix16_t p;
	fix16_t y;
	qf16 q;
	fix16_t T;
	uint8_t input_mask; //Mappings set with the defines above, setting a bit will cause that input to be ignored during operation
} command_input_t;

typedef struct {
	fix16_t r;
	fix16_t p;
	fix16_t y;
	fix16_t T;
} control_output_t;

extern command_input_t _command_input;
extern control_output_t _control_output;

void controller_init();
void controller_run( uint32_t now );
void controller_reset();

#ifdef __cplusplus
}
#endif
