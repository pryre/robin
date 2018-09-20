#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "fixquat.h"

#include "params.h"
#include "pid_controller.h"

//input_mask defines
#define CMD_IN_IGNORE_ROLL_RATE		(uint8_t)1		//0b00000001
#define CMD_IN_IGNORE_PITCH_RATE	(uint8_t)2		//0b00000010
#define CMD_IN_IGNORE_YAW_RATE		(uint8_t)4		//0b00000100
#define CMD_IN_IGNORE_THROTTLE		(uint8_t)64		//0b01000000
#define CMD_IN_IGNORE_ATTITUDE		(uint8_t)128	//0b10000000

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

typedef struct {
	uint32_t period_update;
	uint32_t period_stale;
	uint32_t time_last;
	fix16_t average_update;
} control_timing_t;

extern control_timing_t _control_timing;
extern command_input_t _cmd_ob_input;
extern command_input_t _control_input;
extern control_output_t _control_output;

extern pid_controller_t _pid_roll_rate;
extern pid_controller_t _pid_pitch_rate;
extern pid_controller_t _pid_yaw_rate;

//void controller_reset(void);
void control_init(void);
//void controller_run( uint32_t now );
void control_run( uint32_t now );

#ifdef __cplusplus
}
#endif
