#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"
#include "fixquat.h"

#include "params.h"

typedef struct {
	fix16_t r;
	fix16_t p;
	fix16_t y;
	qf16 q;
	fix16_t T;
	uint8_t input_mask; // Mappings set with the defines above, setting a bit will
						// cause that input to be ignored during operation
} command_input_t;

typedef struct {
	uint32_t period_update;
	uint32_t period_stale;
	uint32_t time_last;
	fix16_t average_update;
} control_timing_t;

extern control_timing_t _control_timing;
extern command_input_t _cmd_ob_input;
extern command_input_t _control_input;

// void controller_reset(void);
void control_init( void );
// void controller_run( uint32_t now );
void control_run( uint32_t now );

#ifdef __cplusplus
}
#endif
