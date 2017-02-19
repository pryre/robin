#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "params.h"
#include "pid.h"

typedef struct {
	fix16_t F;
	fix16_t x;
	fix16_t y;
	fix16_t z;
} command_t;

pid_t pid_roll;
pid_t pid_roll_rate;
pid_t pid_pitch;
pid_t pid_pitch_rate;
pid_t pid_yaw_rate;
pid_t pid_altitude;

extern command_t _command;

void controller_init();
void controller_run();

#ifdef __cplusplus
}
#endif
