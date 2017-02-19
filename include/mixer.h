#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "controller.h"

//TODO: Need to add support later for more mixing types
typedef enum {
	QUADCOPTER_PLUS,
	QUADCOPTER_X,
	QUADCOPTER_H,
	NUM_MIXERS
} mixer_type_t;

typedef enum {
	NONE,	//None
	S,		//Servo
	M,		//Motor
	G		//GPIO
} output_type_t;

typedef struct {
	output_type_t output_type[8];
	fix16_t F[8];
	fix16_t x[8];
	fix16_t y[8];
	fix16_t z[8];
} mixer_t;

extern int32_t _GPIO_outputs[8];
extern output_type_t _GPIO_output_type[8];

extern int32_t _outputs[8];

void PWM_init();
void mixer_init();
void mixer_output();
