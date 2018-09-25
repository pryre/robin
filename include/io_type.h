#pragma once

#define PWM_MAX_PORTS 14

#include <stdint.h>

typedef enum {
	IO_TYPE_N = 0,	//None
	IO_TYPE_IP,		//Input PPM
	IO_TYPE_IW,		//Input PWM
	IO_TYPE_OD,		//Output Digital
	IO_TYPE_OM,		//Output Motor
	IO_TYPE_OS		//Output Servo
} io_type_t;

typedef struct {
	uint8_t port[PWM_MAX_PORTS];
	uint8_t type[PWM_MAX_PORTS];
} io_def_t;
