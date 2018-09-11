#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "controller.h"

#define MIXER_NUM_MOTORS 8
#define MIXER_TEST_MOTORS_ALL 0xFF
#define MIXER_NUM_AUX 4

//TODO: Need to add support later for more mixing types
//http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/frame-motor-mixing-table/
typedef enum {
	MIXER_NONE = 0,
	MIXER_QUADROTOR_PLUS = 5001,
	MIXER_QUADROTOR_X = 4001,
	MIXER_HEXAROTOR_X = 6001,
	MIXER_PLANE_STANDARD = 2100,
	NUM_MIXERS
} mixer_type_t;

typedef enum {
	MT_NONE,	//None
	MT_S,		//Servo
	MT_M,		//Motor
	MT_A,		//Actuator
	MT_G		//GPIO
} output_type_t;

typedef struct {
	bool mixer_ok;

	output_type_t output_type[MIXER_NUM_MOTORS];
	fix16_t T[MIXER_NUM_MOTORS];
	fix16_t x[MIXER_NUM_MOTORS];
	fix16_t y[MIXER_NUM_MOTORS];
	fix16_t z[MIXER_NUM_MOTORS];
} mixer_t;


typedef struct {
	uint32_t start;
	fix16_t throttle;
	uint32_t duration;
	bool test_all;
	uint8_t motor_step;
} mixer_motor_test_t;

extern int32_t _GPIO_outputs[MIXER_NUM_MOTORS];
extern output_type_t _GPIO_output_type[MIXER_NUM_MOTORS];

extern int32_t _pwm_control[MIXER_NUM_MOTORS];

//TODO: Switch PWM Control to G1 input / override
extern int8_t _actuator_apply_g1_map[MIXER_NUM_MOTORS];
extern bool _actuator_apply_g2;
extern fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
extern fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];
extern fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];

extern mixer_motor_test_t _motor_test;

extern int32_t _pwm_output_requested[MIXER_NUM_MOTORS];
extern int32_t _pwm_output[MIXER_NUM_MOTORS];

void pwm_init();
void mixer_init();

void write_output_pwm(uint8_t index, uint32_t value, uint32_t value_disarm);
void write_motor(uint8_t index, uint32_t value);
void write_servo(uint8_t index, int32_t value);

void mixer_output();
