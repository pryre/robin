#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "control.h"
#include "fix16.h"

#include "io_type.h"

#define MIXER_NUM_MOTORS 8
#define MIXER_TEST_MOTORS_ALL 0xFF
#define MIXER_NUM_AUX 4

// TODO: Need to add support later for more mixing types
// http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/frame-motor-mixing-table/
typedef enum {
	MIXER_NONE = 0,
	MIXER_FREE = 999,
	MIXER_QUADROTOR_PLUS = 5001,
	MIXER_QUADROTOR_X = 4001,
	MIXER_HEXAROTOR_X = 6001,
	MIXER_PLANE_STANDARD = 2100,
	NUM_MIXERS
} mixer_type_t;

typedef struct {
	bool mixer_ok;

	io_type_t output_type[MIXER_NUM_MOTORS];
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

// extern int32_t _pwm_control[MIXER_NUM_MOTORS];

//extern int8_t _actuator_apply_g1_map[MIXER_NUM_MOTORS];
extern io_type_t _actuator_type_map[MIXER_NUM_MOTORS];
extern fix16_t _actuator_control_g0[MIXER_NUM_MOTORS]; // Motors Controls
extern fix16_t _actuator_control_g1[MIXER_NUM_MOTORS]; // Motor Additions
extern fix16_t _actuator_control_g2[MIXER_NUM_MOTORS]; // RC PWM
extern fix16_t _actuator_control_g3[MIXER_NUM_MOTORS]; // RC Digital
extern fix16_t _actuator_control_g4[MIXER_NUM_MOTORS]; // OB PWM
extern fix16_t _actuator_control_g5[MIXER_NUM_MOTORS]; // OB Digital

extern mixer_motor_test_t _motor_test;

//extern int32_t _pwm_output[MIXER_NUM_MOTORS];
extern const mixer_t* _mixer_to_use;

void mixer_clear_outputs( void );

void mixer_init( void );

// void write_output_pwm(uint8_t index, uint32_t value, uint32_t value_disarm);
// void write_motor(uint8_t index, uint32_t value);
// void write_servo(uint8_t index, int32_t value);

void calc_mixer_output(void);
void mixer_output( uint32_t time_now );

#ifdef __cplusplus
}
#endif
