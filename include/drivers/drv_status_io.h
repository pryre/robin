#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"
#include "fixextra.h"

typedef enum {
	STATUS_BUZZER_QUIET,
	STATUS_BUZZER_SUCCESS,
	STATUS_BUZZER_FAILURE,
	STATUS_BUZZER_FAILSAFE
} status_buzzer_modes_t;

typedef struct {
	uint32_t period_us;
	uint32_t length_us;
	uint32_t last_pulse;
	bool state;
} status_led_t;

typedef struct {
	uint8_t num_beeps;
	uint32_t period;
	uint32_t last_beep;
	bool state;
} status_buzzer_t;

#define IO_PIN_STATE_GROUP_MIX 255

typedef enum {
	IO_PIN_STATE_ID_HEART = 0,
	IO_PIN_STATE_ID_ARM,
	IO_PIN_STATE_ID_BUZZER,
	IO_PIN_STATE_NUM
} status_io_pin_states_t;

extern fix16_t _io_pin_states[IO_PIN_STATE_NUM];

// XXX: status_devices_init() calls the other init() to do hardware specific
// setup
void status_devices_init( void );
void status_led_arm_init( void );
void status_led_heart_init( void );
void status_buzzer_init( void );

void status_led_arm_set( bool on );
void status_led_heart_set( bool on );
void status_buzzer_set( bool on );

void status_buzzer_success( void );
void status_buzzer_failure( void );
// void status_buzzer_set( int8_t num, uint32_t period );

void status_devices_run( uint32_t time_now );

#ifdef __cplusplus
}
#endif
