#pragma once

#include <stdbool.h>
#include <stdint.h>

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
} status_led_t;

typedef struct {
	uint8_t num_beeps;
	uint32_t period;
	uint32_t last_beep;
} status_buzzer_t;


void status_devices_init( void );

void status_led_arm_set( bool on );
void status_led_heart_set( bool on );

void status_buzzer_success( void );
void status_buzzer_failure( void );
//void status_buzzer_set( int8_t num, uint32_t period );

void status_devices_run( uint32_t time_now );
