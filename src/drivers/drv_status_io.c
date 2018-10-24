#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_status_io.h"
#include "safety.h"

#include "fix16.h"
#include "fixextra.h"

fix16_t _io_pin_states[8];

static status_led_t led_arm;
static status_led_t led_heart;
static status_buzzer_t buzzer;

void status_devices_init( void ) {
	status_led_arm_init();
	status_led_heart_init();
	status_buzzer_init();

	led_arm.period_us = 500000;
	led_arm.length_us = 250000;
	led_arm.last_pulse = 0;
	led_arm.state = false;

	led_heart.period_us = 1000000;
	led_heart.length_us = 250000;
	led_heart.last_pulse = 0;
	led_heart.state = false;

	buzzer.num_beeps = 0;	//Number of beeps to make
	buzzer.period = 0;		//200ms beep length
	buzzer.last_beep = 0;	//Time last beep started

	for(int i=0; i<8; i++)
		_io_pin_states[i] = 0;
}

static void status_buzzer_new_beep_set( int8_t num, uint32_t period ) {
	buzzer.last_beep = 0;	//Time last beep started

	buzzer.num_beeps = 2*num;	//double to get the correct on/off states
	buzzer.period = period;

    status_buzzer_set( false );
	buzzer.state = false;
}

void status_buzzer_success( void ) {
	//Play 2 quick beeps
	status_buzzer_new_beep_set(2, 100000);
}

void status_buzzer_failure( void ) {
	//Play 1 long beep
	status_buzzer_new_beep_set(1, 1000000);
}

static void status_buzzer_update( uint32_t time_now ) {
	//If the system is in a failsafe mode
	//	and the buzzer isn't set to make a beep
	if( ( buzzer.num_beeps == 0 ) &&
		( (_system_status.state == MAV_STATE_CRITICAL ) ||
	      (_system_status.state == MAV_STATE_EMERGENCY ) ) ) {

		status_buzzer_new_beep_set(1, 100000);
	}

	if( ( buzzer.num_beeps > 0 ) &&
		( ( time_now - buzzer.last_beep ) > buzzer.period ) ) {

		buzzer.state = !buzzer.state;
		status_buzzer_set(buzzer.state);

		buzzer.last_beep = time_now;
		buzzer.num_beeps--;
	}
}

static bool status_led_calc_pulse( uint32_t time_now, status_led_t *led ) {
	bool turn_on = false;

	if( ( time_now - led->last_pulse ) > led->period_us )
		led->last_pulse = time_now;

	if( ( time_now - led->last_pulse ) < led->length_us )
		turn_on = true;

	return turn_on;
}

static void status_led_update( uint32_t time_now ) {
	// Safety LED
	if( safety_is_armed() ) {
		led_arm.state = true;
	} else if( !safety_switch_engaged() ) {
		led_arm.state = status_led_calc_pulse( time_now, &led_arm );
	} else {
		led_arm.state = false;
	}

	//System heartbeat LED
	led_heart.state = status_led_calc_pulse( time_now, &led_heart );

	//Actually set the pin states
	status_led_heart_set(led_heart.state);
	status_led_arm_set(led_arm.state);
}

void status_devices_run( uint32_t time_now ) {
	//Update LED flashes to match system state
	status_led_update( time_now );

	//Update buzzer to see if it should be making any noise
	status_buzzer_update( time_now );

	//Update pin state data
	_io_pin_states[IO_PIN_STATE_ID_HEART] = (led_heart.state) ? _fc_1 : 0;
	_io_pin_states[IO_PIN_STATE_ID_ARM] = (led_arm.state) ? _fc_1 : 0;
	_io_pin_states[IO_PIN_STATE_ID_BUZZER] = (buzzer.state) ? _fc_1 : 0;
}
