#include <stdbool.h>
#include <stdint.h>

#include "breezystm32.h"

#include "drv_status_io.h"
#include "safety.h"

static GPIO_TypeDef *gpio_p_arm;
static uint16_t pin_arm;

GPIO_TypeDef *gpio_p_heart;
static uint16_t pin_heart;

GPIO_TypeDef *gpio_p_buzzer;
static uint16_t pin_buzzer;

static status_led_t led_arm;
static status_led_t led_heart;
static status_buzzer_t buzzer;

static void status_led_arm_init( void ) {
	gpio_p_arm = LED1_GPIO;
	pin_arm = LED1_PIN;

	led_arm.period_us = 500000;
	led_arm.length_us = 250000;
	led_arm.last_pulse = 0;
}

static void status_led_heart_init( void ) {
	gpio_p_heart = LED0_GPIO;
	pin_heart = LED0_PIN;

	led_heart.period_us = 1000000;
	led_heart.length_us = 250000;
	led_heart.last_pulse = 0;
}

static void status_buzzer_init() {
	//Buzzer
	gpio_p_buzzer = GPIOA;
	pin_buzzer = Pin_12;

	gpio_config_t safety_buzzer_cfg;
    safety_buzzer_cfg.pin = pin_buzzer;
    safety_buzzer_cfg.mode = Mode_Out_PP;
    safety_buzzer_cfg.speed = Speed_2MHz;
    gpioInit(gpio_p_buzzer, &safety_buzzer_cfg);

	buzzer.num_beeps = 0;	//Number of beeps to make
	buzzer.period = 0;		//200ms beep length
	buzzer.last_beep = 0;	//Time last beep started

    digitalLo( gpio_p_buzzer, pin_buzzer );

	//TODO: Make this play a startup tune
}

void status_devices_init( void ) {
	status_led_arm_init();

	status_led_heart_init();

	status_buzzer_init();
}

void status_led_arm_set( bool on ) {
	if( on ) {
		digitalLo(gpio_p_arm, pin_arm);	//On
	} else {
		digitalHi(gpio_p_arm, pin_arm);	//Off
	}
}

void status_led_heart_set( bool on ) {
	if( on ) {
		digitalLo(gpio_p_heart, pin_heart);	//On
	} else {
		digitalHi(gpio_p_heart, pin_heart);	//Off
	}
}

static void status_buzzer_set( int8_t num, uint32_t period ) {
	buzzer.last_beep = 0;	//Time last beep started

	buzzer.num_beeps = 2*num;	//double to get the correct on/off states
	buzzer.period = period;

    digitalLo( gpio_p_buzzer, pin_buzzer );
}

void status_buzzer_success( void ) {
	//Play 2 quick beeps
	status_buzzer_set(2, 100000);
}

void status_buzzer_failure( void ) {
	//Play 1 long beep
	status_buzzer_set(1, 1000000);
}

static void status_buzzer_update( uint32_t time_now ) {
	//If the system is in a failsafe mode
	//	and the buzzer isn't set to make a beep
	if( ( buzzer.num_beeps == 0 ) &&
		( (_system_status.state == MAV_STATE_CRITICAL ) ||
	      (_system_status.state == MAV_STATE_EMERGENCY ) ) ) {

		status_buzzer_set(1, 100000);
	}

	if( ( buzzer.num_beeps > 0 ) &&
		( ( time_now - buzzer.last_beep ) > buzzer.period ) ) {

		digitalToggle(gpio_p_buzzer, pin_buzzer);

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
		status_led_arm_set(true);
	} else if( !safety_switch_engaged() ) {
		status_led_arm_set( status_led_calc_pulse( time_now, &led_arm ) );
	} else {
		status_led_arm_set(false);
	}

	//System heartbeat LED
	status_led_heart_set( status_led_calc_pulse( time_now, &led_heart ) );
}



void status_devices_run( uint32_t time_now ) {
	//Update LED flashes to match system state
	status_led_update( time_now );

	//Update buzzer to see if it should be making any noise
	status_buzzer_update( time_now );
}
