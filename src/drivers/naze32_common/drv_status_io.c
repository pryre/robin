#include <stdbool.h>
#include <stdint.h>

#include "breezystm32.h"

#include "drivers/drv_status_io.h"

static GPIO_TypeDef *gpio_p_arm;
static uint16_t pin_arm;

GPIO_TypeDef *gpio_p_heart;
static uint16_t pin_heart;

GPIO_TypeDef *gpio_p_buzzer;
static uint16_t pin_buzzer;

void status_led_arm_init( void ) {
	gpio_p_heart = LED1_GPIO;
	pin_heart = LED1_PIN;

	status_led_arm_set(false);
}

void status_led_heart_init( void ) {
	gpio_p_heart = LED0_GPIO;
	pin_heart = LED0_PIN;

	status_led_heart_set(false);
}

void status_buzzer_init() {
	//Buzzer
	gpio_p_buzzer = GPIOA;
	pin_buzzer = Pin_12;

	gpio_config_t safety_buzzer_cfg;
    safety_buzzer_cfg.pin = pin_buzzer;
    safety_buzzer_cfg.mode = Mode_Out_PP;
    safety_buzzer_cfg.speed = Speed_2MHz;
    gpioInit(gpio_p_buzzer, &safety_buzzer_cfg);

	status_buzzer_set(false);
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

void status_buzzer_set( bool on ) {
	if( on ) {
		digitalLo(gpio_p_buzzer, pin_buzzer);	//On
	} else {
		digitalHi(gpio_p_buzzer, pin_buzzer);	//Off
	}
}
