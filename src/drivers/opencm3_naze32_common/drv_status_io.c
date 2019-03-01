#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_status_io.h"

#include <libopencm3/stm32/gpio.h>

/*
#include "breezystm32.h"

static GPIO_TypeDef* gpio_p_arm;
static uint16_t pin_arm;

GPIO_TypeDef* gpio_p_heart;
static uint16_t pin_heart;

GPIO_TypeDef* gpio_p_buzzer;
static uint16_t pin_buzzer;
*/

#define GPIO_ARM GPIOB
#define GPIO_HEART GPIOB
#define GPIO_BUZZER GPIOA

#define PIN_ARM GPIO4
#define PIN_HEART GPIO3
#define PIN_BUZZER GPIO12

void status_led_arm_init( void ) {
	gpio_set_mode(GPIO_ARM, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, PIN_ARM);

	status_led_arm_set( false );
}

void status_led_heart_init( void ) {
	gpio_set_mode(GPIO_HEART, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, PIN_HEART);

	status_led_heart_set( false );
}

void status_buzzer_init() {
	gpio_set_mode(GPIO_BUZZER, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, PIN_BUZZER);

	status_buzzer_set( false );
}

void status_led_arm_set( bool on ) {
	// XXX: LED Hi/Lo is backwards
	if ( on ) {
		gpio_clear(GPIO_ARM, PIN_ARM);	// On
	} else {
		gpio_set(GPIO_ARM, PIN_ARM);	// Off
	}
}

void status_led_heart_set( bool on ) {
	// XXX: LED Hi/Lo is backwards
	if ( on ) {
		gpio_clear(GPIO_HEART, PIN_HEART);	// On
	} else {
		gpio_set(GPIO_HEART, PIN_HEART);	// Off
	}
}

void status_buzzer_set( bool on ) {
	if ( on ) {
		gpio_set(GPIO_BUZZER, PIN_BUZZER);		// On
	} else {
		gpio_clear(GPIO_BUZZER, PIN_BUZZER);	// Off
	}
}
