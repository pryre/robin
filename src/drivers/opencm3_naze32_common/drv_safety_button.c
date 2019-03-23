#include <stdbool.h>

#include "drivers/drv_sensors.h"

#include <libopencm3/stm32/gpio.h>

#define GPIO_SAFETY GPIOA
#define PIN_SAFETY GPIO6

bool drv_sensors_safety_button_init( void ) {
	//Set safety button for input pull-up
	gpio_set_mode(GPIO_SAFETY, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, PIN_SAFETY);
	gpio_set(GPIO_SAFETY, PIN_SAFETY);

	return true;
}

bool drv_sensors_safety_button_read( void ) {
	return gpio_get(GPIO_SAFETY, PIN_SAFETY);
}
