#include <stdbool.h>

#include "breezystm32.h"
#include "drivers/drv_sensors.h"

static GPIO_TypeDef* gpio_p_safety;
static uint16_t pin_safety;

bool drv_sensors_safety_button_init( void ) {
	gpio_p_safety = GPIOA;
	pin_safety = Pin_6;

	gpio_config_t safety_button_cfg;
	safety_button_cfg.pin = pin_safety;
	safety_button_cfg.mode = Mode_IPU;
	safety_button_cfg.speed = Speed_2MHz;
	gpioInit( gpio_p_safety, &safety_button_cfg );

	return true;
}

bool drv_sensors_safety_button_read( void ) {
	return digitalIn( gpio_p_safety, pin_safety );
}
