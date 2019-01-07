#include <stdbool.h>

#include "adc.h"
#include "breezystm32.h"
#include "drivers/drv_sensors.h"

bool drv_sensors_battery_monitor_init( void ) {
	return true;
}

uint16_t drv_sensors_battery_monitor_read( void ) {
	return adcGetChannel( ADC_EXTERNAL_PAD );
}
