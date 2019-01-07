#include <stdbool.h>

#include "breezystm32.h"
#include "drivers/drv_pwm.h"
#include "drivers/drv_sensors.h"

bool drv_sensors_rc_input_init( void ) {
	return true; // XXX: This is init with pwm_init()
}

void drv_sensors_rc_input_read( uint16_t* readings ) {
	for ( int i = 0; i < MAX_INPUTS; i++ ) {
		readings[i] = pwmRead( i );
	}
}
