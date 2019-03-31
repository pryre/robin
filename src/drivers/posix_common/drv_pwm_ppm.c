#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_pwm.h"

//Record of the current PWM output
static uint16_t pwm_frame_[DRV_PWM_MAX_OUTPUTS];

bool drv_pwm_init(void) {
	for(int i=0; i<DRV_PWM_MAX_OUTPUTS; i++)
		drv_pwm_write(i,0);

	return true;
}

void drv_pwm_write( uint8_t index, uint16_t value ) {
	if(index < DRV_PWM_MAX_OUTPUTS)
		pwm_frame_[DRV_PWM_MAX_OUTPUTS] = value;
}

uint16_t drv_pwm_get_current( uint8_t index ) {
	return ( (index < DRV_PWM_MAX_OUTPUTS) ? pwm_frame_[index] : 0 );
}

bool drv_ppm_init(void) {
	return false;
}

bool drv_ppm_ready( void ) {
	return false;
}

bool drv_ppm_read_frame( const uint16_t *frame ) {
	return false;
}
