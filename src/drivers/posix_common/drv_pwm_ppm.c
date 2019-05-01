#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_pwm.h"
#include "drivers/drv_ppm.h"

//Record of the current PWM output
static uint16_t pwm_frame_[DRV_PWM_MAX_OUTPUTS];
//Record of the current PPM input
static uint16_t ppm_frame_[DRV_PPM_MAX_INPUTS];
static bool ppm_frame_available_;


//=======================================================================
//	PWM Driver
//=======================================================================

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


//=======================================================================
//	PPM Driver
//=======================================================================

static void drv_ppm_set_ready( bool ready ) {
	ppm_frame_available_ = ready;
}

bool drv_ppm_init(void) {
	ppm_frame_available_ = false;

	return false;
}

bool drv_ppm_ready( void ) {
	return ppm_frame_available_;
}

bool drv_ppm_read_frame( uint16_t *frame ) {
	bool success = false;

	if( drv_ppm_ready() ) {
		for(int i=0; i<DRV_PPM_MAX_INPUTS; i++)
			frame[i] = ppm_frame_[i];

		drv_ppm_set_ready(false);

		success = true;
	}

	return success;
}
