#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRV_PWM_MAX_OUTPUTS 8
#define DRV_PWM_MAX_INPUTS 8
//#define DRV_PWM_PULSE_MIN ( 750 )  // minimum PWM pulse width which is considered valid
//#define DRV_PWM_PULSE_MAX ( 2250 ) // maximum PWM pulse width which is considered valid

#define DRV_PWM_PPM_MIN 900
#define DRV_PWM_PPM_MAX 2100
#define DRV_PWM_PPM_SYNC_MIN 5000
#define DRV_PWM_PPM_SYNC_MAX 15000

#define DRV_PWM_MIN 1000		//Minimum output
#define DRV_PWM_MID 1500		//Middle output
#define DRV_PWM_MAX 2000		//Maximum output

#define DRV_PWM_DIGITAL_OFF 1000
#define PULSE_DIGITAL_ON 2000

void drv_pwm_init( void );
void drv_pwm_write( uint8_t index, uint16_t value );

bool drv_pwm_ppm_ready( void );
bool drv_pwm_ppm_read_frame( uint16_t *frame );	//XXX: Frame should be of size DRV_PWM_MAX_INPUTS

#ifdef __cplusplus
}
#endif
