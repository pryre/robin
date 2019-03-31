#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRV_PWM_MAX_OUTPUTS 8

#define DRV_PWM_MIN 1000		//Minimum output
#define DRV_PWM_MID 1500		//Middle output
#define DRV_PWM_MAX 2000		//Maximum output

#define DRV_PWM_DIGITAL_OFF 1000
#define DRV_PWM_PULSE_DIGITAL_ON 2000

bool drv_pwm_init( void );
void drv_pwm_write( uint8_t index, uint16_t value );	//Set the current PWM output
uint16_t drv_pwm_get_current( uint8_t index );			//Read the current PWM output

#ifdef __cplusplus
}
#endif
