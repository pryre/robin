#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MAX_SERVOS 8
#define MAX_INPUTS 8

#define PULSE_MIN ( 750 )  // minimum PWM pulse width which is considered valid
#define PULSE_LOW ( 1000 ) // maximum PWM pulse width which is considered valid
#define PULSE_MID ( 1500 ) // maximum PWM pulse width which is considered valid
#define PULSE_HIG ( 2000 ) // maximum PWM pulse width which is considered valid
#define PULSE_MAX ( 2250 ) // maximum PWM pulse width which is considered valid

#define PULSE_DIGITAL_OFF 1000
#define PULSE_DIGITAL_ON 2000

//We need a 50 Hz period (1000 / 20ms = 50)
//thus devide 1000000 by 50 = 20000 (us).
#define DRV_PWM_PERIOD_50HZ 20000
#define DRV_PWM_PERIOD_400HZ 2500

void drv_pwm_init( void );
void drv_pwm_write( uint8_t index, uint16_t value );
uint16_t drv_pwm_read( uint8_t index );

#ifdef __cplusplus
}
#endif
