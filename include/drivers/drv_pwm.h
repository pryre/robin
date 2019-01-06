#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "io_type.h"

#define MAX_SERVOS 8
#define MAX_INPUTS 8
#define PULSE_MIN ( 750 )  // minimum PWM pulse width which is considered valid
#define PULSE_MAX ( 2250 ) // maximum PWM pulse width which is considered valid
#define PULSE_DIGITAL_OFF 1000
#define PULSE_DIGITAL_ON 2000

void pwmInit( io_def_t* io_map, bool usePwmFilter, uint32_t motorPwmRate, uint32_t servoPwmRate, uint16_t idlePulseUsec );
void pwmWriteMotor( uint8_t index, uint16_t value );
uint16_t pwmRead( uint8_t channel );

#ifdef __cplusplus
}
#endif
