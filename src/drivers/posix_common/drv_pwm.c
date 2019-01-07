#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_pwm.h"
#include "io_type.h"

void pwmInit( io_def_t* io_map, bool usePwmFilter, uint32_t motorPwmRate,
			  uint32_t servoPwmRate, uint16_t idlePulseUsec ) {
}

void pwmWriteMotor( uint8_t index, uint16_t value ) {
}

uint16_t pwmRead( uint8_t channel ) {
	return 0;
}
