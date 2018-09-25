#include <stdbool.h>
#include <stdint.h>

#include "breezystm32.h"
#include "adc.h"
#include "drivers/drv_system.h"

extern void SetSysClock(bool overclock);

void system_init(void) {
    SetSysClock(false);

    systemInit();

	adcInit(false);
}

uint32_t system_micros( void ) {
	return micros();
}

void system_pause_ms( uint32_t ms ) {
	delay(ms);
}

void system_reset(void) {
	systemReset();
}

void system_bootloader(void) {
	systemResetToBootloader();
}
