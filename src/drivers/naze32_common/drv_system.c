#include <stdbool.h>

#include "breezystm32.h"
#include "drv_system.h"

extern void SetSysClock(bool overclock);

void system_init(void) {
    SetSysClock(false);

    systemInit();
}

uint32_t system_micros( void ) {
	return micros();
}

void system_reset(void) {
	systemReset();
}

void system_bootloader(void) {
	systemResetToBootloader();
}
