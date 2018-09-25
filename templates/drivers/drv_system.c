#include <stdint.h>

#include "drivers/drv_system.h"

void system_init(void) {
	system_debug_print("--== robin ==--");
}

uint32_t system_micros(void) {
	return 0;
}

void system_pause_ms(uint32_t ms) {

}

void system_reset(void) {

}

void system_bootloader(void) {

}

void system_debug_print( char *msg ) {

}

void system_rate_limit(void) {

}
