#include <stdint.h>

#include "drivers/drv_system.h"

#include <time.h>

void system_init(void) {

}

uint32_t system_micros(void) {
	return 0;
}

void system_pause_ms(uint32_t ms) {
	timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 1000000 * ms;	//millis to nanos

	nanosleep(&sleep_time, NULL);
}

void system_reset(void) {

}

void system_bootloader(void) {

}
