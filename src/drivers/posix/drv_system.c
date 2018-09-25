#include <stdint.h>

#include "drivers/drv_system.h"

#include <time.h>
#include <stdio.h>

void system_init(void) {
	system_debug_print("--== robin ==--");
}

uint32_t system_micros(void) {
	return 0;
}

void system_pause_ms(uint32_t ms) {
	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 1000000 * ms;	//millis to nanos

	nanosleep(&sleep_time, NULL);
}

void system_reset(void) {

}

void system_bootloader(void) {

}

void system_debug_print( char *msg ) {
	printf( msg );
	printf( "\n" );
}

void system_rate_limit(void) {
	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 10000;	//sleep for 0.01 millis

	nanosleep(&sleep_time, NULL);
}
