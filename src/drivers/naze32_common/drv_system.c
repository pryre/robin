#include <stdbool.h>
#include <stdint.h>

#include "breezystm32.h"
#include "adc.h"
#include "drivers/drv_system.h"

extern void SetSysClock(bool overclock);

void system_init(void) {
	system_debug_print("--== robin ==--");

	SetSysClock(false);

	systemInit();

	adcInit(false);

	system_pause_ms(100);
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

uint16_t system_vendor_id(void) {
	//TODO: This is the serial vendor and product ID, should be dynamic?
	return 0x10c4;
}

uint16_t system_product_id(void) {
	//TODO: This is the serial vendor and product ID, should be dynamic?
	return 0xea60;
}

uint64_t system_unique_id(void) {
	return U_ID_0;
}


int system_debug_print( const char *format, ... ) {
	//XXX: Currently unsupported
	/*
	int result;
    va_list args;

    va_start(args, format);
    result = vprintf(format, args);
    va_end(args);

    return result;
	*/
}

void system_rate_limit(void) {
	//XXX: Not required for naze32
}

