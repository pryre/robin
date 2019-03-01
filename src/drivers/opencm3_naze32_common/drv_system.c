#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_system.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>	//Needed for AFIO_MAPR
#include <libopencm3/cm3/systick.h>	//Needed for clock
#include <libopencm3/cm3/scb.h>		//Needed for resets
#include <libopencm3/cm3/nvic.h>	//Needed for sys_tick_handler()

static volatile uint32_t time_secs;
static volatile uint32_t time_usecs;

void sys_tick_handler(void) {
	time_usecs += 10;

	if (time_usecs >= 1000000) {
		time_secs++;
		time_usecs -= 1000000;

		gpio_clear(GPIOB, GPIO3 | GPIO4);
	}
}

static void clock_setup(void) {
	// Set STM32 to 72 MHz.
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// Enable GPIO Clocks
	//rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO3 | GPIO4);

	gpio_set(GPIOB, GPIO3 | GPIO4);

	// Diable JTAG IO, used for status LEDs on Naze32
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

	// Do SysTick setup
	time_secs = 0;
	time_usecs = 0;
	// 72MHz / 8 => 9000000 counts per second
	//systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	// 9000000/9000 = 100000 overflows per second - every 1ms one interrupt
	// SysTick interrupt every N clock pulses: set reload to N-1
	//systick_set_reload(8999);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(719);	//overflows at 10us
	systick_interrupt_enable();
	systick_counter_enable();	// Start counting.
}

void system_init( void ) {
	system_debug_print( "--== robin ==--" );

	clock_setup();

	while (1); // Halt.
}

uint32_t system_micros( void ) {
	return (1000000 * time_secs) + time_usecs;
}


void system_pause_us(uint32_t us) {
    uint32_t now = system_micros();

    while (system_micros() - now < us)
		__asm__("nop");
}

void system_pause_ms( uint32_t ms ) {
    while (ms--)
        system_pause_us(1000);
}

void system_reset( void ) {
	scb_reset_system();
}

void system_bootloader( void ) {
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC

    *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
	system_reset();
}

uint16_t system_vendor_id( void ) {
	// TODO: This is the serial vendor and product ID, should be dynamic?
	return 0x10c4;
}

uint16_t system_product_id( void ) {
	// TODO: This is the serial vendor and product ID, should be dynamic?
	return 0xea60;
}

uint64_t system_unique_id( void ) {
	return 0;
}

int system_debug_print( const char* format, ... ) {
	// XXX: Currently unsupported
	/*
	int result;
	va_list args;

	va_start(args, format);
	result = vprintf(format, args);
	va_end(args);

	return result;
	*/

	return 0;
}

void system_rate_limit( void ) {
	// XXX: Not required for naze32
}
