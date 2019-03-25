#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_system.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>	//Needed for AFIO_MAPR
#include <libopencm3/cm3/systick.h>	//Needed for clock
#include <libopencm3/cm3/scb.h>		//Needed for resets
#include <libopencm3/cm3/nvic.h>	//Needed for sys_tick_handler()


#define BOOT_SYSCHK_VAL 0xDEADBEEF
#define BOOT_JUMP_ADDR 0x1FFFC400UL

#ifdef STM32F1
#define BOOT_SYSCHK_ADDR 0x20004FF0 // 20KB STM32F103
#define SYSTICK_ROLLOVER 9000
#endif


static volatile uint32_t uptime_ms_;
static uint32_t us_ticks_;

void sys_tick_handler(void) {
	//Called every 1ms, 49 day rollover
	uptime_ms_++;
}

static void rcc_setup(void) {
	// Set STM32 to 72 MHz.
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	//XXX: rcc_clock_setup_in_hse_16mhz_out_72mhz();

	// Enable GPIO clocks.
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

	// Diable JTAG IO, used for status LEDs on Naze32
	//AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF;
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);
}

static void clock_setup(void) {
	// Do SysTick setup
	uptime_ms_ = 0;
	// 72MHz / 8 => 9000000 counts per second
	//systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	// 9000000/9000 = 1000 overflows per second - every 1ms one interrupt
	// SysTick interrupt every N clock pulses: set reload to N-1
	//systick_set_reload(8999);
	//XXX: This means that systick will be counting at 9MHz
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	us_ticks_ = 9;	//systick will count 9 ticks per us (9000000 / 1000000)
	systick_set_reload(SYSTICK_ROLLOVER-1);	//overflows at 1ms	//TODO: Need to see if this can be dropped to 1us
	systick_clear();
	systick_interrupt_enable();
	systick_counter_enable();	// Start counting.
}
/*
//===========================================================
// Taken from devanlai from project dap42
//===========================================================

static inline void __set_MSP(uint32_t topOfMainStack) {
    asm("msr msp, %0" : : "r" (topOfMainStack));
}

static void jump_to_bootloader(void) __attribute__ ((noreturn));

// Sets up and jumps to the bootloader
static void jump_to_bootloader(void) {
	uint32_t boot_stack_ptr = *(uint32_t*)(BOOT_JUMP_ADDR);
	uint32_t dfu_reset_addr = *(uint32_t*)(BOOT_JUMP_ADDR+4);

	//Do some trickery to allow function pointer to "object" pointer
	void (*dfu_bootloader)(void);
	*(void **) (&dfu_bootloader) = (void (*))(dfu_reset_addr);


	// Reset the stack pointer
	__set_MSP(boot_stack_ptr);

	dfu_bootloader();
	while (1);
}
//===========================================================

static void check_reboot_bootloader(void) {
#ifdef BOOT_SYSCHK_ADDR
	if( (*((uint32_t *)BOOT_SYSCHK_ADDR)) == BOOT_SYSCHK_VAL )
		jump_to_bootloader();
#endif
}
*/

void system_init( void ) {
	system_debug_print( "--== robin ==--" );

	rcc_setup();
	clock_setup();
}

uint32_t system_micros( void ) {
    register uint32_t ms, cycle_cnt;

	//Basically do a read, and then make sure that the clock didn't roll over
	//If it did, take another reading
    do {
        ms = uptime_ms_;
        cycle_cnt = systick_get_value();
    } while (ms != uptime_ms_);

    return (ms * 1000) + (us_ticks_ * 1000 - cycle_cnt) / us_ticks_;
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
/*
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC

#ifdef BOOT_SYSCHK_ADDR
    *((uint32_t *)BOOT_SYSCHK_ADDR) = BOOT_SYSCHK_VAL;
	system_reset();
#else
	mavlink_queue_broadcast_error( "[SYS] Unable to enter bootloader, undefined address" );
#endif
*/
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
