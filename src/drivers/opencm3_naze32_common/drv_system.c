#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_system.h"
#include "drivers/opencm3_naze32_common/drv_clock_select.h"
#include "drivers/drv_status_io.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>	//Needed for AFIO_MAPR
#include <libopencm3/cm3/systick.h>	//Needed for clock
#include <libopencm3/cm3/scb.h>		//Needed for resets
#include <libopencm3/cm3/nvic.h>	//Needed for sys_tick_handler()


#ifdef STM32F1
#define BOOT_SYSCHK_ADDR 0x20004E1C	// 20KB STM32F103, pick an address near the end
#define BOOT_SYSCHK_VAL 0xDEADBEEF

#define BOOT_ADDR_P 0x1FFFF000
#define BOOT_ADDR 0x1FFFF004

#define SYSTICK_ROLLOVER 9000
#endif


static volatile uint32_t uptime_ms_;
static uint32_t us_ticks_;

void sys_tick_handler(void) {
	//Called every 1ms, 49 day rollover
	uptime_ms_++;
}

static void rcc_setup(void) {
	// Enable GPIO clocks.
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

	// Diable JTAG IO, used for status LEDs on Naze32
	//AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF;
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);

	hs_clock_select();
}

static void rcc_shutdown(void) {
	// Disable GPIO clocks.
	rcc_periph_clock_disable(RCC_GPIOA);
	rcc_periph_clock_disable(RCC_GPIOB);
	rcc_periph_clock_disable(RCC_GPIOC);
	rcc_periph_clock_disable(RCC_AFIO);
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
	systick_set_reload(SYSTICK_ROLLOVER-1);	//overflows at 1ms
	systick_clear();
	systick_interrupt_enable();
	systick_counter_enable();	// Start counting.
}

static inline void __set_MSP(uint32_t topOfMainStack) {
	//  Set the stack pointer.
	__asm__("msr msp, %0" : : "r" (topOfMainStack));
}

static void system_check_bootjump(void) {
    void(*bootJump)(void);

	if (*((uint32_t *)BOOT_SYSCHK_ADDR) == BOOT_SYSCHK_VAL) {
		//Quickly put LEDs on so we can signify bootloader mode
		status_led_arm_init();
		status_led_heart_init();
		status_led_arm_set(true);
		status_led_heart_set(true);

		//Shutdown RCC peripherals
		rcc_shutdown();

		//Clear the boot check address
        *((uint32_t *)BOOT_SYSCHK_ADDR) = 0x0;

        //__enable_irq();

		//Jump time!
	    // 1FFFF000 -> 20000200 -> SP
	    // 1FFFF004 -> 1FFFF021 -> PC
        __set_MSP(*((uint32_t *)BOOT_ADDR_P));
        bootJump = (void(*)(void))(*((uint32_t *) BOOT_ADDR));
        bootJump();

		//Catch here in case the bootloader jumps back for some reason
        while (1);
    }
}

void system_init( void ) {
	rcc_setup();

	//Bootloader jump Should be done as quickly as possible after main / rcc_setup
	system_check_bootjump();

	system_debug_print( "--== robin ==--" );

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

void system_bootloader(void) {
    *((uint32_t *)BOOT_SYSCHK_ADDR) = BOOT_SYSCHK_VAL;
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
