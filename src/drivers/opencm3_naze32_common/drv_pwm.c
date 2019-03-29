#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "drivers/drv_pwm.h"
#include "io_type.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <stdint.h>

typedef struct {
	uint32_t timer_peripheral;
	enum tim_oc_id oc_id;
	uint32_t gpio;
	uint32_t pin;
} drv_pwm_map_t;

static void drv_pwm_init_timer(uint32_t rcc_tim,
							   uint32_t tim,
							   uint32_t reset,
							   uint32_t prescaler,
							   uint32_t period) {

	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(rcc_tim);

	/* Enable TIM2 interrupt. */
	//XXX: nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(reset);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(tim, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	timer_set_prescaler(tim, prescaler);

	/* Disable preload. */
	timer_disable_preload(tim);
	timer_continuous_mode(tim);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(tim, period);

	/* Set the initual output compare value for OC1. */
	//timer_set_oc_value(tim, TIM_OC1, frequency_sequence[frequency_sel++]);

	/* Counter enable. */
	timer_enable_counter(tim);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

static void drv_pwm_init_output_channel(uint32_t timer_peripheral,
										enum tim_oc_id oc_id,
										uint32_t gpio_port,
										uint16_t gpio_pin) {
     /* Set timer channel to output */
     gpio_set_mode(gpio_port, GPIO_MODE_OUTPUT_50_MHZ,
                   GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                   gpio_pin);

     timer_disable_oc_output(timer_peripheral, oc_id);
     timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
     timer_set_oc_value(timer_peripheral, oc_id, 0);
     timer_enable_oc_output(timer_peripheral, oc_id);
}

static void drv_pwm_set_pulse_width(uint32_t timer_peripheral,
									enum tim_oc_id oc_id,
									uint32_t pulse_width) {

     timer_set_oc_value(timer_peripheral, oc_id, pulse_width);
}

static void pwm_start_timer(uint32_t timer_peripheral) {
     timer_enable_counter(timer_peripheral);
}

//=======================================================================

/* FreeFlight/Naze32 timer layout
    Primary IO
	Name	Timer		Pin		Function1	Function2
    PWM1	TIM1_CH1	PA8		PWM1
    PWM2	TIM1_CH4	PA11	PWM2
    PWM3	TIM4_CH1	PB6		PWM3		I2C1_SCL
    PWM4	TIM4_CH2	PB7		PWM4		I2C1_SDA
    PWM5	TIM4_CH3	PB8		PWM5
    PWM6	TIM4_CH4	PB9		PWM6

	Secondary IO
	Name	Timer		Pin		Function1	Function2
    RX1		TIM2_CH1	PA0		PPM
    RX2		TIM2_CH2	PA1
    RX3		TIM2_CH3	PA2		UART2_TX
    RX4		TIM2_CH4	PA3		UART2_RX
    RX5		TIM3_CH1	PA6		SAFETY		ADC_IN6
    RX6		TIM3_CH2	PA7					ADC_IN7
    RX7		TIM3_CH3	PB0		PWM7		ADC_IN8
    RX8		TIM3_CH4	PB1		PWM8		ADC_IN9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/

static const drv_pwm_map_t drv_pwm_map[MAX_SERVOS] = {
	{TIM1,	TIM_OC1,	GPIOA,	GPIO8},
	{TIM1,	TIM_OC4,	GPIOA,	GPIO11},
	{TIM4,	TIM_OC1,	GPIOB,	GPIO6},
	{TIM4,	TIM_OC2,	GPIOB,	GPIO7},
	{TIM4,	TIM_OC3,	GPIOB,	GPIO8},
	{TIM4,	TIM_OC4,	GPIOB,	GPIO9},
	{TIM3,	TIM_OC3,	GPIOB,	GPIO0},
	{TIM3,	TIM_OC4,	GPIOB,	GPIO1},
};

void drv_pwm_init( void ) {
	// Init all our timers to run at 1us intervals, overflow at PWM_PERIOD
	drv_pwm_init_timer(RCC_TIM1, TIM1, RST_TIM1, (rcc_apb1_frequency / 1000000), DRV_PWM_PERIOD_50HZ);
	drv_pwm_init_timer(RCC_TIM2, TIM2, RST_TIM2, (rcc_apb2_frequency / 1000000), DRV_PWM_PERIOD_50HZ);
	drv_pwm_init_timer(RCC_TIM3, TIM3, RST_TIM3, (rcc_apb2_frequency / 1000000), DRV_PWM_PERIOD_50HZ);
	drv_pwm_init_timer(RCC_TIM4, TIM4, RST_TIM4, (rcc_apb2_frequency / 1000000), DRV_PWM_PERIOD_50HZ);

	/* init output of channel2 of timer2 */
	//drv_pwm_init_output_channel(TIM2, SERVO_CH1, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH2);

	/* init output of channel3 of timer2 */
	//pwm_init_output_channel(TIM2, SERVO_CH2, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH3);

	//pwm_set_pulse_width(TIM2, SERVO_CH1, SERVO_NULL);
	//pwm_set_pulse_width(TIM2, SERVO_CH2, SERVO_NULL);

	for(uint32_t i=0; i< MAX_SERVOS; i++) {
		drv_pwm_init_output_channel(drv_pwm_map[i].timer_peripheral,
									drv_pwm_map[i].oc_id,
									drv_pwm_map[i].gpio,
									drv_pwm_map[i].pin);

		drv_pwm_write(i, PULSE_MID);
	}

	//TODO: PPM

	/* start timer1 */
	pwm_start_timer(TIM1);
	pwm_start_timer(TIM2);
	pwm_start_timer(TIM3);
	pwm_start_timer(TIM4);
}

void drv_pwm_write( uint8_t port, uint16_t pwm ) {
	if(port < MAX_SERVOS) {
		drv_pwm_set_pulse_width(drv_pwm_map[port].timer_peripheral,
								drv_pwm_map[port].oc_id,
								pwm);
	}
}

uint16_t drv_pwm_read( uint8_t index ) {
	//return captures[channel];
	return 0;
}
