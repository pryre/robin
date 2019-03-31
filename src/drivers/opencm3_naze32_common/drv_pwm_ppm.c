#include "drivers/drv_pwm.h"
#include "drivers/drv_ppm.h"
#include "drivers/drv_system.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <stdint.h>
#include <stdbool.h>


#define DRV_PWM_BASE_FREQ 1000000
#define DRV_PWM_BASE_PERIOD 20000

#define PPM_PULSE_TYPE_POSITIVE 1
#define PPM_PULSE_TYPE_NEGATIVE 2
#define PPM_PULSE_TYPE PPM_PULSE_TYPE_POSITIVE


typedef struct {
	uint32_t timer_peripheral;
	uint32_t timer_channel;
	uint32_t port;
	uint32_t pin;
} drv_pwm_map_t;

//Record of the current PWM output
static uint16_t pwm_frame_[DRV_PWM_MAX_OUTPUTS];

//State machine for decoding ppm frames
static volatile uint16_t ppm_pulses_[DRV_PPM_MAX_INPUTS];	//Measurement storage
static volatile uint16_t ppm_frame_[DRV_PPM_MAX_INPUTS];	//Read OK storage
static volatile bool ppm_frame_available_;
static volatile uint32_t timer_rollover_cnt_;

static volatile uint8_t  ppm_cur_pulse_;
static volatile uint32_t ppm_last_pulse_time_;
static volatile bool ppm_data_valid_;

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
#define PPM_IRQ			NVIC_TIM2_IRQ
#define PPM_TIMER_INPUT	TIM_IC_IN_TI1
static const drv_pwm_map_t drv_ppm_map_ = {TIM2,	TIM_IC1,	GPIOA,	GPIO0};

static const drv_pwm_map_t drv_pwm_map_[DRV_PWM_MAX_OUTPUTS] = {
	{TIM1,	TIM_OC1,	GPIOA,	GPIO8},
	{TIM1,	TIM_OC4,	GPIOA,	GPIO11},
	{TIM4,	TIM_OC1,	GPIOB,	GPIO6},
	{TIM4,	TIM_OC2,	GPIOB,	GPIO7},
	{TIM4,	TIM_OC3,	GPIOB,	GPIO8},
	{TIM4,	TIM_OC4,	GPIOB,	GPIO9},
	{TIM3,	TIM_OC3,	GPIOB,	GPIO0},
	{TIM3,	TIM_OC4,	GPIOB,	GPIO1},
};

//=======================================================================

static uint32_t rcc_get_ppre1(void) {
	return RCC_CFGR & RCC_CFGR_PPRE1;
}

static uint32_t rcc_get_ppre2(void) {
	return RCC_CFGR & RCC_CFGR_PPRE2;
}

static uint32_t timer_get_frequency(uint32_t timer_peripheral) {
	uint32_t freq = 0;

	switch (timer_peripheral) {
		// Timers on high speed APB2
		case TIM1:
		case TIM8:
			if (!rcc_get_ppre2()) {
				// without APB2 prescaler, runs at APB2 freq
				//say("No ppre2");
				freq = rcc_apb2_frequency;
			} else {
				// with any ABP2 prescaler, runs at 2 * APB2 freq
				//say("2x ppre2");
				freq = rcc_apb2_frequency * 2;
			}
			break;
		// timers on low speed APB1
		case TIM2:
		case TIM3:
		case TIM4:
		case TIM5:
		case TIM6:
		case TIM7:
			if (!rcc_get_ppre1()) {
				// without APB1 prescaler, runs at APB1 freq
				//say("No ppre1");
				freq = rcc_apb1_frequency;
			} else {
				// with any ABP1 prescaler, runs at 2 * APB1 freq
				//say("2x ppre1");
				freq = rcc_apb1_frequency * 2;
			}
			break;
		// other timers currently not supported
		default:
			break;
	}

	return freq;
}

static uint32_t timer_get_rcc_peripheral(uint32_t timer_peripheral) {
	uint32_t rcc_tim = 0;

	switch (timer_peripheral) {
		case TIM1: {
			rcc_tim = RCC_TIM1;
			break;
		}
		case TIM2: {
			rcc_tim = RCC_TIM2;
			break;
		}
		case TIM3: {
			rcc_tim = RCC_TIM3;
			break;
		}
		case TIM4: {
			rcc_tim = RCC_TIM4;
			break;
		}
	}

	return rcc_tim;
}

static uint32_t timer_get_reset_peripheral(uint32_t timer_peripheral) {
	uint32_t reset_tim = 0;

	switch (timer_peripheral) {
		case TIM1: {
			reset_tim = RST_TIM1;
			break;
		}
		case TIM2: {
			reset_tim = RST_TIM2;
			break;
		}
		case TIM3: {
			reset_tim = RST_TIM3;
			break;
		}
		case TIM4: {
			reset_tim = RST_TIM4;
			break;
		}
	}

	return reset_tim;
}

static void drv_pwm_init_timer(uint32_t tim) {
	//Enable TIM clock
	rcc_periph_clock_enable(timer_get_rcc_peripheral(tim));

	//Reset TIM peripheral to defaults
	rcc_periph_reset_pulse(timer_get_reset_peripheral(tim));

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	//Set prescaler, need to make sure it is consistent with the APB1/APB2 clocks
	timer_set_prescaler( tim, ( timer_get_frequency(tim) / DRV_PWM_BASE_FREQ ) - 1 );

	//Disable preload
	timer_disable_preload(tim);

	//Compare value continuously and count 20ms range for PWM
	timer_continuous_mode(tim);
	timer_set_period(tim, DRV_PWM_BASE_PERIOD);

	/* Set the initual output compare value for OC1. */
	//timer_set_oc_value(tim, TIM_OC1, frequency_sequence[frequency_sel++]);

	/* Enable TIM2 interrupt. */
	//XXX: nvic_enable_irq(NVIC_TIM2_IRQ);
	/* Enable Channel 1 compare interrupt to recalculate compare values */
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);

	// Counter enable
	//timer_enable_preload(tim);	//XXX: Probably won't be used
	//timer_enable_counter(tim);
}

static void drv_pwm_init_output_channel(uint32_t timer_peripheral,
										uint32_t oc_id,
										uint32_t gpio_port,
										uint16_t gpio_pin) {
	/* Set timer channel to output */
	gpio_set_mode(gpio_port, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
				  gpio_pin);
	/*
	timer_disable_oc_output(timer_peripheral, oc_id);
	timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
	timer_set_oc_value(timer_peripheral, oc_id, 0);
	timer_enable_oc_output(timer_peripheral, oc_id);
	*/
	timer_disable_oc_output(timer_peripheral, oc_id);
	timer_enable_oc_preload(timer_peripheral, oc_id);
	timer_set_oc_slow_mode(timer_peripheral, oc_id);
	timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(timer_peripheral, oc_id);
	timer_enable_oc_output(timer_peripheral, oc_id);
	// Used for TIM1 and TIM8, the function does nothing if other timer is specified.
	timer_enable_break_main_output(timer_peripheral);
}

static void drv_pwm_set_pulse_width(uint32_t timer_peripheral,
									uint32_t oc_id,
									uint32_t pulse_width) {

     timer_set_oc_value(timer_peripheral, oc_id, pulse_width);
}

static void drv_ppm_set_ready( bool ready ) {
	ppm_frame_available_ = ready;
}

static void drv_ppm_decode_frame_width(uint32_t ppm_width) {
	//say("ppm_decode_frame_width()");

	if (ppm_cur_pulse_ == DRV_PPM_MAX_INPUTS) {
		if( ( ppm_width > DRV_PPM_SYNC_MIN ) &&
			( ppm_width < DRV_PPM_SYNC_MAX ) ) {

			if (ppm_data_valid_) {
				//Store the data in a clean buffer and make it available
				for(int i=0; i<DRV_PPM_MAX_INPUTS; i++)
					ppm_frame_[i] = ppm_pulses_[i];

				drv_ppm_set_ready( true );

				//Prepare for next data read
				ppm_data_valid_ = false;
			}

			ppm_cur_pulse_ = 0;
		} else {
			ppm_data_valid_ = false;
		}

		//say("sync");
	} else {
		if( ( ppm_width > DRV_PPM_MIN ) &&
			( ppm_width < DRV_PPM_MAX ) ) {

			ppm_pulses_[ppm_cur_pulse_] = ppm_width;
			ppm_cur_pulse_++;

			if (ppm_cur_pulse_ == DRV_PPM_MAX_INPUTS) {
				ppm_data_valid_ = true;
			}
		} else {
			ppm_cur_pulse_ = DRV_PPM_MAX_INPUTS;
			ppm_data_valid_ = false;
		}
	}
}

static void drv_ppm_decode_frame(uint32_t ppm_time) {
	uint32_t length = ppm_time - ppm_last_pulse_time_;
	ppm_last_pulse_time_ = ppm_time;

	drv_ppm_decode_frame_width(length);
}

void tim2_isr(void) {
	if( TIM2_SR & TIM_SR_CC1IF ) {
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
		uint32_t now = timer_get_counter(TIM2) + timer_rollover_cnt_;
		drv_ppm_decode_frame(now);
	} else if( TIM2_SR & TIM_SR_UIF ) {
		timer_clear_flag(TIM2, TIM_SR_UIF);
		timer_rollover_cnt_ += DRV_PWM_BASE_PERIOD;//(1 << 16);
	}
}

//=======================================================================
//	PWM Driver
//=======================================================================

bool drv_pwm_init( void ) {
	// Init all our timers to run at 1us intervals, overflow at PWM_PERIOD
	//say("Setup: TIM1");
	drv_pwm_init_timer(TIM1);
	//say("Setup: TIM3");
	drv_pwm_init_timer(TIM3);
	//say("Setup: TIM4");
	drv_pwm_init_timer(TIM4);

	// init output of channel2 of timer2
	//drv_pwm_init_output_channel(TIM2, SERVO_CH1, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH2);

	// init output of channel3 of timer2
	//pwm_init_output_channel(TIM2, SERVO_CH2, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH3);

	//pwm_set_pulse_width(TIM2, SERVO_CH1, SERVO_NULL);
	//pwm_set_pulse_width(TIM2, SERVO_CH2, SERVO_NULL);

	//say("Setup: PWM");
	for(uint32_t i=0; i< DRV_PWM_MAX_OUTPUTS; i++) {
		drv_pwm_init_output_channel(drv_pwm_map_[i].timer_peripheral,
									drv_pwm_map_[i].timer_channel,
									drv_pwm_map_[i].port,
									drv_pwm_map_[i].pin);

		//Initialize to "digital low" to start with
		//Also inits our pwm_frame_ storage variable
		drv_pwm_write(i, 0);
	}

	//Start timers
	timer_enable_counter(TIM1);
	timer_enable_counter(TIM3);
	timer_enable_counter(TIM4);

	return true;
}

void drv_pwm_write( uint8_t index, uint16_t value ) {
	if(index < DRV_PWM_MAX_OUTPUTS) {
		drv_pwm_set_pulse_width(drv_pwm_map_[index].timer_peripheral,
								drv_pwm_map_[index].timer_channel,
								value);

		pwm_frame_[index] = value;
	}
}

uint16_t drv_pwm_get_current( uint8_t index ) {
	return ( (index < DRV_PWM_MAX_OUTPUTS) ? pwm_frame_[index] : 0 );
}

//=======================================================================
//	PPM Driver
//=======================================================================

bool drv_ppm_init(void) {
	timer_rollover_cnt_ = 0;
	ppm_cur_pulse_ = 0;
	ppm_last_pulse_time_ = 0;
	ppm_data_valid_ = false;
	drv_ppm_set_ready(false);

	//Timer clock enable
	//XXX: Handled in drv_pwm_init()
	//rcc_periph_clock_enable(timer_get_rcc_peripheral(drv_ppm_map_.timer_peripheral));
	//rcc_periph_reset_pulse(timer_get_reset_peripheral(drv_ppm_map_.timer_peripheral));

	// Time Base configuration
	//XXX: Handled in drv_pwm_init()
	//timer_reset(PPM_TIMER);
	//timer_set_mode(PPM_TIMER, TIM_CR1_CKD_CK_INT,
	//	 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	//timer_set_period(PPM_TIMER, 0xFFFF);
	//uint32_t timer_clk = timer_get_frequency(PPM_TIMER);
	//timer_set_prescaler(PPM_TIMER, (timer_clk / (RC_PPM_TICKS_PER_USEC * ONE_MHZ_CLK)) - 1);

	//say("Setup: TIM2");
	drv_pwm_init_timer(drv_ppm_map_.timer_peripheral);

	// TIM channel configuration: Input Capture mode
	// The first rising/falling edge is used as active edge

	// GPIO configuration as input capture for timer
	gpio_set_mode(drv_ppm_map_.port, GPIO_MODE_INPUT,
			  GPIO_CNF_INPUT_PULL_UPDOWN,//GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
			  drv_ppm_map_.pin);
	//gpio_setup_pin_af(PPM_GPIO_PORT, PPM_GPIO_PIN, PPM_GPIO_AF, FALSE);

	//TODO: PPM pulse type could be a variable?
	#if defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_POSITIVE
	timer_ic_set_polarity(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel, TIM_IC_RISING);
	gpio_clear(drv_ppm_map_.port, drv_ppm_map_.pin); //Enable pull-down
	#elif defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_NEGATIVE
	timer_ic_set_polarity(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel, TIM_IC_FALLING);
	gpio_set(drv_ppm_map_.port, drv_ppm_map_.pin); //Enable pull-up
	#else
	#error "Unknown PPM_PULSE_TYPE"
	#endif
	timer_ic_set_input(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel, PPM_TIMER_INPUT);
	timer_ic_set_prescaler(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel, TIM_IC_PSC_OFF);
	timer_ic_set_filter(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel, TIM_IC_OFF);

	// Enable timer interrupt
	//nvic_set_priority(PPM_IRQ, PPM_IRQ_PRIO);
	nvic_enable_irq(PPM_IRQ);
	// Enable the Capture/Compare and Update interrupt requests
	timer_enable_irq(drv_ppm_map_.timer_peripheral, (TIM_DIER_CC1IE | TIM_DIER_UIE));

	// Enable capture channel
	timer_ic_enable(drv_ppm_map_.timer_peripheral, drv_ppm_map_.timer_channel);

	// TIM enable counter
	//timer_enable_counter(drv_ppm_map_.timer_peripheral);
	timer_enable_counter(drv_ppm_map_.timer_peripheral);

	return true;
}

bool drv_ppm_ready( void ) {
	return ppm_frame_available_;
}

bool drv_ppm_read_frame( uint16_t *frame ) {
	bool success = false;

	if( drv_ppm_ready() ) {
		CM_ATOMIC_BLOCK() {
		for(int i=0; i<DRV_PPM_MAX_INPUTS; i++)
			frame[i] = ppm_frame_[i];

		drv_ppm_set_ready(false);
		}
		success = true;
	}

	return success;
}
