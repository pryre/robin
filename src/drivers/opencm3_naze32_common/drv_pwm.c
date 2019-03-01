#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "drivers/drv_pwm.h"
#include "io_type.h"

/*
#include "breezystm32.h"
#include "stm32f10x_conf.h"

#include "gpio.h"
#include "timer.h"
*/

/*
Configuration maps:

1) multirotor PPM input
PWM1 used for PPM
PWM5..8 used for motors
PWM9..10 used for motors
PWM11..14 used for motors

2) multirotor PWM input
PWM1..8 used for input
PWM9..10 used for motors
PWM11..14 used for motors
*/

/*
typedef struct {
	volatile uint16_t* ccr;
	volatile uint16_t* cr1;
	volatile uint16_t* cnt;
	uint16_t period;

	// for input only
	uint8_t channel;
	uint8_t state;
	uint16_t rise;
	uint16_t fall;
	uint16_t capture;
} pwmPortData_t;

// typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function
// pointer used to write motors

static pwmPortData_t pwmPorts[PWM_MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
// static pwmWriteFuncPtr pwmWritePtr = NULL;
static uint8_t pwmFilter = 0;

#define PWM_TIMER_MHZ 1
#define PWM_TIMER_8_MHZ 8

static void pwmOCConfig( TIM_TypeDef* tim, uint8_t channel, uint16_t value ) {
	uint16_t tim_oc_preload;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCStructInit( &TIM_OCInitStructure );
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = value;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	tim_oc_preload = TIM_OCPreload_Enable;

	switch ( channel ) {
	case TIM_Channel_1:
		TIM_OC1Init( tim, &TIM_OCInitStructure );
		TIM_OC1PreloadConfig( tim, tim_oc_preload );
		break;
	case TIM_Channel_2:
		TIM_OC2Init( tim, &TIM_OCInitStructure );
		TIM_OC2PreloadConfig( tim, tim_oc_preload );
		break;
	case TIM_Channel_3:
		TIM_OC3Init( tim, &TIM_OCInitStructure );
		TIM_OC3PreloadConfig( tim, tim_oc_preload );
		break;
	case TIM_Channel_4:
		TIM_OC4Init( tim, &TIM_OCInitStructure );
		TIM_OC4PreloadConfig( tim, tim_oc_preload );
		break;
	}
}

static void pwmICConfig( TIM_TypeDef* tim, uint8_t channel, uint16_t polarity ) {
	TIM_ICInitTypeDef TIM_ICInitStructure;

	TIM_ICStructInit( &TIM_ICInitStructure );
	TIM_ICInitStructure.TIM_Channel = channel;
	TIM_ICInitStructure.TIM_ICPolarity = polarity;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = pwmFilter;

	TIM_ICInit( tim, &TIM_ICInitStructure );
}

static void pwmGPIOConfig( GPIO_TypeDef* gpio, uint32_t pin, GPIO_Mode mode ) {
	gpio_config_t cfg;

	cfg.pin = pin;
	cfg.mode = mode;
	cfg.speed = Speed_2MHz;
	gpioInit( gpio, &cfg );
}

static pwmPortData_t* pwmOutConfig( uint8_t port, uint8_t mhz, uint16_t period,
									uint16_t value ) {
	pwmPortData_t* p = &pwmPorts[port];
	configTimeBase( timerHardware[port].tim, period, mhz );
	pwmGPIOConfig( timerHardware[port].gpio, timerHardware[port].pin, Mode_AF_PP );
	pwmOCConfig( timerHardware[port].tim, timerHardware[port].channel, value );
	// Needed only on TIM1
	if ( timerHardware[port].outputEnable )
		TIM_CtrlPWMOutputs( timerHardware[port].tim, ENABLE );
	TIM_Cmd( timerHardware[port].tim, ENABLE );

	p->cr1 = &timerHardware[port].tim->CR1;
	p->cnt = &timerHardware[port].tim->CNT;

	switch ( timerHardware[port].channel ) {
	case TIM_Channel_1:
		p->ccr = &timerHardware[port].tim->CCR1;
		break;
	case TIM_Channel_2:
		p->ccr = &timerHardware[port].tim->CCR2;
		break;
	case TIM_Channel_3:
		p->ccr = &timerHardware[port].tim->CCR3;
		break;
	case TIM_Channel_4:
		p->ccr = &timerHardware[port].tim->CCR4;
		break;
	}
	p->period = period;
	return p;
}

static pwmPortData_t* pwmInConfig( uint8_t port, timerCCCallbackPtr callback,
								   uint8_t channel ) {
	pwmPortData_t* p = &pwmPorts[port];
	const timerHardware_t* timerHardwarePtr = &( timerHardware[port] );

	p->channel = channel;

	pwmGPIOConfig( timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPD );
	pwmICConfig( timerHardwarePtr->tim, timerHardwarePtr->channel,
				 TIM_ICPolarity_Rising );

	timerConfigure( timerHardwarePtr, 0xFFFF, PWM_TIMER_MHZ );
	configureTimerCaptureCompareInterrupt( timerHardwarePtr, port, callback );

	return p;
}

static void ppmCallback( uint8_t port, uint16_t capture ) {
	(void)port;
	uint16_t diff;
	static uint16_t now;
	static uint16_t last = 0;
	static uint8_t chan = 0;

	last = now;
	now = capture;
	diff = now - last;

	if ( diff > 2700 ) { // Per
		// http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
		// "So, if you use 2.5ms or higher as being the reset for the PPM
		// stream start, you will be fine. I use 2.7ms just to be safe."
		chan = 0;
	} else {
		if ( diff > PULSE_MIN && diff < PULSE_MAX && chan < MAX_INPUTS ) { // 750 to 2250 ms is our 'valid' channel range
			captures[chan] = diff;
		}
		chan++;
	}
}

static void pwmCallback( uint8_t port, uint16_t capture ) {
	if ( pwmPorts[port].state == 0 ) {
		pwmPorts[port].rise = capture;
		pwmPorts[port].state = 1;
		pwmICConfig( timerHardware[port].tim, timerHardware[port].channel,
					 TIM_ICPolarity_Falling );
	} else {
		pwmPorts[port].fall = capture;
		// compute capture
		pwmPorts[port].capture = pwmPorts[port].fall - pwmPorts[port].rise;
		if ( pwmPorts[port].capture > PULSE_MIN && pwmPorts[port].capture < PULSE_MAX ) { // valid pulse width
			captures[pwmPorts[port].channel] = pwmPorts[port].capture;
		}
		// switch state
		pwmPorts[port].state = 0;
		pwmICConfig( timerHardware[port].tim, timerHardware[port].channel,
					 TIM_ICPolarity_Rising );
	}
}
*/
// ===========================================================================
/*
static pwmPortData_t* motors[PWM_MAX_PORTS];
static uint8_t numMotors = 0;
static uint8_t numInputs = 0;
*/

/*
XXX: Unused
static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value<1000) ? 0 : (value - 1000) *
motors[index]->period / 1000;
}
*/

/*
static void pwmWriteStandard( uint8_t index, uint16_t value ) {
	*motors[index]->ccr = value;
}
*/

void pwmInit( io_def_t* io_map, bool usePwmFilter, uint32_t motorPwmRate,
			  uint32_t servoPwmRate, uint16_t idlePulseUsec ) {
/*
	// pwm filtering on input
	pwmFilter = usePwmFilter ? 1 : 0;

	int i;
	for ( i = 0; i < PWM_MAX_PORTS; i++ ) {
		// uint8_t port = setup[i] & 0x0F;
		// uint8_t mask = setup[i] & 0xF0;
		uint8_t port = io_map->port[i];
		uint8_t mask = io_map->type[i];

		if ( mask == IO_TYPE_IP ) {
			pwmInConfig( port, ppmCallback, 0 );
			numInputs = 8;
		} else if ( mask == IO_TYPE_IW ) {
			pwmInConfig( port, pwmCallback, numInputs );
			numInputs++;
		} else if ( mask == IO_TYPE_OD ) {
			// TODO: Digital is to behave just as a PWM, but should be fixed later
			uint32_t mhz = PWM_TIMER_MHZ;
			uint32_t hz = mhz * 1000000;
			uint16_t period = hz / servoPwmRate;

			motors[numMotors++] = pwmOutConfig( port, mhz, period, idlePulseUsec );
		} else if ( mask == IO_TYPE_OM ) {
			// uint32_t mhz = (motorPwmRate > 500 || fastPWM) ? PWM_TIMER_8_MHZ :
			// PWM_TIMER_MHZ;
			uint32_t mhz = PWM_TIMER_MHZ;
			uint32_t hz = mhz * 1000000;
			uint16_t period = hz / motorPwmRate;

			motors[numMotors++] = pwmOutConfig( port, mhz, period, idlePulseUsec );
		} else if ( mask == IO_TYPE_OS ) {
			// uint32_t mhz = (motorPwmRate > 500 || fastPWM) ? PWM_TIMER_8_MHZ :
			// PWM_TIMER_MHZ;
			uint32_t mhz = PWM_TIMER_MHZ;
			uint32_t hz = mhz * 1000000;
			uint16_t period = hz / servoPwmRate;

			motors[numMotors++] = pwmOutConfig( port, mhz, period, idlePulseUsec );
		}
	}
*/
	/*
// determine motor writer function
pwmWritePtr = pwmWriteStandard;
if (motorPwmRate > 500) {
pwmWritePtr = pwmWriteBrushed;
}
*/
}

void pwmWriteMotor( uint8_t index, uint16_t value ) {
	/*
if (index < numMotors)
pwmWritePtr(index, value);
*/
	//if ( index < numMotors )
		//pwmWriteStandard( index, value );
}

uint16_t pwmRead( uint8_t channel ) {
	//return captures[channel];
	return 0;
}
