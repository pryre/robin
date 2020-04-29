/*
   pwm.c : PWM support for STM32F103

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/pwm.c

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "breezystm32.h"
#include "stm32f10x_conf.h"

#include "gpio.h"
#include "timer.h"

#include "io_type.h"
#include "params.h"
#include "mixer.h"
#include "drivers/drv_pwm.h"
#include "drivers/drv_ppm.h"

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

static pwmPortData_t pwmPorts[DRV_PWM_MAX_OUTPUTS];
static uint16_t captures[DRV_PPM_MAX_INPUTS];
static uint16_t pwmwrites[DRV_PPM_MAX_INPUTS];
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

	if( get_param_fix16(PARAM_RC_PPM_PPM_PULSE_TYPE) ) {
		pwmGPIOConfig( timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPD );
		pwmICConfig( timerHardwarePtr->tim, timerHardwarePtr->channel,
					 TIM_ICPolarity_Rising );
	} else {
		pwmGPIOConfig( timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU );
		pwmICConfig( timerHardwarePtr->tim, timerHardwarePtr->channel,
					 TIM_ICPolarity_Falling );
	}

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

	if ( diff > get_param_uint( PARAM_RC_PPM_PPM_SYNC_MIN ) ) { // Per
		// http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
		// "So, if you use 2.5ms or higher as being the reset for the PPM
		// stream start, you will be fine. I use 2.7ms just to be safe."
		chan = 0;
	} else {
		if ( diff > get_param_uint( PARAM_RC_PPM_PPM_DATA_MIN ) &&
			 diff < get_param_uint( PARAM_RC_PPM_PPM_DATA_MAX ) &&
			 chan < DRV_PPM_MAX_INPUTS ) { // 'valid' channel range and is a valid channel
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
		if ( pwmPorts[port].capture > DRV_PPM_MIN && pwmPorts[port].capture < DRV_PPM_MAX ) { // valid pulse width
			captures[pwmPorts[port].channel] = pwmPorts[port].capture;
		}
		// switch state
		pwmPorts[port].state = 0;
		pwmICConfig( timerHardware[port].tim, timerHardware[port].channel,
					 TIM_ICPolarity_Rising );
	}
}

// ===========================================================================

/*
static const uint8_t multiPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // Swap to servo if needed
    PWM10 | TYPE_M,     // Swap to servo if needed
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM5 | TYPE_M,      // Swap to servo if needed
    PWM6 | TYPE_M,      // Swap to servo if needed
    PWM7 | TYPE_M,      // Swap to servo if needed
    PWM8 | TYPE_M,      // Swap to servo if needed
    0xFF
};

static const uint8_t multiPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | TYPE_M,     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | TYPE_M,     // motor #1 or #3
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,     // motor #4 or #6
    0xFF
};
*/

static pwmPortData_t* motors[PWM_MAX_PORTS];
static uint8_t numMotors = 0;
static uint8_t numInputs = 0;

/*
static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value<1000) ? 0 : (value - 1000) *
motors[index]->period / 1000;
}
*/

static void pwmWriteStandard( uint8_t index, uint16_t value ) {
	*motors[index]->ccr = value;
}

static void pwmInit( io_def_t* io_map, bool usePwmFilter, uint32_t motorPwmRate,
			  uint32_t servoPwmRate, uint16_t idlePulseUsec ) {
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

	/*
// determine motor writer function
pwmWritePtr = pwmWriteStandard;
if (motorPwmRate > 500) {
pwmWritePtr = pwmWriteBrushed;
}
*/
}

//=======================================================================
// Driver IO as seen by Robin
//=======================================================================

// XXX: Pinout Mapping for Naze32!
static const uint8_t io_map_naze32_ppm = 0;
static const uint8_t io_map_naze32_pwm[MIXER_NUM_MOTORS] = {8, 9, 10, 11,
															12, 13, 4, 5};

io_type_t _actuator_type_map[MIXER_NUM_MOTORS];

bool drv_pwm_init( void ) {
	// XXX: Loop backwards through the IO map to set the number of motor/servo
	// ports
	uint8_t io_c = 0;
	io_def_t io_map;
	io_map.port[io_c] = io_map_naze32_ppm; // IO set for PPM input
	io_map.type[io_c] = IO_TYPE_IP;		   // IO set for PPM input
	io_c++;

	for ( int i = 0; i < MIXER_NUM_MOTORS; i++ ) {
		io_map.port[io_c] = io_map_naze32_pwm[i]; // IO map set for PWM input
		// io_map.type[io_c] = (_mixer_to_use->output_type[i] == IO_TYPE_OM ) ?
		// IO_TYPE_OM : IO_TYPE_OS;	//IO map set for PWM input
		io_map.type[io_c] = _actuator_type_map[i];
		io_c++;
	}

	// Fill out the rest of the ports to not be initialized
	while ( io_c < PWM_MAX_PORTS ) {
		io_map.port[io_c] = 0;
		io_map.type[io_c] = IO_TYPE_N;
		io_c++;
	}

	pwmInit( &io_map, false, get_param_uint( PARAM_MOTOR_PWM_SEND_RATE ),
			 get_param_uint( PARAM_SERVO_PWM_SEND_RATE ),
			 get_param_uint( PARAM_MOTOR_PWM_MIN ) );

	return true;
}


void drv_pwm_write( uint8_t index, uint16_t value ) {
	if ( index < numMotors ) {
		pwmWriteStandard( index, value );
		pwmwrites[index] = value;
	}
}

uint16_t drv_pwm_get_current( uint8_t index ) {
	return ( (index < DRV_PPM_MAX_INPUTS) ? pwmwrites[index] : 0 );
}

bool drv_ppm_ready(void) {
	return true;
}

bool drv_ppm_read_frame( uint16_t * readings ) {
	for(int i=0; i< DRV_PPM_MAX_INPUTS; i++) {
		readings[i] = captures[i];
	}

	return true;
}

