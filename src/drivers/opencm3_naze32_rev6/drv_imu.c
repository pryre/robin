#include "drivers/drv_sensors.h"
#include "drivers/drv_system.h"
#include "drivers/opencm3_naze32_common/drv_mpu.h"
#include "sensors.h"

#include "mavlink_system.h"

#include "fix16.h"
#include "fixextra.h"

//#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

/*=======================================================
 * Custom ISR Registration
 * This method registers a custom interrpt to be
 * run upon the interrupt pin on the MPU6050 going high
 */

//XXX: GPIOB for boardVersion <= 4
#define GPIO_INT GPIOC
#define PIN_INT GPIO13
#define EXTI_INT EXTI13

static void ( *imuInterruptCallbackPtr )( void );

void exti15_10_isr( void ) {
/*
	if ( EXTI_GetITStatus( EXTI_Line13 ) != RESET ) {

	}
	EXTI_ClearITPendingBit( EXTI_Line13 );
*/
	//Record down the current reset status and request a reset on out specific pin
	exti_reset_request(EXTI_INT);

	//If it was our pin that needed to be reset, handle it
	if ( imuInterruptCallbackPtr != NULL )
		imuInterruptCallbackPtr();
}

void drv_sensors_imu_configure_int( void ( *functionPtr )( void ) ) {
	// Enable the  interrupt on pins 10-15
	nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x0F);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);

	// Configure our GPIO
	gpio_set_mode(GPIO_INT, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_FLOAT, PIN_INT);

	// Configure the EXTI subsystem
	//We only care about the rising trigger
	exti_select_source(EXTI_INT, GPIO_INT);
	exti_set_trigger(EXTI_INT, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI_INT);

	imuInterruptCallbackPtr = functionPtr;
}

bool drv_sensors_imu_init( uint32_t i2c, fix16_t* scale_accel, fix16_t* scale_gyro ) {
	imuInterruptCallbackPtr = NULL;

	// Get the 1g gravity scale (raw->g's)
	uint16_t acc1G = 0;
	bool success = mpu6500_init( i2c, INV_FSR_8G, INV_FSR_2000DPS, &acc1G );

	if (success) {
		*scale_accel = fix16_div( _fc_gravity,
								  fix16_from_int( acc1G ) ); // Get the m/s scale (raw->g's->m/s/s)
		*scale_gyro = fix16_from_float( MPU_GYRO_SCALE );	// Get radians scale (raw->rad/s)
	}

	return success;
}
