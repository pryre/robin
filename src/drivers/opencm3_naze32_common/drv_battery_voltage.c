#include <stdbool.h>

#include "drivers/drv_sensors.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "drivers/drv_status_io.h"
#include "drivers/drv_system.h"
#include "mavlink_system.h"


static bool adc_is_calibrated_ = false;
static uint32_t adc_start_time_ = 0;
static uint32_t adc_cal_time_ = 0;


bool drv_sensors_battery_monitor_init( void ) {
	adc_is_calibrated_ = false;
	adc_start_time_  = system_micros();
	adc_cal_time_ = 0;

	rcc_periph_clock_enable(RCC_ADC1);

	// Make sure the ADC doesn't run during config.
	adc_power_off(ADC1);

	// We configure everything for one single conversion.
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	// We want to read the temperature sensor, so we have to enable it.
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/*
	//Wait for ADC starting up.
	for (int i = 0; i < 800000; i++)    //wait a while
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate_async(ADC1); // Will check this later
	*/

	// Select the channel we want to convert. 16=temperature_sensor.
	// Only use one channel for now (ADC_Channel_4 = Voltage)
	uint8_t channel_array[ADC_SQR_MAX_CHANNELS_REGULAR];
	channel_array[0] = ADC_CHANNEL4;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	return true;
}

uint16_t drv_sensors_battery_monitor_read( void ) {
	uint16_t voltage = 0;

	// Start cal if device is warmed up
	if( ( !adc_is_calibrated_ ) &&
		( system_micros() - adc_start_time_ > 1000000 ) ) {
		//If we haven't started calibrating
		//then start calibration
		if( !adc_cal_time_ ) {
			adc_cal_time_ = system_micros();
			adc_reset_calibration(ADC1);
			adc_calibrate_async(ADC1); // Will check this later
		} else {
			// calibration is done
			if( !adc_is_calibrating(ADC1) ) {
				adc_is_calibrated_ = true;

				mavlink_queue_broadcast_info( "[SENSOR] ADC cal done, enabling battery voltage" );
			}
		}
	}

	if( adc_is_calibrated_ ) {
		// Start ADC conversion without trigger
		adc_start_conversion_direct(ADC1);

		// Wait for end of conversion.
		while (!(adc_eoc(ADC1)));

		// Read voltage
		voltage = adc_read_regular(ADC1);
	}

	return voltage;
}
