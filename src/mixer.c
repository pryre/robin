#include <stdint.h>

#include "breezystm32.h"
#include "pwm.h"
#include "mavlink/common/mavlink.h"
#include "fix16.h"
//#include "fixvector3d.h"
//#include "fixmatrix.h"
//#include "fixquat.h"
#include "fixextra.h"

#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "controller.h"

control_output_t _control_output;
system_t _system_status;

int32_t _GPIO_outputs[8];
output_type_t _GPIO_output_type[8];
int32_t _outputs[8];

//TODO: Double check
static mixer_t quadcopter_plus_mixing = {
	{M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

	{ CONST_ONE, CONST_ONE, CONST_ONE, CONST_ONE, 0, 0, 0, 0}, // F Mix
	{ 0, -CONST_ONE, CONST_ONE, 0, 0, 0, 0, 0}, // X Mix
	{-CONST_ONE, 0, 0, CONST_ONE, 0, 0, 0, 0}, // Y Mix
	{-CONST_ONE, CONST_ONE, CONST_ONE, -CONST_ONE, 0, 0, 0, 0}  // Z Mix
};

//TODO: Double check
static mixer_t quadcopter_x_mixing = {
	{M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

	{ CONST_ONE, CONST_ONE, CONST_ONE, CONST_ONE,  0, 0, 0, 0}, // F Mix
	{-CONST_ONE,-CONST_ONE, CONST_ONE, CONST_ONE,  0, 0, 0, 0}, // X Mix
	{-CONST_ONE, CONST_ONE,-CONST_ONE, CONST_ONE,  0, 0, 0, 0}, // Y Mix
	{ CONST_ONE,-CONST_ONE,-CONST_ONE, CONST_ONE,  0, 0, 0, 0}  // Z Mix
};

static mixer_t mixer_to_use;

static mixer_t *array_of_mixers[NUM_MIXERS] = {
	&quadcopter_plus_mixing,
	&quadcopter_x_mixing
};

void mixer_init() {
	//TODO: We need a better way to choosing the mixer
	mixer_to_use = *array_of_mixers[get_param_int(PARAM_MIXER)];

	for (int8_t i=0; i<8; i++) {
		_outputs[i] = 0;
		_GPIO_outputs[i] = 0;
		_GPIO_output_type[i] = NONE;
	}
}

//TODO: Errors here with linker because pwm.h isn't included properly?
void pwm_init() {
	bool useCPPM = false;

	/*TODO: Put this back in?
	if(get_param_int(PARAM_RC_TYPE) == 1)
		useCPPM = true;
	*/

	int16_t motor_refresh_rate = get_param_int(PARAM_MOTOR_PWM_SEND_RATE);
	int16_t idle_pwm = get_param_int(PARAM_MOTOR_PWM_IDLE);
	pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);	//TODO: Poor use of similar naming
}

static int32_t int32_constrain(int32_t i, const int32_t min, const int32_t max) {
	/*
	if(i < min) {
		i = min;
	} else if(i > max) {
		i = max;
	}

	return i;
	*/

	return (i < min) ? min : (i > max) ? max : i;
}

//TODO: Maybe this logic should be checked elsewhere?
//Write a pwm value to the motor channel, value should be between 0 and 1000
void write_motor(uint8_t index, int32_t value) {
	if ( (_system_status.mode & MAV_MODE_FLAG_SAFETY_ARMED) && (_system_status.mode == MAV_STATE_ACTIVE) ) {
		//If the "hardware" safety and software safety is armed (double safety check)
		_outputs[index] = int32_constrain(value, 0, 1000) + get_param_int(PARAM_MOTOR_PWM_MIN);

		//If there is an idle set
		if( value < get_param_int(PARAM_MOTOR_PWM_IDLE) )
			_outputs[index] = get_param_int(PARAM_MOTOR_PWM_IDLE);
	} else {
		_outputs[index] = get_param_int(PARAM_MOTOR_PWM_MIN);
	}

	pwmWriteMotor(index, _outputs[index]);
}

//TODO: Is this even needed? (Tricopters)
//Write a pwm value to the motor channel, value should be between -500 and 500
void write_servo(uint8_t index, int32_t value) {
	_outputs[index] = int32_constrain(value, -500, 500) + 1500;
	pwmWriteMotor(index, _outputs[index]);
}


//TODO: Need to do fix16 operations in this section
//TODO: Some of this logic does not look right at all
//TODO: Errors here with linker because pwm.h isn't included properly?
void mixer_output() {
	int32_t max_output = 0;
	int32_t prescaled_outputs[8];
	int32_t scaled_outputs[8];

	for (uint8_t i = 0; i<8; i++) {
		//TODO: This logic needs to be double checked
		if (mixer_to_use.output_type[i] != NONE) {
			// Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
			//TODO: This might actually be very easy to do with fix16 matrix operations...
			//TODO: HERE!
			//prescaled_outputs[i] = (int32_t)((_control_output.F*mixer_to_use.F[i] + _control_output.x*mixer_to_use.x[i] +
			//								  _control_output.y*mixer_to_use.y[i] + _control_output.z*mixer_to_use.z[i])*1000.0f);

			if (prescaled_outputs[i] > 1000 && prescaled_outputs[i] > max_output) {
				max_output = prescaled_outputs[i];
			}
			// negative motor outputs are set to zero when writing to the motor,
			// but they have to be allowed here because the same logic is used for
			// servo commands, which may be negative
		}
	}

	// saturate outputs to maintain controllability even during aggressive maneuvers
	if (max_output > 1000) {
		int32_t scale_factor = 1000*1000/max_output;

		for (int8_t i=0; i<8; i++) {
			if (mixer_to_use.output_type[i] == M) {
				scaled_outputs[i] = (prescaled_outputs[i])*scale_factor/1000; // divide by scale factor
			}
		}
	}

	// Add in GPIO inputs from Onboard Computer
	for (int8_t i=0; i<8; i++) {
		output_type_t output_type = mixer_to_use.output_type[i];

		//TODO: This logic needs to be double checked
		if (output_type == NONE) {
			// Incorporate GPIO on not already reserved outputs
			scaled_outputs[i] = _GPIO_outputs[i];
			output_type = _GPIO_output_type[i];
		}

		// Write output to motors
		if (output_type == S) {
			write_servo(i, scaled_outputs[i]);
		} else if (output_type == M) {
			write_motor(i, scaled_outputs[i]);
		}
	}
}
