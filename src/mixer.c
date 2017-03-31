#include <stdint.h>

#include "breezystm32.h"
#include "fix16.h"
//#include "fixvector3d.h"
//#include "fixmatrix.h"
//#include "fixquat.h"
#include "fixextra.h"

#include "mixer.h"
#include "params.h"
#include "safety.h"	//TODO: See where this functionallity should be used
#include "controller.h"
#include "rc.h"		//TODO: Put this file back in!

int32_t _GPIO_outputs[8];
static int32_t prescaled_outputs[8];
int32_t _outputs[8];

control_output_t _control_output;
output_type_t _GPIO_output_type[8];

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

//TODO: This logic needs to be double checked
void mixer_init()
{
	//TODO: We need a better way to choosing the mixer
	mixer_to_use = *array_of_mixers[get_param_int(PARAM_MIXER)];

	for (int8_t i=0; i<8; i++) {
		_outputs[i] = 0;
		prescaled_outputs[i] = 0;
		_GPIO_outputs[i] = 0;
		_GPIO_output_type[i] = NONE;
	}

	//TODO: This is handled by the controller
	_command.F = 0;
	_command.x = 0;
	_command.y = 0;
	_command.z = 0;
}

void PWM_init()
{
	bool useCPPM = false;

	if(get_param_int(PARAM_RC_TYPE) == 1) {
		useCPPM = true;
	}

	int16_t motor_refresh_rate = get_param_int(PARAM_MOTOR_PWM_SEND_RATE);
	int16_t idle_pwm = get_param_int(PARAM_MOTOR_IDLE_PWM);
	pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);
}

//TODO: This logic needs to be double checked
//Write a pwm value to the motor channel, value should be between 0 and 1000
void write_motor(uint8_t index, int32_t value) {
	value += 1000;

	if (_armed_state == ARMED) {
		if (value > 2000) {
			value = 2000;
		} else if (value < get_param_int(PARAM_MOTOR_IDLE_PWM) && get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED)) {
			value = get_param_int(PARAM_MOTOR_IDLE_PWM);
		} else if (value < 1000) {
			value = 1000;
		}
	} else {
		value = 1000;
	}

	_outputs[index] = value;
	pwmWriteMotor(index, _outputs[index]);
}

//TODO: This logic needs to be double checked
//Write a pwm value to the motor channel, value should be between -500 and 500
void write_servo(uint8_t index, int32_t value) {
	if (value > 500) {
		value = 500;
	} else if (value < -500) {
		value = -500;
	}
	_outputs[index] = value+1500;
	pwmWriteMotor(index, _outputs[index]);
}


//TODO: This logic needs to be double checked
//TODO: Need to do fix16 operations in this section
//TODO: Need correct references to _control_output
//TODO: HERE!
void mixer_output() {
	int32_t max_output = 0;
	for (int8_t i=0; i<8; i++) {
		if (mixer_to_use.output_type[i] != NONE) {
			// Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
			prescaled_outputs[i] = (int32_t)((_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
											  _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])*1000.0f);

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
				prescaled_outputs[i] = (prescaled_outputs[i])*scale_factor/1000; // divide by scale factor
			}
		}
	}
}
