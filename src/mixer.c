#include <stdint.h>

#include "breezystm32.h"
#include "pwm.h"
#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"
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
system_status_t _system_status;

int32_t _GPIO_outputs[8];
output_type_t _GPIO_output_type[8];
int32_t _pwm_control[8];
int32_t _pwm_output_requested[8];
int32_t _pwm_output[8];

static mixer_t mixer_none = {
	{NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE}, // output_type

	{ 0, 0, 0, 0, 0, 0, 0, 0}, // F Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}, // X Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}, // Y Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}  // Z Mix
};

//TODO: Double check
static mixer_t mixer_quadrotor_plus = {
	{M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

	{ _fc_1, _fc_1, _fc_1, _fc_1, 0, 0, 0, 0}, // F Mix
	{-_fc_1, _fc_1, 0,    0,    0, 0, 0, 0}, // X Mix
	{ 0,    0,   -_fc_1, _fc_1, 0, 0, 0, 0}, // Y Mix
	{ _fc_1, _fc_1,-_fc_1,-_fc_1, 0, 0, 0, 0}  // Z Mix
};

static mixer_t mixer_quadrotor_x = {
	{M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

	{ _fc_1, _fc_1, _fc_1, _fc_1, 0, 0, 0, 0}, // F Mix
	{-_fc_1, _fc_1, _fc_1,-_fc_1, 0, 0, 0, 0}, // X Mix
	{ _fc_1,-_fc_1, _fc_1,-_fc_1, 0, 0, 0, 0}, // Y Mix
	{ _fc_1, _fc_1,-_fc_1,-_fc_1, 0, 0, 0, 0}  // Z Mix
};

static mixer_t mixer_hexarotor_x = {
	{M, M, M, M, M, M, NONE, NONE}, // output_type

	{ _fc_1, _fc_1,   _fc_1,   _fc_1,    _fc_1,   _fc_1, 0, 0}, // F Mix
	{-_fc_1, _fc_1, _fc_0_5,-_fc_0_5, -_fc_0_5, _fc_0_5, 0, 0}, // X Mix
	{    0,    0,     _fc_1,  -_fc_1,    _fc_1,   _fc_1, 0, 0}, // Y Mix
	{-_fc_1, _fc_1,  -_fc_1,   _fc_1,    _fc_1,   _fc_1, 0, 0}  // Z Mix
};

static mixer_t mixer_plane_standard = {
	{M, S, S, S, NONE, NONE, NONE, NONE}, // output_type

	{ _fc_1, 0, 0, 0, 0, 0, 0, 0}, // F Mix
	{ 0, _fc_1, 0, 0, 0, 0, 0, 0}, // X Mix
	{ 0, 0, _fc_1, 0, 0, 0, 0, 0}, // Y Mix
	{ 0, 0, 0, _fc_1, 0, 0, 0, 0}  // Z Mix

	//TODO: Should double check this, but it would allow stabilize control with motor throughput and a servo out for ailerons, elevator and rudder
};

static mixer_t *mixer_to_use;

void mixer_init() {
	switch( get_param_uint(PARAM_MIXER) ) {
		case MIXER_QUADROTOR_PLUS: {
			mixer_to_use = &mixer_quadrotor_plus;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR);
			mavlink_queue_broadcast_error("[MIX] Using mixer QUAD +");

			break;
		}
		case MIXER_QUADROTOR_X: {
			mixer_to_use = &mixer_quadrotor_x;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR);
			mavlink_queue_broadcast_error("[MIX] Using mixer QUAD X");

			break;
		}
		case MIXER_HEXAROTOR_X: {
			mixer_to_use = &mixer_hexarotor_x;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_HEXAROTOR);
			mavlink_queue_broadcast_error("[MIX] Using mixer HEX X");

			break;
		}
		case MIXER_PLANE_STANDARD: {
			mixer_to_use = &mixer_plane_standard;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_FIXED_WING);
			mavlink_queue_broadcast_error("[MIX] Using mixer PLANE STANDARD");

			break;
		}
		default: {
			mixer_to_use = &mixer_none;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_GENERIC);
			mavlink_queue_broadcast_error("[MIX] Unknown mixer! Disabling!");

			break;
		}
	}

	for (uint8_t i = 0; i < 8; i++) {
		_pwm_control[i] = 0;
		_pwm_output_requested[i] = 0;
		_pwm_output[i] = 0;
		_GPIO_outputs[i] = 0;
		_GPIO_output_type[i] = NONE;
	}
}

void pwm_init() {
	bool useCPPM = true;
	/* XXX: Frees up the ports for telem2 and safety switch
	//In-case needed in future
	if(get_param_int(PARAM_RC_TYPE) == 1)
		useCPPM = true;
	*/

	int16_t motor_refresh_rate = get_param_uint(PARAM_MOTOR_PWM_SEND_RATE);
	int16_t pwm_disarm = get_param_uint(PARAM_MOTOR_PWM_MIN);
	pwmInit(useCPPM, false, false, motor_refresh_rate, pwm_disarm);

	if( get_param_uint( PARAM_DO_ESC_CAL ) ) {
		for (uint8_t i = 0; i < 8; i++)
			if (mixer_to_use->output_type[i] != M)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MAX ) );

		delay(2000);

		for (uint8_t i = 0; i < 8; i++)
			if (mixer_to_use->output_type[i] != M)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MIN ) );

		delay(2000);

		set_param_uint( PARAM_DO_ESC_CAL, 0);

		write_params();
	}
}

static int32_t int32_constrain(int32_t i, const int32_t min, const int32_t max) {
	return (i < min) ? min : (i > max) ? max : i;
}

//Direct write to the motor with failsafe checks
//1000 <= value <= 2000
//value_disarm (for motors) should be 1000
void write_output_pwm(uint8_t index, uint32_t value, uint32_t value_disarm) {
	if( safety_is_armed() ) {
		_pwm_output[index] = int32_constrain(value, 1000, 2000);
	} else {
		_pwm_output[index] = value_disarm;
	}

	pwmWriteMotor(index, _pwm_output[index]);
}

//TODO: Maybe this logic should be checked elsewhere?
//Write a pwm value to the motor channel, value should be between 0 and 1000
void write_motor(uint8_t index, uint32_t value) {
	value = int32_constrain(value, 0, 1000) + 1000;

	//If there is an idle set
	if( value < get_param_uint(PARAM_MOTOR_PWM_IDLE) )
		value = get_param_uint(PARAM_MOTOR_PWM_IDLE);

	write_output_pwm(index, value, get_param_uint(PARAM_MOTOR_PWM_MIN));
}

//TODO: Is this even needed? (Tricopters)
//Write a pwm value to the motor channel, value should be between -500 and 500
void write_servo(uint8_t index, int32_t value) {
	value = int32_constrain(value, -500, 500) + 1500;	//TODO: Make and use servo_min, servo_max, and servo_mid an actual parameter

	write_output_pwm(index, value, 1500);	//TODO: Failsafe param here as well
}

//Used to send a PWM while
static void pwm_output() {
	// Add in GPIO inputs from Onboard Computer
	for (int8_t i=0; i<8; i++) {
		output_type_t output_type = mixer_to_use->output_type[i];

		//TODO: This logic needs to be double checked
		if (output_type == NONE) {
			// Incorporate GPIO on not already reserved outputs
			_pwm_output_requested[i] = _GPIO_outputs[i];
			output_type = _GPIO_output_type[i];
		}

		// Write output to motors
		if (output_type == S) {
			//write_servo(i, _pwm_output_requested[i]);
		} else if (output_type == M) {
			write_motor(i, _pwm_output_requested[i]);
		} else if (output_type == G) {
			//write_servo(i, _pwm_output_requested[i]);	//XXX: Could have another function here to handle GPIO cases
		}
	}
}

//TODO: Need to do fix16 operations in this section
void mixer_output() {
	int32_t max_output = 1000;
	int32_t scale_factor = 1000;
	int32_t prescaled_outputs[8];

	for (uint8_t i = 0; i < 8; i++) {
		//TODO: This logic needs to be double checked
		if (mixer_to_use->output_type[i] != NONE) {
			// Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
			//TODO: This might actually be very easy to do with fix16 matrix operations...
			/*
			float thrust_calc = (_control_output.T * mixer_to_use.T[i])
								+ (_control_output.r * mixer_to_use.x[i])
								+ (_control_output.p * mixer_to_use.y[i])
								+ (_control_output.y * mixer_to_use.z[i]);
			prescaled_outputs[i] = (int32_t)(thrust_calc * 1000.0f);
			*/
			fix16_t thrust_calc = fix16_add(fix16_mul(_control_output.T, mixer_to_use->T[i]),
								  fix16_add(fix16_mul(_control_output.r, mixer_to_use->x[i]),
								  fix16_add(fix16_mul(_control_output.p, mixer_to_use->y[i]),
											fix16_mul(_control_output.y, mixer_to_use->z[i]))));

			prescaled_outputs[i] = fix16_to_int(fix16_mul(thrust_calc, _fc_1000));
			//Note: Negitive PWM values could be calculated here, but will be saturated to 0pwm later

			if( mixer_to_use->output_type[i] == M ) {
				if( prescaled_outputs[i] > max_output )
					max_output = prescaled_outputs[i];

				//If the thrust is 0, zero motor outputs, as we don't want any thrust at all for safety
				if( _control_output.T <= 0 )
					prescaled_outputs[i] = 0;
			}
		}
	}

	//TODO: Need to check if this still holds
	// saturate outputs to maintain controllability even during aggressive maneuvers
	if (max_output > 1000)
		scale_factor = 1000 * 1000 / max_output;

	for (uint8_t i=0; i<8; i++) {
		if (mixer_to_use->output_type[i] == M) {
			_pwm_output_requested[i] = prescaled_outputs[i] * scale_factor / 1000; // divide by scale factor
		} else {
			_pwm_output_requested[i] = prescaled_outputs[i];
		}

		if( _system_status.sensors.pwm_control.health == SYSTEM_HEALTH_OK ) {
			//If the user wants to control this channel
			if( (_pwm_control[i] != 0xFFFF) && (_pwm_control[i] != 0) ) {
				//Get the pwm_override, and clamp it
				int32_t pwm_override = int32_constrain(_pwm_control[i],
													   get_param_uint(PARAM_MOTOR_PWM_MIN),
													   get_param_uint(PARAM_MOTOR_PWM_MAX));
				//Get requested pwm from 0 to 1000
				_pwm_output_requested[i] = pwm_override - get_param_uint(PARAM_MOTOR_PWM_MIN);
			}
		}
	}

	pwm_output();
}
