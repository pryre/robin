#include <stdint.h>

#include "breezystm32.h"
#include "pwm.h"
#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"
#include "fix16.h"
#include "fixextra.h"

#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "controller.h"

#include "mixers/mixer_none.h"
#include "mixers/mixer_mc_p4.h"
#include "mixers/mixer_mc_x4.h"
#include "mixers/mixer_mc_x6.h"
#include "mixers/mixer_fw_std.h"


control_output_t _control_output;
system_status_t _system_status;

int32_t _GPIO_outputs[MIXER_NUM_MOTORS];
output_type_t _GPIO_output_type[MIXER_NUM_MOTORS];

int8_t _actuator_apply_g1_map[MIXER_NUM_MOTORS];
bool _actuator_apply_g2;
fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];

int32_t _pwm_control[MIXER_NUM_MOTORS];
int32_t _pwm_output_requested[MIXER_NUM_MOTORS];
int32_t _pwm_output[MIXER_NUM_MOTORS];
mixer_motor_test_t _motor_test;

static const mixer_t *mixer_to_use;

static int32_t int32_constrain(int32_t i, const int32_t min, const int32_t max) {
	return (i < min) ? min : (i > max) ? max : i;
}

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

	_actuator_apply_g1_map[0] = -1;
	_actuator_apply_g1_map[1] = -1;
	_actuator_apply_g1_map[2] = -1;
	_actuator_apply_g1_map[3] = -1;
	_actuator_apply_g1_map[4] = get_param_uint(PARAM_RC_MAP_PASSTHROUGH_AUX1) - 1;
	_actuator_apply_g1_map[5] = get_param_uint(PARAM_RC_MAP_PASSTHROUGH_AUX2) - 1;
	_actuator_apply_g1_map[6] = get_param_uint(PARAM_RC_MAP_PASSTHROUGH_AUX3) - 1;
	_actuator_apply_g1_map[7] = get_param_uint(PARAM_RC_MAP_PASSTHROUGH_AUX4) - 1;

	_actuator_apply_g2 = false;

	for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++) {
		_pwm_control[i] = 0;
		_pwm_output_requested[i] = 0;
		_pwm_output[i] = 0;
		_GPIO_outputs[i] = 0;
		_GPIO_output_type[i] = MT_NONE;

		//Ensure that RC Aux mapping is either disabled (-1), or a valid channel
		int8_t rca_map = _actuator_apply_g1_map[i];
		_actuator_apply_g1_map[i] = ( (rca_map < -1) || (rca_map > (MIXER_NUM_MOTORS -1) ) ) ? -1 : rca_map;

		//Set initial actuator outputs
		_actuator_control_g0[i] = 0;
		_actuator_control_g1[i] = 0;
		_actuator_control_g2[i] = 0;
	}

	_motor_test.start = 0;
	_motor_test.throttle = 0;
	_motor_test.duration = 0;
	_motor_test.test_all = false;
	_motor_test.motor_step = 0;
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
		mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[MIXER] Performing ESC calibration");

		for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++)
			if (mixer_to_use->output_type[i] == MT_M)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MAX ) );

		LED1_OFF;
		for (uint8_t i = 0; i < 20; i++) {
			LED0_TOGGLE;
			delay(100);
		}

		for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++)
			if (mixer_to_use->output_type[i] == MT_M)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MIN ) );

		LED0_OFF;
		LED1_OFF;

		status_buzzer_success();
		mavlink_send_broadcast_statustext(MAV_SEVERITY_INFO, "[MIXER] ESC calibration complete!");

		set_param_uint( PARAM_DO_ESC_CAL, 0 );

		write_params();
	}
}

static uint32_t map_fix16_to_pwm(fix16_t f) {
	fix16_t fc = fix16_constrain(f,-_fc_1,_fc_1);
	//range is -500 to 500
	fix16_t pwm_range = fix16_from_int( (get_param_uint(PARAM_MOTOR_PWM_MAX) - get_param_uint(PARAM_MOTOR_PWM_MIN) ) / 2 );
	//Returns 1000 to 2000
	return fix16_to_int(fix16_mul(fc, pwm_range)) + 1500;
}

//Direct write to the motor with failsafe checks
//1000 <= value <= 2000
//value_disarm (for motors) should be 1000
void write_output_pwm(uint8_t index, uint32_t value, uint32_t value_disarm) {
	if( safety_is_armed() ) {
		_pwm_output[index] = value;
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
	for (int8_t i=0; i<MIXER_NUM_MOTORS; i++) {
		output_type_t output_type = mixer_to_use->output_type[i];

		//TODO: This logic needs to be double checked
		if (output_type == MT_NONE) {
			// Incorporate GPIO on not already reserved outputs
			_pwm_output_requested[i] = _GPIO_outputs[i];
			output_type = _GPIO_output_type[i];
		}

		//XXX: This is getting pretty messy, but it looks like a good spot for AUX passthrough as well

		// Write output to motors
		if (output_type == MT_S) {
			//write_servo(i, _pwm_output_requested[i]);
		} else if (output_type == MT_M) {
			write_motor(i, _pwm_output_requested[i]);
		} else if( _actuator_apply_g1_map[i] && (_system_status.sensors.rc_input.health == SYSTEM_HEALTH_OK) ) {
			//RC aux. actuator control
			uint16_t pwm_act_disarm = 0;
			uint16_t pwm_act_out = int32_constrain( map_fix16_to_pwm( _actuator_control_g1[i] ),
																	  get_param_uint(PARAM_MOTOR_PWM_MIN),
																	  get_param_uint(PARAM_MOTOR_PWM_MAX) );

			if( get_param_uint(PARAM_ACTUATORS_RC_RESPECT_ARM) ) {
				if( !get_param_uint(PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT) ) {
					uint16_t pwmd = map_fix16_to_pwm( get_param_fix16(PARAM_ACTUATORS_RC_DISARM_VALUE) );
					pwm_act_disarm = int32_constrain( pwmd,
													  get_param_uint(PARAM_MOTOR_PWM_MIN),
													  get_param_uint(PARAM_MOTOR_PWM_MAX) );
				}
			} else {
				pwm_act_disarm = pwm_act_out;
			}

			write_output_pwm(i, pwm_act_out, pwm_act_disarm);
		} else if( _actuator_apply_g2 ) {
			//Offboard aux. actuator control
			uint16_t pwm_act_disarm = 0;
			uint16_t pwm_act_out = int32_constrain( map_fix16_to_pwm( _actuator_control_g2[i] ),
																	  get_param_uint(PARAM_MOTOR_PWM_MIN),
																	  get_param_uint(PARAM_MOTOR_PWM_MAX) );

			if( get_param_uint(PARAM_ACTUATORS_OB_RESPECT_ARM) ) {
				if( !get_param_uint(PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT) ) {
					uint16_t pwmd = map_fix16_to_pwm( get_param_fix16(PARAM_ACTUATORS_OB_DISARM_VALUE) );
					pwm_act_disarm = int32_constrain( pwmd,
													  get_param_uint(PARAM_MOTOR_PWM_MIN),
													  get_param_uint(PARAM_MOTOR_PWM_MAX) );
				}
			} else {
				pwm_act_disarm = pwm_act_out;
			}

			write_output_pwm(i, pwm_act_out, pwm_act_disarm);
		} else if (output_type == MT_G) {
			//write_servo(i, _pwm_output_requested[i]);	//XXX: Could have another function here to handle GPIO cases
		} else {
			//Disable output
			write_output_pwm(i, 0, 0);
		}
	}
}

//TODO: Need to do fix16 operations in this section
void mixer_output() {
	int32_t max_output = 1000;
	int32_t scale_factor = 1000;
	int32_t prescaled_outputs[MIXER_NUM_MOTORS];

	for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++) {
		//TODO: This logic needs to be double checked
		if (mixer_to_use->output_type[i] != MT_NONE) {
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

			if( mixer_to_use->output_type[i] == MT_M ) {
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

	//Handle motor testing
	if(_motor_test.start > 0) {
		if( micros() < (_motor_test.start + _motor_test.duration) ) {
			//Test in progress
			fix16_t test_pwm_range = fix16_from_int(get_param_uint(PARAM_MOTOR_PWM_MAX) - get_param_uint(PARAM_MOTOR_PWM_MIN));
			uint16_t test_pwm = fix16_to_int(fix16_mul(_motor_test.throttle, test_pwm_range)) + get_param_uint(PARAM_MOTOR_PWM_MIN);

			//Override PWM control
			for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++) {
				_pwm_control[i] = 0;
			}

			_pwm_control[_motor_test.motor_step] = test_pwm;
			safety_update_sensor(&_system_status.sensors.pwm_control);
		} else {
			//If there are motors left to test
			if( (_motor_test.test_all) &&
				(_motor_test.motor_step < (MIXER_NUM_MOTORS - 1) ) ) {
					_motor_test.start = micros();
					_motor_test.motor_step++;

					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[MIXER] Testing motor: ";
					char mchar[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					itoa(_motor_test.motor_step, mchar, 10);
					strncat(text, mchar, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1);
					mavlink_queue_broadcast_notice(text);
			} else {
				//Test is done, reset
				_motor_test.start = 0;
				_motor_test.throttle = 0;
				_motor_test.duration = 0;
				_motor_test.test_all = false;
				_motor_test.motor_step = 0;
			}
		}
	}

	//Prepare the motor mixing
	for (uint8_t i=0; i<MIXER_NUM_MOTORS; i++) {
		if (mixer_to_use->output_type[i] == MT_M) {
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

	if(mixer_to_use->mixer_ok)
		pwm_output();
}
