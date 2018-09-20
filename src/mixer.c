#include <stdint.h>
#include <stdio.h>

#include "breezystm32.h"
#include "drv_pwm.h"
#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"
#include "fix16.h"
#include "fixextra.h"

#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "controller.h"

#include "io_type.h"

#include "mixers/mixer_none.h"
#include "mixers/mixer_direct.h"
#include "mixers/mixer_mc_p4.h"
#include "mixers/mixer_mc_x4.h"
#include "mixers/mixer_mc_x6.h"
#include "mixers/mixer_fw_std.h"


control_output_t _control_output;
system_status_t _system_status;

bool _actuator_apply_g0_map[MIXER_NUM_MOTORS];
//XXX: Same as g0: bool _actuator_apply_g1_map[MIXER_NUM_MOTORS];
static bool _actuator_apply_g2_map[MIXER_NUM_MOTORS];
static bool _actuator_apply_g3_map[MIXER_NUM_MOTORS];
static bool _actuator_apply_g4_map[MIXER_NUM_MOTORS];
static bool _actuator_apply_g5_map[MIXER_NUM_MOTORS];

static fix16_t _actuator_control_g0m[MIXER_NUM_MOTORS];	//Mixer calculated g0 controls
fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g3[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g4[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g5[MIXER_NUM_MOTORS];

//int32_t _pwm_control[MIXER_NUM_MOTORS];
//int32_t _pwm_output_requested[MIXER_NUM_MOTORS];
int32_t _pwm_output[MIXER_NUM_MOTORS];
//mixer_motor_test_t _motor_test;

static const mixer_t *mixer_to_use;
static io_type_t _actuator_type_map[MIXER_NUM_MOTORS];

//XXX: Pinout Mapping for Naze32!
static const uint8_t io_map_naze32_ppm = 0;
static const uint8_t io_map_naze32_pwm[MIXER_NUM_MOTORS] = {8, 9, 10, 11, 12, 13, 4, 5};

static int32_t int32_constrain(int32_t i, const int32_t min, const int32_t max) {
	return (i < min) ? min : (i > max) ? max : i;
}

static uint32_t map_fix16_to_pwm_dual(fix16_t f) {
	fix16_t fc = fix16_constrain(f, -_fc_1, _fc_1);
	//range is -500 to 500
	fix16_t pwm_range = fix16_from_int( (get_param_uint(PARAM_MOTOR_PWM_MAX) - get_param_uint(PARAM_MOTOR_PWM_MIN) ) / 2 );
	//Returns 1000 to 2000
	return fix16_to_int(fix16_mul(fc, pwm_range)) + 1500;
}

static uint32_t map_fix16_to_pwm(fix16_t f) {
	fix16_t fc = fix16_constrain(f, 0, _fc_1);
	//range is -500 to 500
	fix16_t pwm_range = fix16_from_int( get_param_uint(PARAM_MOTOR_PWM_MAX) - get_param_uint(PARAM_MOTOR_PWM_MIN) );
	//Returns 1000 to 2000
	return fix16_to_int(fix16_mul(fc, pwm_range)) + 1000;
}

void mixer_init() {
	switch( get_param_uint(PARAM_MIXER) ) {
		case MIXER_DIRECT: {
			mixer_to_use = &mixer_direct;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_GENERIC);
			mavlink_queue_broadcast_notice("[MIXER] Using mixer DIRECT");

			break;
		}
		case MIXER_QUADROTOR_PLUS: {
			mixer_to_use = &mixer_quadrotor_plus;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR);
			mavlink_queue_broadcast_notice("[MIXER] Using mixer QUAD +");

			break;
		}
		case MIXER_QUADROTOR_X: {
			mixer_to_use = &mixer_quadrotor_x;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR);
			mavlink_queue_broadcast_notice("[MIXER] Using mixer QUAD X");

			break;
		}
		case MIXER_HEXAROTOR_X: {
			mixer_to_use = &mixer_hexarotor_x;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_HEXAROTOR);
			mavlink_queue_broadcast_notice("[MIXER] Using mixer HEX X");

			break;
		}
		case MIXER_PLANE_STANDARD: {
			mixer_to_use = &mixer_plane_standard;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_FIXED_WING);
			mavlink_queue_broadcast_notice("[MIXER] Using mixer PLANE STANDARD");

			break;
		}
		default: {
			mixer_to_use = &mixer_none;
			set_param_uint(PARAM_MAV_TYPE, MAV_TYPE_GENERIC);
			mavlink_queue_broadcast_error("[MIXER] Unknown mixer! Disabling!");

			break;
		}
	}

	char text_map[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[MIXER] Layout: ";

	for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++) {
		_pwm_control[i] = 0;
		//_pwm_output_requested[i] = 0;
		_pwm_output[i] = 0;

		//Calculate final actuator mapping
		//First handle output mixers, then auxiliaries
		if(mixer_to_use->output_type[i] != IO_TYPE_N) {
			_actuator_type_map[i] = mixer_to_use->output_type[i];
			_actuator_apply_g0_map[i] = true;
			//_actuator_apply_g1_map[i] = true;
		} else if( ( get_param_uint(PARAM_ACTUATORS_RC_PWM_MAP) >> i ) & 0x01) {
			_actuator_type_map[i] = IO_TYPE_OS;
			_actuator_apply_g2_map[i] = true;
		} else if( ( get_param_uint(PARAM_ACTUATORS_RC_DIGITAL_MAP) >> i ) & 0x01) {
			_actuator_type_map[i] = IO_TYPE_OD;
			_actuator_apply_g3_map[i] = true;
		} else if( ( get_param_uint(PARAM_ACTUATORS_OB_PWM_MAP) >> i ) & 0x01) {
			_actuator_type_map[i] = IO_TYPE_OS;
			_actuator_apply_g4_map[i] = true;
		} else if( ( get_param_uint(PARAM_ACTUATORS_OB_DIGITAL_MAP) >> i ) & 0x01) {
			_actuator_type_map[i] = IO_TYPE_OD;
			_actuator_apply_g5_map[i] = true;
		}

		//Set initial actuator outputs
		_actuator_control_g0m[i] = 0;
		_actuator_control_g0[i] = 0;
		_actuator_control_g1[i] = 0;

		if( get_param_uint(PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT) ) {
			_actuator_control_g2[i] = 0;
			_actuator_control_g3[i] = 0;
			_actuator_control_g4[i] = 0;
			_actuator_control_g5[i] = 0;
		} else {
			_actuator_control_g2[i] = get_param_fix16(PARAM_ACTUATORS_RC_PWM_DISARM_VALUE);
			_actuator_control_g3[i] = get_param_uint(PARAM_ACTUATORS_RC_DIGITAL_DISARM_VALUE);
			_actuator_control_g4[i] = get_param_fix16(PARAM_ACTUATORS_OB_PWM_DISARM_VALUE);
			_actuator_control_g5[i] = get_param_uint(PARAM_ACTUATORS_OB_DIGITAL_DISARM_VALUE);
		}

		if (_actuator_type_map[i] == IO_TYPE_N) {
			strncat(text_map,
					 "-,",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if(_actuator_type_map[i] == IO_TYPE_OD) {
			strncat(text_map,
					 "D,",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if(_actuator_type_map[i] == IO_TYPE_OM) {
			strncat(text_map,
					 "M,",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else if (_actuator_type_map[i] == IO_TYPE_OS) {
			strncat(text_map,
					 "S,",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		} else {
			strncat(text_map,
					 "?",
					 MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
		}
	}

	mavlink_queue_broadcast_notice(text_map);

	_motor_test.start = 0;
	_motor_test.throttle = 0;
	_motor_test.duration = 0;
	_motor_test.test_all = false;
	_motor_test.motor_step = 0;
}

void pwm_init() {
	//XXX: Loop backwards through the IO map to set the number of motor/servo ports
	uint8_t io_c = 0;
	io_def_t io_map;
	io_map.port[io_c] = io_map_naze32_ppm;	//IO set for PPM input
	io_map.type[io_c] = IO_TYPE_IP;	//IO set for PPM input
	io_c++;

	for(int i=0; i<MIXER_NUM_MOTORS; i++) {
		io_map.port[io_c] = io_map_naze32_pwm[i];	//IO map set for PWM input
		//io_map.type[io_c] = (mixer_to_use->output_type[i] == IO_TYPE_OM ) ? IO_TYPE_OM : IO_TYPE_OS;	//IO map set for PWM input
		io_map.type[io_c] = _actuator_type_map[i];
		io_c++;
	}

	//Fill out the rest of the ports to not be initialized
	while(io_c < PWM_MAX_PORTS) {
		io_map.port[io_c] = 0;
		io_map.type[io_c] = IO_TYPE_N;
		io_c++;
	}

	pwmInit(&io_map,
			false,
			get_param_uint(PARAM_MOTOR_PWM_SEND_RATE),
			get_param_uint(PARAM_SERVO_PWM_SEND_RATE),
			get_param_uint(PARAM_MOTOR_PWM_MIN));

	//==-- Perform remaining PWM setup tasks
	if( get_param_uint( PARAM_DO_ESC_CAL ) ) {
		mavlink_send_broadcast_statustext(MAV_SEVERITY_NOTICE, "[MIXER] Performing ESC calibration");

		for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++)
			if (mixer_to_use->output_type[i] == IO_TYPE_OM)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MAX ) );

		LED1_OFF;
		for (uint8_t i = 0; i < 20; i++) {
			LED0_TOGGLE;
			delay(100);
		}

		for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++)
			if (mixer_to_use->output_type[i] == IO_TYPE_OM)
				pwmWriteMotor(i, get_param_uint( PARAM_MOTOR_PWM_MIN ) );

		LED0_OFF;
		LED1_OFF;

		status_buzzer_success();
		mavlink_send_broadcast_statustext(MAV_SEVERITY_INFO, "[MIXER] ESC calibration complete!");

		set_param_uint( PARAM_DO_ESC_CAL, 0 );

		write_params();
	}
}

//Direct write to the motor with failsafe checks
//1000 <= value <= 2000
//value_disarm (for motors) should be 1000
static void write_output_pwm(uint8_t index, uint32_t value, uint32_t value_disarm) {
	if( safety_is_armed() ) {
		_pwm_output[index] = value;
	} else {
		_pwm_output[index] = value_disarm;
	}

	pwmWriteMotor(index, _pwm_output[index]);
}

//TODO: Maybe this logic should be checked elsewhere?
//Write a pwm value to the motor channel, value should be between 0 and 1
static void write_motor(uint8_t index, fix16_t value) {
	uint16_t pwm = map_fix16_to_pwm(value);

	//If there is an idle set
	if( pwm < get_param_uint(PARAM_MOTOR_PWM_IDLE) )
		pwm = get_param_uint(PARAM_MOTOR_PWM_IDLE);

	write_output_pwm(index, pwm, get_param_uint(PARAM_MOTOR_PWM_MIN));
}

//TODO: Is this even needed? (Tricopters)
//Write a pwm value to the motor channel, value should be between -1 and 1
static void write_servo(uint8_t index, fix16_t value) {
	uint16_t pwm = map_fix16_to_pwm_dual(value);
	uint16_t pwm_mid = get_param_uint(PARAM_MOTOR_PWM_MIN) + ( (get_param_uint(PARAM_MOTOR_PWM_MAX) - get_param_uint(PARAM_MOTOR_PWM_MIN)) / 2 );

	write_output_pwm(index, pwm, pwm_mid);
}

static void write_aux_pwm(uint8_t index, fix16_t value, bool respect_arm, fix16_t value_disarm) {
	uint16_t pwm_act_disarm = 0;
	uint16_t pwm_act_out = int32_constrain( map_fix16_to_pwm_dual( value ),
															  get_param_uint(PARAM_MOTOR_PWM_MIN),
															  get_param_uint(PARAM_MOTOR_PWM_MAX) );

	if( respect_arm ) {
		if( !get_param_uint(PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT) )
			pwm_act_disarm = int32_constrain( map_fix16_to_pwm_dual( value_disarm ),
															  get_param_uint(PARAM_MOTOR_PWM_MIN),
															  get_param_uint(PARAM_MOTOR_PWM_MAX) );
	} else {
		pwm_act_disarm = pwm_act_out;
	}

	write_output_pwm(index, pwm_act_out, pwm_act_disarm);
}

static void write_aux_digital(uint8_t index, bool value, bool respect_arm, bool value_disarm) {
	//XXX:
	//The digital write is controlled as a PWM signal still
	//but instead of all the fuss, we just fill up the
	//pulse to be on longer than the update rate (+ a little)
	uint16_t pwm_digital_on = 1000 + 1000*(1000/get_param_uint(PARAM_SERVO_PWM_SEND_RATE));
	uint16_t pwm_digital_off = 0;

	uint16_t pwm_act_disarm = 0;
	uint16_t pwm_act_out = value ? pwm_digital_on : pwm_digital_off;

	if( respect_arm ) {
		if( !get_param_uint(PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT) )
			pwm_act_disarm = value_disarm ? pwm_digital_on : pwm_digital_off;
	} else {
		pwm_act_disarm = pwm_act_out;
	}

	write_output_pwm(index, pwm_act_out, pwm_act_disarm);
}

//Used to send a PWM while
void pwm_output() {
	if(mixer_to_use->mixer_ok) {
		for (int8_t i=0; i<MIXER_NUM_MOTORS; i++) {
			if(_actuator_apply_g0_map[i]) {
				//Handle mixer output
				io_type_t output_type = mixer_to_use->output_type[i];

				fix16_t val = 0;

				//TODO: _actuator_control_g0
				//TODO: _actuator_control_g1
				val = _actuator_control_g0m[i];

				if( output_type == IO_TYPE_OM ) {
					write_motor(i, val);
				} else {
					write_servo(i, val);
				}
			} else if(_actuator_apply_g2_map[i]) {
				//Handle Aux RC PWM
				write_aux_pwm(i,
							  _actuator_control_g2[i],
							  get_param_uint(PARAM_ACTUATORS_RC_RESPECT_ARM),
							  get_param_fix16(PARAM_ACTUATORS_RC_PWM_DISARM_VALUE) );
			} else if(_actuator_apply_g3_map[i]) {
				//Handle Aux OB Digital
				bool val = (_actuator_control_g3[i] > 0);
				write_aux_digital(i,
								  val,
								  get_param_uint(PARAM_ACTUATORS_RC_RESPECT_ARM),
								  get_param_uint(PARAM_ACTUATORS_RC_DIGITAL_DISARM_VALUE) );
			} else if(_actuator_apply_g4_map[i]) {
				//Handle Aux OB PWM
				write_aux_pwm(i,
							  _actuator_control_g4[i],
							  get_param_uint(PARAM_ACTUATORS_OB_RESPECT_ARM),
							  get_param_fix16(PARAM_ACTUATORS_OB_PWM_DISARM_VALUE) );
			} else if(_actuator_apply_g5_map[i]) {
				//Handle Aux OB Digital
				bool val = (_actuator_control_g5[i] > 0);
				write_aux_digital(i,
								  val,
								  get_param_uint(PARAM_ACTUATORS_OB_RESPECT_ARM),
								  get_param_uint(PARAM_ACTUATORS_OB_DIGITAL_DISARM_VALUE) );
			} else {
				//Disable output
				write_output_pwm(i, 0, 0);
			}
		}
	} else {
		for (int8_t i=0; i<MIXER_NUM_MOTORS; i++) {
			write_output_pwm(i, 0, 0);
		}
	}
}

//TODO: Need to do fix16 operations in this section
void calc_mixer_output() {
	fix16_t max_output = 0;
	fix16_t scale_factor = _fc_1;
	fix16_t prescaled_outputs[MIXER_NUM_MOTORS];

	for (uint8_t i = 0; i < MIXER_NUM_MOTORS; i++) {
		//If we're using the mixer io for motor/servo output
		//and there is some throttle input
		if( ( (mixer_to_use->output_type[i] == IO_TYPE_OM) ||
			  (mixer_to_use->output_type[i] == IO_TYPE_OS) ) &&
			( _control_output.T > 0 ) ) {

			//Matrix multiply
			prescaled_outputs[i] = fix16_add(fix16_mul(_control_output.T, mixer_to_use->T[i]),
								   fix16_add(fix16_mul(_control_output.r, mixer_to_use->x[i]),
								   fix16_add(fix16_mul(_control_output.p, mixer_to_use->y[i]),
											fix16_mul(_control_output.y, mixer_to_use->z[i]))));

			if( mixer_to_use->output_type[i] == IO_TYPE_OM ) {
				if( prescaled_outputs[i] > max_output )
					max_output = prescaled_outputs[i];
			}
		} else {
			//zero motor outputs as we don't want any output at all for safety
			prescaled_outputs[i] = 0;
		}
	}

	//TODO: Need to check if this still holds
	// saturate outputs to maintain controllability even during aggressive maneuvers
	if (max_output > _fc_1)
		scale_factor = _fc_1 / max_output;

	for (uint8_t i=0; i<MIXER_NUM_MOTORS; i++) {
		if (mixer_to_use->output_type[i] == IO_TYPE_OM) {
			_actuator_control_g0m[i] = fix16_mul( prescaled_outputs[i], scale_factor );
		} else {
			_actuator_control_g0m[i] = prescaled_outputs[i];
		}
	}
}
