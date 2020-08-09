#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"
#include "fixextra.h"
#include "mavlink/mavlink_types.h"
#include "mavlink_system.h"

#include "calibration.h"
#include "control.h"
#include "drivers/drv_pwm.h"
#include "mixer.h"
#include "params.h"
#include "safety.h"
#include "io_type.h"

#include "robin_itoa.h"

#include "mixers/mixer_free.h"
#include "mixers/mixer_fw_std.h"
#include "mixers/mixer_mc_p4.h"
#include "mixers/mixer_mc_x4.h"
#include "mixers/mixer_mc_x6.h"
#include "mixers/mixer_none.h"

//control_output_t _control_output;
//system_status_t _system_status;

io_type_t _actuator_type_map[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g3[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g4[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g5[MIXER_NUM_MOTORS];

mixer_motor_test_t _motor_test;

static bool actuator_apply_g0_map_[MIXER_NUM_MOTORS];
// XXX: Same as g0: bool _actuator_apply_g1_map[MIXER_NUM_MOTORS];
static bool actuator_apply_g2_map_[MIXER_NUM_MOTORS];
static bool actuator_apply_g3_map_[MIXER_NUM_MOTORS];
static bool actuator_apply_g4_map_[MIXER_NUM_MOTORS];
static bool actuator_apply_g5_map_[MIXER_NUM_MOTORS];

static fix16_t actuator_control_g0m_[MIXER_NUM_MOTORS]; // Mixer calculated g0 controls

const mixer_t* _mixer_to_use;
static uint8_t num_motors_;

static int32_t int32_constrain( int32_t i, const int32_t min,
								const int32_t max ) {
	return ( i < min ) ? min : ( i > max ) ? max : i;
}

static uint32_t map_fix16_to_pwm( fix16_t f ) {
	fix16_t fc = fix16_constrain( f, 0, _fc_1 );
	// range is 0 to 1000
	fix16_t pwm_range = fix16_from_int( get_param_uint( PARAM_MOTOR_PWM_MAX ) - get_param_uint( PARAM_MOTOR_PWM_MIN ) );
	// Returns 1000 to 2000
	return fix16_to_int( fix16_mul( fc, pwm_range ) ) + get_param_uint( PARAM_MOTOR_PWM_MIN );
}

static uint32_t map_fix16_to_pwm_dual( fix16_t f ) {
	fix16_t fc = fix16_constrain( f, -_fc_1, _fc_1 );
	// range is 0 to 500
	uint16_t pwm_range = ( get_param_uint( PARAM_MOTOR_PWM_MAX ) - get_param_uint( PARAM_MOTOR_PWM_MIN ) ) / 2;
	// Returns 1000 to 2000
	return fix16_to_int( fix16_mul( fc, fix16_from_int( pwm_range ) ) ) + pwm_range + get_param_uint( PARAM_MOTOR_PWM_MIN );
}

static void motor_test_reset( void ) {
	_motor_test.start = 0;
	_motor_test.throttle = 0;
	_motor_test.duration = 0;
	_motor_test.test_all = false;
	_motor_test.motor_step = 0;
}

void mixer_init(void) {
	num_motors_ = 0;

	mixer_type_t mixer_type = get_param_uint( PARAM_MIXER );
	if(MIXER_NUM_MOTORS > FIXMATRIX_MAX_SIZE) {
		mavlink_queue_broadcast_notice( "[MIXER] INVALID FIRMWARE SETUP, DISABLING!" );
		mixer_type = MIXER_NONE;
	}

	switch ( mixer_type ) {
	case MIXER_FREE: {
		_mixer_to_use = &mixer_free;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_GENERIC );
		mavlink_queue_broadcast_notice( "[MIXER] Using mixer FREE" );

		break;
	}
	case MIXER_QUADROTOR_PLUS: {
		_mixer_to_use = &mixer_quadrotor_plus;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR );
		mavlink_queue_broadcast_notice( "[MIXER] Using mixer QUAD +" );

		break;
	}
	case MIXER_QUADROTOR_X: {
		_mixer_to_use = &mixer_quadrotor_x;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_QUADROTOR );
		mavlink_queue_broadcast_notice( "[MIXER] Using mixer QUAD X" );

		break;
	}
	case MIXER_HEXAROTOR_X: {
		_mixer_to_use = &mixer_hexarotor_x;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_HEXAROTOR );
		mavlink_queue_broadcast_notice( "[MIXER] Using mixer HEX X" );

		break;
	}
	case MIXER_PLANE_STANDARD: {
		_mixer_to_use = &mixer_plane_standard;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_FIXED_WING );
		mavlink_queue_broadcast_notice( "[MIXER] Using mixer PLANE STANDARD" );

		break;
	}
	default: {
		_mixer_to_use = &mixer_none;
		set_param_uint( PARAM_MAV_TYPE, MAV_TYPE_GENERIC );
		mavlink_queue_broadcast_error( "[MIXER] Unknown mixer, disabling!" );

		break;
	}
	}

	// Set up the parameters for the motor test utility
	motor_test_reset();

	char text_map[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[MIXER] Layout: [";

	for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
		// Calculate final actuator mapping
		// First handle output mixers, then auxiliaries
		if ( _mixer_to_use->output_type[i] != IO_TYPE_N ) {
			_actuator_type_map[i] = _mixer_to_use->output_type[i];
			actuator_apply_g0_map_[i] = true;

			//Dynamically keep track of how many motors are mapped
			if(_mixer_to_use->output_type[i] == IO_TYPE_OM)
				num_motors_++;
		} else if ( ( get_param_uint( PARAM_ACTUATORS_RC_PWM_MAP ) >> i ) & 0x01 ) {
			_actuator_type_map[i] = IO_TYPE_OS;
			actuator_apply_g2_map_[i] = true;
		} else if ( ( get_param_uint( PARAM_ACTUATORS_RC_DIGITAL_MAP ) >> i ) & 0x01 ) {
			_actuator_type_map[i] = IO_TYPE_OD;
			actuator_apply_g3_map_[i] = true;
		} else if ( ( get_param_uint( PARAM_ACTUATORS_OB_PWM_MAP ) >> i ) & 0x01 ) {
			_actuator_type_map[i] = IO_TYPE_OS;
			actuator_apply_g4_map_[i] = true;
		} else if ( ( get_param_uint( PARAM_ACTUATORS_OB_DIGITAL_MAP ) >> i ) & 0x01 ) {
			_actuator_type_map[i] = IO_TYPE_OD;
			actuator_apply_g5_map_[i] = true;
		}

		// Set initial actuator outputs
		actuator_control_g0m_[i] = 0;
		_actuator_control_g0[i] = 0;
		_actuator_control_g1[i] = 0;

		if ( get_param_uint( PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT ) ) {
			_actuator_control_g2[i] = 0;
			_actuator_control_g3[i] = 0;
			_actuator_control_g4[i] = 0;
			_actuator_control_g5[i] = 0;
		} else {
			_actuator_control_g2[i] = get_param_fix16( PARAM_ACTUATORS_RC_PWM_DISARM_VALUE );
			_actuator_control_g3[i] = get_param_uint( PARAM_ACTUATORS_RC_DIGITAL_DISARM_VALUE );
			_actuator_control_g4[i] = get_param_fix16( PARAM_ACTUATORS_OB_PWM_DISARM_VALUE );
			_actuator_control_g5[i] = get_param_uint( PARAM_ACTUATORS_OB_DIGITAL_DISARM_VALUE );
		}

		if ( _actuator_type_map[i] == IO_TYPE_N ) {
			strncat( text_map, "-", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		} else if ( _actuator_type_map[i] == IO_TYPE_OD ) {
			strncat( text_map, "D", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		} else if ( _actuator_type_map[i] == IO_TYPE_OM ) {
			strncat( text_map, "M", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		} else if ( _actuator_type_map[i] == IO_TYPE_OS ) {
			strncat( text_map, "S", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		} else {
			strncat( text_map, "?", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		}

		// Prettify output
		if ( i < ( MIXER_NUM_MOTORS - 1 ) ) {
			strncat( text_map, ",", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		} else {
			strncat( text_map, "]", MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1 );
		}
	}

	mavlink_queue_broadcast_notice( text_map );

	drv_pwm_init();

	mixer_clear_outputs();

	//==-- Perform remaining PWM setup tasks
	calibrate_esc();
}

// Direct write to the motor with failsafe checks
// 1000 <= value <= 2000
// value_disarm (for motors) should be 1000
static void write_output_pwm( const uint8_t index, const uint32_t value,
							  const uint32_t value_disarm ) {

	drv_pwm_write( index, ( safety_is_armed() ? value : value_disarm ) );
}

// Write a pwm value to the motor channel, value should be between 0 and 1
static void write_motor( const uint8_t index, const fix16_t value ) {
	fix16_t cmd_throttle = fix16_constrain( value, 0, _fc_1 );

	// Lift = 0.5*Cl*A*(V^2)
	// Kl = 0.5*Cl*A
	// Lift = Kl*(V^2)
	// Throttle controls V, so to get linear lift, need to sqrt V
	if ( get_param_uint( PARAM_MOTOR_PWM_LINEARIZE ) )
		cmd_throttle = fix16_sqrt( cmd_throttle );

	uint16_t pwm = map_fix16_to_pwm( cmd_throttle );

	// If there is an idle set
	if ( pwm < get_param_uint( PARAM_MOTOR_PWM_IDLE ) )
		pwm = get_param_uint( PARAM_MOTOR_PWM_IDLE );

	write_output_pwm( index, pwm, get_param_uint( PARAM_MOTOR_PWM_MIN ) );
}

// Write a pwm value to the motor channel, value should be between -1 and 1
static void write_servo( const uint8_t index, const fix16_t value ) {
	uint16_t pwm = map_fix16_to_pwm_dual( value );
	uint16_t pwm_mid = get_param_uint( PARAM_MOTOR_PWM_MIN ) + ( ( get_param_uint( PARAM_MOTOR_PWM_MAX ) - get_param_uint( PARAM_MOTOR_PWM_MIN ) ) / 2 );

	write_output_pwm( index, pwm, pwm_mid );
}

static void write_aux_pwm( const uint8_t index, const fix16_t value,
						   const bool respect_arm, const fix16_t value_disarm ) {
	uint16_t pwm_act_disarm = 0;
	uint16_t pwm_act_out = int32_constrain( map_fix16_to_pwm_dual( value ),
											get_param_uint( PARAM_MOTOR_PWM_MIN ),
											get_param_uint( PARAM_MOTOR_PWM_MAX ) );

	if ( respect_arm ) {
		if ( !get_param_uint( PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT ) )
			pwm_act_disarm = int32_constrain( map_fix16_to_pwm_dual( value_disarm ),
											  get_param_uint( PARAM_MOTOR_PWM_MIN ),
											  get_param_uint( PARAM_MOTOR_PWM_MAX ) );
	} else {
		pwm_act_disarm = pwm_act_out;
	}

	write_output_pwm( index, pwm_act_out, pwm_act_disarm );
}

static void write_aux_digital( const uint8_t index, const bool value,
							   const bool respect_arm, const bool value_disarm ) {
	// XXX:
	// The digital write is controlled as a PWM signal still
	// but instead of all the fuss, we just fill up the
	// pulse to be on longer than the update rate (+ a little)
	uint16_t pwm_digital_on = 1000 + 1000 * ( 1000 / get_param_uint( PARAM_SERVO_PWM_SEND_RATE ) );
	uint16_t pwm_digital_off = 0;

	uint16_t pwm_act_disarm = 0;
	uint16_t pwm_act_out = value ? pwm_digital_on : pwm_digital_off;

	if ( respect_arm ) {
		if ( !get_param_uint( PARAM_ACTUATORS_AUX_DISARM_ZERO_OUTPUT ) )
			pwm_act_disarm = value_disarm ? pwm_digital_on : pwm_digital_off;
	} else {
		pwm_act_disarm = pwm_act_out;
	}

	write_output_pwm( index, pwm_act_out, pwm_act_disarm );
}

static bool motor_test_in_progress( const uint32_t time_now ) {
	bool in_progress = false;

	// Handle motor testing
	if ( _motor_test.start > 0 ) {
		if ( !safety_switch_engaged() && !safety_is_armed() ) {
			// Check to see if the test should move onto next motor or end
			if ( time_now > ( _motor_test.start + _motor_test.duration ) ) {
				// If there are motors left to test
				if ( ( _motor_test.test_all ) && ( _motor_test.motor_step < ( MIXER_NUM_MOTORS - 1 ) ) ) {
					_motor_test.start = time_now;
					_motor_test.motor_step++;

					char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[SENSOR] Testing motor: ";
					char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1, _motor_test.motor_step + 1 , 10);
					strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
					mavlink_queue_broadcast_notice( text );
				} else {
					// Test is done, reset
					mavlink_queue_broadcast_info( "[MIXER] Motor test complete!" );

					motor_test_reset();
				}
			}

			// If test in progress
			if ( time_now < ( _motor_test.start + _motor_test.duration ) ) {
				in_progress = true;

				// Override mixer inputs for all motors to set to 0
				for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
					if ( actuator_apply_g0_map_[i] )
						actuator_control_g0m_[i] = 0;
				}

				// Apply the test to the one we care about
				actuator_control_g0m_[_motor_test.motor_step] = _motor_test.throttle;
			} else {
			}
		} else {
			// Safety switch was reset or armed, stop test
			mavlink_queue_broadcast_error(
				"[MIXER] Safety status changed, cancelling test" );

			motor_test_reset();
		}
	}

	return in_progress;
}

// Used to send a PWM while
void mixer_run( uint32_t time_now ) {
	bool test_running = motor_test_in_progress( time_now );

	if ( _mixer_to_use->mixer_ok ) {
		for ( int8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
			if ( actuator_apply_g0_map_[i] ) {
				// Handle mixer output
				io_type_t output_type = _mixer_to_use->output_type[i];

				fix16_t val = actuator_control_g0m_[i];

				if ( !test_running ) {
					if ( _system_status.sensors.offboard_mixer_g0_control.health == SYSTEM_HEALTH_OK ) {
						val = _actuator_control_g0[i];
					}

					if ( _system_status.sensors.offboard_mixer_g1_control.health == SYSTEM_HEALTH_OK ) {
						val = fix16_add( val, _actuator_control_g1[i] );
					}

					if ( output_type == IO_TYPE_OM ) {
						write_motor( i, val );
					} else {
						write_servo( i, val );
					}
				} else {
					uint16_t test_pwm = 0;

					if ( output_type == IO_TYPE_OM ) {
						test_pwm = map_fix16_to_pwm( actuator_control_g0m_[i] );
					} else {
						test_pwm = map_fix16_to_pwm_dual( actuator_control_g0m_[i] );
					}

					write_output_pwm( i, test_pwm, test_pwm );
				}
			} else if ( actuator_apply_g2_map_[i] ) {
				// Handle Aux RC PWM
				write_aux_pwm( i, _actuator_control_g2[i],
							   get_param_uint( PARAM_ACTUATORS_RC_RESPECT_ARM ),
							   get_param_fix16( PARAM_ACTUATORS_RC_PWM_DISARM_VALUE ) );
			} else if ( actuator_apply_g3_map_[i] ) {
				// Handle Aux OB Digital
				bool val = ( _actuator_control_g3[i] > 0 );
				write_aux_digital(
					i, val, get_param_uint( PARAM_ACTUATORS_RC_RESPECT_ARM ),
					get_param_uint( PARAM_ACTUATORS_RC_DIGITAL_DISARM_VALUE ) );
			} else if ( actuator_apply_g4_map_[i] ) {
				// Handle Aux OB PWM
				write_aux_pwm( i, _actuator_control_g4[i],
							   get_param_uint( PARAM_ACTUATORS_OB_RESPECT_ARM ),
							   get_param_fix16( PARAM_ACTUATORS_OB_PWM_DISARM_VALUE ) );
			} else if ( actuator_apply_g5_map_[i] ) {
				// Handle Aux OB Digital
				bool val = ( _actuator_control_g5[i] > 0 );
				write_aux_digital(
					i, val, get_param_uint( PARAM_ACTUATORS_OB_RESPECT_ARM ),
					get_param_uint( PARAM_ACTUATORS_OB_DIGITAL_DISARM_VALUE ) );
			} else {
				// Disable output
				write_output_pwm( i, 0, 0 );
			}
		}
	} else {
		for ( int8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
			write_output_pwm( i, 0, 0 );
		}
	}
}

//XXX: c is the input signal: c = [tx; ty; tz; Tz]
void mixer_set_primary_using_map( const mf16* c ) {
	fix16_t max_output = 0;
	fix16_t scale_factor = _fc_1;
	//fix16_t prescaled_outputs[MIXER_NUM_MOTORS];

	/*
	for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
		// If we're using the mixer io for motor/servo output
		// and there is some throttle input
		if ( ( ( _mixer_to_use->output_type[i] == IO_TYPE_OM ) || ( _mixer_to_use->output_type[i] == IO_TYPE_OS ) ) && ( _control_output.T > 0 ) ) {

			// Matrix multiply
			prescaled_outputs[i] = fix16_add(
				fix16_mul( _control_output.T, _mixer_to_use->T[i] ),
				fix16_add(
					fix16_mul( _control_output.r, _mixer_to_use->x[i] ),
					fix16_add( fix16_mul( _control_output.p, _mixer_to_use->y[i] ),
							   fix16_mul( _control_output.y, _mixer_to_use->z[i] ) ) ) );

			if ( _mixer_to_use->output_type[i] == IO_TYPE_OM ) {
				if ( prescaled_outputs[i] > max_output )
					max_output = prescaled_outputs[i];
			}

		} else {
			// zero motor outputs as we don't want any output at all for safety
			prescaled_outputs[i] = 0;
		}
	}
	*/

	//Calculate our input signals
	mf16 u = {.rows = MIXER_NUM_MOTORS, .columns = 1, .errors = 0};
	mf16_mul(&u, &(_mixer_to_use->map), c);

	//Check to see what our maximum motor output was
	for(uint8_t i = 0; i < MIXER_NUM_MOTORS; i++)
		if ( _mixer_to_use->output_type[i] == IO_TYPE_OM )
				if ( u.data[i][0] > max_output )
					max_output = u.data[i][0];

	// saturate outputs to maintain controllability even during aggressive
	// maneuvers
	if ( max_output > _fc_1 )
		scale_factor = fix16_div(_fc_1, max_output);

	//
	for ( uint8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
		if ( _mixer_to_use->output_type[i] == IO_TYPE_OM ) {
			actuator_control_g0m_[i] = fix16_mul( u.data[i][0], scale_factor );
		} else {
			actuator_control_g0m_[i] = u.data[i][0];
		}
	}
}

void mixer_clear_outputs( void ) {
	for ( int8_t i = 0; i < MIXER_NUM_MOTORS; i++ ) {
		write_output_pwm( i, 0, 0 );
	}
}

uint8_t mixer_get_num_motors( void ) {
	return num_motors_;
}

#ifdef __cplusplus
}
#endif
