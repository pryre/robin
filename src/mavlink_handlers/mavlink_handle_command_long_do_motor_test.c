#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"
#include "mixer.h"

#include "fix16.h"
#include "fixextra.h"

#include "drivers/drv_system.h"

const mixer_t *_mixer_to_use;
mixer_motor_test_t _motor_test;

MAV_RESULT mavlink_handle_command_long_do_motor_test( mavlink_channel_t chan, float *params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	if( !_mixer_to_use->mixer_ok ) {
		mavlink_queue_broadcast_error("[MIXER] No valid mixer selected!");

		command_result = MAV_RESULT_DENIED;
	} else if( safety_switch_engaged() || safety_is_armed() ) {
		mavlink_queue_broadcast_error("[MIXER] Motor test requires saftety off, not armed");

		command_result = MAV_RESULT_DENIED;
	} else {
		uint8_t motor_test_number = (int)params[0];
		//fix16_t motor_test_type = fix16_from_float(params[1]);
		fix16_t motor_test_throttle = fix16_from_float(params[2]);
		fix16_t motor_test_timeout = fix16_from_float(params[3]);
		//uint8_t motor_test_count = (int)params[4];
		//uint8_t motor_test_order = (int)params[5];

		if( ( (motor_test_number < MIXER_NUM_MOTORS) ||
			  (motor_test_number == MIXER_TEST_MOTORS_ALL) ) &&
			( (motor_test_throttle > 0) && (motor_test_throttle < _fc_1) ) &&
			( motor_test_timeout > 0 ) ) {

			//Configure motor test
			if(motor_test_number == MIXER_TEST_MOTORS_ALL) {
				_motor_test.test_all = true;
				_motor_test.motor_step = 0;
			} else {
				_motor_test.motor_step = motor_test_number;
			}

			_motor_test.start = system_micros();
			_motor_test.duration = 10000*fix16_to_int(fix16_mul(motor_test_timeout, _fc_100)); //XXX: Motor test should be accurate to 0.01 seconds, and max out at ~1min
			_motor_test.throttle = motor_test_throttle;

			//XXX: Override sensor health as autopilot is in control
			//_system_status.sensors.pwm_control.count = get_param_uint(_system_status.sensors.pwm_control.param_stream_count);
			//safety_update_sensor(&_system_status.sensors.pwm_control);

			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
			snprintf(text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "[MIXER] Testing motor: %d", _motor_test.motor_step);
			mavlink_queue_broadcast_notice(text);

			command_result = MAV_RESULT_ACCEPTED;
		} else {
			mavlink_queue_broadcast_error("[MIXER] Cannot run motor test, bad test variables!");

			command_result = MAV_RESULT_DENIED;
		}
	}

	return command_result;
}

#ifdef __cplusplus
}
#endif
