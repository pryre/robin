#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "controller.h"
#include "safety.h"

#include "fix16.h"
#include "fixextra.h"

command_input_t _cmd_ob_input;

void mavlink_handle_set_attitude_target( mavlink_channel_t chan, mavlink_message_t *msg, mavlink_status_t *status ) {
	if( (mavlink_msg_set_attitude_target_get_target_system(msg) == mavlink_system.sysid) &&
		(mavlink_msg_set_attitude_target_get_target_component(msg) == mavlink_system.compid) ) {

		//TODO: Check timestamp was recent before accepting

		//Input Mask
		_cmd_ob_input.input_mask = mavlink_msg_set_attitude_target_get_type_mask(msg);

		//Rates
		_cmd_ob_input.r = fix16_from_float(mavlink_msg_set_attitude_target_get_body_roll_rate(msg));
		_cmd_ob_input.p = fix16_from_float(mavlink_msg_set_attitude_target_get_body_pitch_rate(msg));
		_cmd_ob_input.y = fix16_from_float(mavlink_msg_set_attitude_target_get_body_yaw_rate(msg));

		//Attitude
		float qt_float[4];
		qf16 qt_fix;
		mavlink_msg_set_attitude_target_get_q(msg, &qt_float[0]);

		qt_fix.a = fix16_from_float(qt_float[0]);
		qt_fix.b = fix16_from_float(qt_float[1]);
		qt_fix.c = fix16_from_float(qt_float[2]);
		qt_fix.d = fix16_from_float(qt_float[3]);

		qf16_normalize_to_unit(&_cmd_ob_input.q, &qt_fix);

		//Trottle
		_cmd_ob_input.T = fix16_from_float(mavlink_msg_set_attitude_target_get_thrust(msg));

		//Update Sensor
		safety_update_sensor(&_system_status.sensors.offboard_control);
	}
}

#ifdef __cplusplus
}
#endif
