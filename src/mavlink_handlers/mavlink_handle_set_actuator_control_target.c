#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"
#include "mixer.h"

static bool got_actuator_g4;
static bool got_actuator_g5;
fix16_t _actuator_control_g0[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g1[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g2[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g3[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g4[MIXER_NUM_MOTORS];
fix16_t _actuator_control_g5[MIXER_NUM_MOTORS];

void mavlink_handle_set_actuator_control_target( mavlink_channel_t chan, mavlink_message_t *msg, mavlink_status_t *status ) {
	if( (mavlink_msg_set_actuator_control_target_get_target_system(msg) == mavlink_system.sysid) &&
		(mavlink_msg_set_actuator_control_target_get_target_component(msg) == mavlink_system.compid) ) {

		float act_float[8];
		mavlink_msg_set_actuator_control_target_get_controls(msg, &act_float[0]);

		if( mavlink_msg_set_actuator_control_target_get_group_mlx(msg) == 0) {
			for(int i=0; i<8; i++)
				_actuator_control_g0[i] = fix16_from_float(act_float[i]);

			safety_update_sensor(&_system_status.sensors.offboard_mixer_g0_control);
		} else if( mavlink_msg_set_actuator_control_target_get_group_mlx(msg) == 1) {
			for(int i=0; i<8; i++)
				_actuator_control_g1[i] = fix16_from_float(act_float[i]);

			safety_update_sensor(&_system_status.sensors.offboard_mixer_g1_control);
		} else if( mavlink_msg_set_actuator_control_target_get_group_mlx(msg) == 4) {
			if(!got_actuator_g4) {
				mavlink_queue_broadcast_info("[MIXER] Accepted OB AUX PWM actuator control");
				got_actuator_g4 = true;
			}

			for(int i=0; i<8; i++)
				_actuator_control_g4[i] = fix16_from_float(act_float[i]);
		} else if( mavlink_msg_set_actuator_control_target_get_group_mlx(msg) == 5) {
			if(!got_actuator_g5) {
				mavlink_queue_broadcast_info("[MIXER] Accepted OB AUX digital actuator control");
				got_actuator_g5 = true;
			}

			for(int i=0; i<8; i++)
				_actuator_control_g5[i] = fix16_from_float(act_float[i]);
		}
	}
}

#ifdef __cplusplus
}
#endif
