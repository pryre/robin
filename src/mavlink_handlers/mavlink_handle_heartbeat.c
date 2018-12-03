#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"

bool _ch_0_have_heartbeat;
bool _ch_1_have_heartbeat;

system_status_t _system_status;

void mavlink_handle_heartbeat( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status ) {
	safety_update_sensor(&_system_status.sensors.offboard_heartbeat);

	//XXX: This won't operate each independently, which is probably not a good thing
	if(_system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK) {
		if(port == MAVLINK_COMM_0) {
			_ch_0_have_heartbeat = true;
		} else if(port == MAVLINK_COMM_1) {
			_ch_1_have_heartbeat = true;
		}
	} else {
		if(port == MAVLINK_COMM_0) {
			_ch_0_have_heartbeat = false;
		} else if(port == MAVLINK_COMM_1) {
			_ch_1_have_heartbeat = false;
		}
	}
}

#ifdef __cplusplus
}
#endif
