#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "drivers/drv_system.h"
#include "fix16.h"
#include "fixquat.h"
#include "fixextra.h"
#include "safety.h"
#include "sensors.h"

//sensor_readings_t _sensors;

void mavlink_handle_att_pos_mocap( mavlink_channel_t chan,
								   mavlink_message_t* msg,
								   mavlink_status_t* status ) {
	_sensors.ext_pose.status.present = true;

	// TODO: Check timestamp was recent before accepting
	_sensors.ext_pose.status.time_read = system_micros();

	// Position
	_sensors.ext_pose.p.x = fix16_from_float( mavlink_msg_att_pos_mocap_get_x( msg ) );
	_sensors.ext_pose.p.y = fix16_from_float( mavlink_msg_att_pos_mocap_get_y( msg ) );
	_sensors.ext_pose.p.z = fix16_from_float( mavlink_msg_att_pos_mocap_get_z( msg ) );

	// Attitude
	float qt_float[4];
	qf16 qt_fix;
	mavlink_msg_att_pos_mocap_get_q( msg, &qt_float[0] );

	qt_fix.a = fix16_from_float( qt_float[0] );
	qt_fix.b = fix16_from_float( qt_float[1] );
	qt_fix.c = fix16_from_float( qt_float[2] );
	qt_fix.d = fix16_from_float( qt_float[3] );

	qf16_normalize_to_unit( &_sensors.ext_pose.q, &qt_fix );

	// Update Sensor
	safety_update_sensor( &_system_status.sensors.ext_pose );
	_sensors.ext_pose.status.new_data = true;
}

#ifdef __cplusplus
}
#endif
