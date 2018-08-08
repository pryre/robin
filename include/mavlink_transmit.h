#pragma once

#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"
#include "params.h"

#include <stdint.h>


// typedefs
typedef struct {
	param_id_t param_rate;
	uint32_t period_update;
	uint32_t last_time_us;
	void (*send_function)(mavlink_channel_t port);
} mavlink_stream_t;

// type definitions
typedef enum {
	MAVLINK_STREAM_ID_HEARTBEAT,
	MAVLINK_STREAM_ID_SYS_STATUS,
	MAVLINK_STREAM_ID_HIGHRES_IMU,
	MAVLINK_STREAM_ID_ATTITUDE,
	MAVLINK_STREAM_ID_ATTITUDE_QUATERNION,
	MAVLINK_STREAM_ID_ATTITUDE_TARGET,
	MAVLINK_STREAM_ID_RC_CHANNELS_RAW,
	MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW,
	MAVLINK_STREAM_ID_TIMESYNC,
	MAVLINK_STREAM_ID_BATTERY_STATUS,
	MAVLINK_STREAM_ID_LOW_PRIORITY,
	MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

// function declarations
void communication_transmit(uint32_t time_us);
void communication_calc_period_update(uint8_t comm_port, mavlink_stream_id_t stream_id);

