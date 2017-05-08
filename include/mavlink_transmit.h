#pragma once

#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"

//This will contain functions to build and transmit mavlink messages
//as well as a listing of utility functions to send each supported message type

#include <stdint.h>


// typedefs
typedef struct {
	uint32_t period_param;
	uint32_t last_time_us;
	void (*send_function)(uint8_t port);
} mavlink_stream_t;

// type definitions
typedef enum {
	MAVLINK_STREAM_ID_HEARTBEAT,
	MAVLINK_STREAM_ID_SYS_STATUS,
	MAVLINK_STREAM_ID_HIGHRES_IMU,
	MAVLINK_STREAM_ID_ATTITUDE,
	MAVLINK_STREAM_ID_ATTITUDE_QUATERNION,
	MAVLINK_STREAM_ID_ATTITUDE_TARGET,
	MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW,
	MAVLINK_STREAM_ID_TIMESYNC,
	MAVLINK_STREAM_ID_LOW_PRIORITY,
	MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

//extern bool stream_comm_0[MAVLINK_STREAM_COUNT];
//extern bool stream_comm_1[MAVLINK_STREAM_COUNT];

// function declarations
void communication_transmit(uint32_t time_us);

