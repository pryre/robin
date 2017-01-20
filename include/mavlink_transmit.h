#pragma once

#include "mavlink_system.h"

//This will contain functions to build and transmit mavlink messages
//as well as a listing of utility functions to send each supported message type

#include <stdint.h>


// typedefs
typedef struct {
	uint32_t period_us;
	uint32_t last_time_us;
	void (*send_function)(void);
} mavlink_stream_t;

// type definitions
typedef enum {
	MAVLINK_STREAM_ID_HEARTBEAT,
	MAVLINK_STREAM_ID_SYS_STATUS,
	MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

// function declarations
void communication_transmit(uint32_t time_us);
//void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate);
//void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us);
//void mavlink_send_heartbeat(void);

