#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PROFILER_IDEAL_MAX_LOOP 1000	//Ideal loop rate in us

//XXX: Hardcode the first 3 Enum values so we know where to start for debug feedback
#define PROFILER_ID_LOOP_ENUM 0
#define PROFILER_ID_SETUP_ENUM 1
#define PROFILER_ID_FIRST_ENUM 2

typedef enum {
	PROFILER_ID_LOOP = PROFILER_ID_LOOP_ENUM,
	PROFILER_ID_SETUP = PROFILER_ID_SETUP_ENUM,
	PROFILER_ID_SENSORS = PROFILER_ID_FIRST_ENUM,
	PROFILER_ID_ESTIMATOR,
	PROFILER_ID_COMMS_RX,
	PROFILER_ID_COMMS_TX,
	PROFILER_ID_SAFETY,
	PROFILER_ID_STATUS,
	PROFILER_ID_CONTROL,
	PROFILER_ID_MIXER,
	PROFILER_ID_NUM
} profiler_ids_t;

typedef struct {
	uint32_t start; // Loop start time
	//uint32_t end;   // Loop end time

	uint32_t counter;	// Times the data from this sensor has been collated
	uint32_t dt_int;	// Sum of dt
	uint32_t max;		// Maximum dt so far
	uint32_t min;		// Minimum dt so far
} profiler_profile_t;

void profiler_init( void );
void profiler_run( profiler_ids_t id, void (*fun_ptr)(uint32_t) );

uint32_t profiler_get_loop_start( void );
//uint32_t profiler_get_loop_stats( uint32_t *min, uint32_t *mean, uint32_t *max );	//Get the stats of the last loop reading

void profiler_set_start( profiler_ids_t id, uint32_t time_us );
void profiler_set_end( profiler_ids_t id, uint32_t time_us );

//void profiler_get_start( profiler_clock_ids_t id );
//void profiler_get_end( profiler_clock_ids_t id );
//void profiler_get_min( profiler_clock_ids_t id );
//void profiler_get_mean( profiler_clock_ids_t id );
//void profiler_get_max( profiler_clock_ids_t id );
bool profiler_read( profiler_ids_t id, uint32_t *min, uint32_t *mean, uint32_t *max );
