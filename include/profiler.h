#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PROFILER_IDEAL_LOOP_RATE 1000	//Ideal loop rate in us

typedef enum {
	PROFILER_ID_LOOP = 0,
	PROFILER_ID_SETUP,
	PROFILER_ID_SENSORS,
	PROFILER_ID_ESTIMATOR,
	PROFILER_ID_COMMS,
	PROFILER_ID_SAFETY,
	PROFILER_ID_STATUS,
	PROFILER_ID_CONTROL,
	PROFILER_ID_MIXER,
	PROFILER_ID_NUM
} profiler_ids_t;

typedef struct {
	uint32_t start; // Loop start time
	uint32_t end;   // Loop end time

	uint32_t counter;	// Times the data from this sensor has been collated
	uint32_t dt_int;	// Sum of dt
	uint32_t max;		// Maximum dt so far
	uint32_t min;		// Minimum dt so far
} profiler_profile_t;

void profiler_init( void );
void profiler_run( profiler_ids_t id, void (*fun_ptr)(uint32_t) );

uint32_t profiler_get_loop_start( void );

void profiler_set_start( profiler_ids_t id, uint32_t time_us );
void profiler_set_end( profiler_ids_t id, uint32_t time_us );

//void profiler_get_start( profiler_clock_ids_t id );
//void profiler_get_end( profiler_clock_ids_t id );
//void profiler_get_min( profiler_clock_ids_t id );
//void profiler_get_mean( profiler_clock_ids_t id );
//void profiler_get_max( profiler_clock_ids_t id );
bool profiler_read( profiler_ids_t id, uint32_t *min, uint32_t *mean, uint32_t *max );
