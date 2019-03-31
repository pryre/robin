#include "profiler.h"
#include "drivers/drv_system.h"


static profiler_profile_t profiles_[PROFILER_ID_NUM];

static void profiler_reset( profiler_ids_t id ) {
	if(id < PROFILER_ID_NUM) {
		profiles_[id].start = 0;
		profiles_[id].end = 0;

		profiles_[id].counter = 0;
		profiles_[id].max = 0;
		profiles_[id].min = 0xFFFF;
	}
}

void profiler_init( void ) {
	for(int i=0; i<PROFILER_ID_NUM; i++) {
		profiler_reset( i );
	}
}

void profiler_run( profiler_ids_t id, void (*fun_ptr)(uint32_t) ) {
	//TODO: HERE!: Add parameter for profiler data collection (turn on/off)

	profiler_set_start( id, system_micros() );
	(*fun_ptr)( system_micros() );
	profiler_set_end( id, system_micros() );
}

/*
uint32_t profiler_get( profiler_clock_ids_t id ) {
	return _sensors.clock.start;
}
*/

uint32_t profiler_get_loop_start( void ) {
	return profiles_[PROFILER_ID_LOOP].start;
}

void profiler_set_start( profiler_ids_t id, uint32_t time_us ) {
	if( id < PROFILER_ID_NUM )
		profiles_[id].start = time_us;
}

void profiler_set_end( profiler_ids_t id, uint32_t time_us ) {
	if( id < PROFILER_ID_NUM ) {
		profiles_[id].end = time_us;

		if( profiles_[id].end > profiles_[id].start ) {
			uint32_t dt = profiles_[id].end - profiles_[id].start;

			profiles_[id].dt_int += dt;
			profiles_[id].counter++;

			if( dt > profiles_[id].max )
				profiles_[id].max = dt;

			if( dt < profiles_[id].min )
				profiles_[id].min = dt;
		}
	}
}

bool profiler_read( profiler_ids_t id, uint32_t *min, uint32_t *mean, uint32_t *max ) {
	bool success = false;

	if( id < PROFILER_ID_NUM ) {
		*min = profiles_[id].min;
		*max = profiles_[id].max;
		*mean = profiles_[id].dt_int / profiles_[id].counter;

		//Clear the profile and give an OK
		profiler_reset(id);
		success = true;
	}

	return success;
}

