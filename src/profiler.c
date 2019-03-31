#include "profiler.h"

#include "mavlink_system.h"
#include "params.h"

#include "drivers/drv_system.h"
#include "fix16.h"
#include "fixextra.h"

static uint32_t debug_period_;
static uint32_t debug_time_last_;
static profiler_profile_t profiles_[PROFILER_ID_NUM];
static const char profile_names_[PROFILER_ID_NUM][10] = {
	"LOOP",
	"SETUP",
	"SENSORS",
	"ESTIMATOR",
	"COMMS_RX",
	"COMMS_TX",
	"SAFETY",
	"STATUS",
	"CONTROL",
	"MIXER"
};

static void profiler_reset( profiler_ids_t id ) {
	if(id < PROFILER_ID_NUM) {
		profiles_[id].start = 0;

		profiles_[id].dt_int = 0;
		profiles_[id].counter = 0;
		profiles_[id].max = 0;
		profiles_[id].min = 0xFFFF;
	}
}

void profiler_init( void ) {
	debug_period_ = 0;
	debug_time_last_ = 0;

	for(int i=0; i<PROFILER_ID_NUM; i++) {
		profiler_reset( i );
	}
}

void profiler_run( profiler_ids_t id, void (*fun_ptr)(uint32_t) ) {
	if( debug_period_ )
		profiler_set_start( id, system_micros() );

	(*fun_ptr)( system_micros() );

	if( debug_period_ )
		profiler_set_end( id, system_micros() );
}

uint32_t profiler_get_loop_start( void ) {
	return profiles_[PROFILER_ID_LOOP].start;
}

void profiler_set_start( profiler_ids_t id, uint32_t time_us ) {
	if( id < PROFILER_ID_NUM ) {
		profiles_[id].start = time_us;

		//On loop start, we load in our debug profiler setting
		if ( id == PROFILER_ID_LOOP ) {
			fix16_t rate = get_param_fix16(PARAM_DEBUG_PROFILING);

			if(rate) {

				debug_period_ = 1000 * fix16_to_int( fix16_div( _fc_1000, rate ) );
			} else {
				debug_period_ = 0;
			}
		}
	}
}

bool profiler_read( profiler_ids_t id, uint32_t *min, uint32_t *mean, uint32_t *max ) {
	bool success = false;

	if( id < PROFILER_ID_NUM ) {
		if( profiles_[id].counter ) {
			*min = profiles_[id].min;
			*max = profiles_[id].max;
			*mean = profiles_[id].dt_int / profiles_[id].counter;

			//Clear the profile and give an OK
			profiler_reset(id);
			success = true;
		}
	}

	return success;
}

static void profiler_send_debug( profiler_ids_t id ) {
	if( id < PROFILER_ID_NUM ) {
		uint32_t min = 0;
		uint32_t mean = 0;
		uint32_t max = 0;

		if( profiler_read( id, &min, &mean, &max ) ) {
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
			snprintf( text,
					  MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
					  "[PRO] %s: [%lu;%lu,%lu]",
					  profile_names_[id],
					  (unsigned long)mean,
					  (unsigned long)min,
					  (unsigned long)max);
			mavlink_queue_broadcast_debug( text );
		}
	}
}

void profiler_set_end( profiler_ids_t id, uint32_t time_us ) {
	if( id < PROFILER_ID_NUM ) {
		//profiles_[id].end = time_us;

		//if( profiles_[id].end > profiles_[id].start ) {
		if( ( profiles_[id].start ) && ( time_us > profiles_[id].start ) ) {
			//uint32_t dt = profiles_[id].end - profiles_[id].start;
			uint32_t dt = time_us - profiles_[id].start;

			profiles_[id].dt_int += dt;
			profiles_[id].counter++;

			if( dt > profiles_[id].max )
				profiles_[id].max = dt;

			if( dt < profiles_[id].min )
				profiles_[id].min = dt;
		}

		profiles_[id].start = 0;

		//On loop end, we handle the debug profiling readout if needed
		if ( debug_period_ && ( id == PROFILER_ID_LOOP ) ) {
			if( ( time_us - debug_time_last_ ) > debug_period_ ) {
				for(int i=PROFILER_ID_FIRST_ENUM; i<PROFILER_ID_NUM; i++)
					profiler_send_debug(i);

				debug_time_last_ = time_us;
			}
		} else if ( id == PROFILER_ID_SETUP ) {
			//Special handler for the setup feedback
			//This should only be sent on boot once
			if( get_param_fix16(PARAM_DEBUG_PROFILING) )
				profiler_send_debug(id);
		}
	}
}

