#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

void mavlink_handle_timesync( mavlink_channel_t chan, mavlink_message_t* msg,
							  mavlink_status_t* status ) {
	/*
//TODO: Get this working

mavlink_timesync_t tsync;
mavlink_msg_timesync_decode(&msg, &tsync);

uint32_t now_ms = micros();

//TODO: Should be in safety_check()?
if( (now_ms - _sensors.clock.rt_sync_last) > 500000) {	//There hasn't
been a sync in a while
    _sensors.clock.rt_offset_ns = 0;
    _sensors.clock.rt_drift = 1.0;
    _sensors.clock.rt_ts_last = 0;
    _sensors.clock.rt_tc_last = 0;
}

//Pulled from px4 firmware
uint64_t now_ns = now_ms * 1000LL;
uint64_t now_ns_corrected = now_ns * _sensors.clock.rt_drift;

int64_t time_offset_new = _sensors.clock.rt_offset_ns;

if (tsync.tc1 == 0) {
    mavlink_send_timesync(port, now_ns_corrected, tsync.ts1);
} else if (tsync.tc1 > 0) {
    if( (_sensors.clock.rt_ts_last != 0) && (_sensors.clock.rt_ts_last !=
0) ) {
            float drift = (float)(tsync.tc1 - _sensors.clock.rt_tc_last) /
(float)(tsync.ts1 - _sensors.clock.rt_ts_last);
            _sensors.clock.rt_drift =
sensors_clock_smooth_time_drift(_sensors.clock.rt_drift, drift);
    }

    _sensors.clock.rt_ts_last = tsync.ts1;
    _sensors.clock.rt_tc_last = tsync.tc1;

    int64_t offset_ns = (int64_t)(tsync.ts1 + now_ns_corrected - tsync.tc1
* 2) / 2;
    int64_t dt = _sensors.clock.rt_offset_ns - offset_ns;

    if ( abs(dt) > 10000000LL ) { // 10 millisecond skew
            time_offset_new = offset_ns;

            char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
            snprintf(text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
"[SENSOR] Hard setting skew: %0.9f", dt / 1e9);
            mavlink_queue_notice_broadcast( &text[0] );
    } else {
            //Filter the new time offset
            time_offset_new =
sensors_clock_smooth_time_skew(_sensors.clock.rt_offset_ns, offset_ns);
    }
}

_sensors.clock.rt_offset_ns = time_offset_new;
_sensors.clock.rt_sync_last = now_ms;
*/
}

#ifdef __cplusplus
}
#endif
