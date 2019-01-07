#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_receive.h"
#include "mavlink_system.h"

#include "params.h"

void mavlink_handle_param_set( mavlink_channel_t chan, mavlink_message_t* msg,
							   mavlink_status_t* status ) {
	if ( ( mavlink_msg_param_set_get_target_system( msg ) == mavlink_system.sysid ) && ( mavlink_msg_param_set_get_target_component( msg ) == mavlink_system.compid ) ) {

		char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
		mavlink_msg_param_set_get_param_id( msg, param_id );
		param_id_t index = lookup_param_id( param_id );

		if ( index < PARAMS_COUNT ) { // If the ID is valid
			MAV_PARAM_TYPE send_type = mavlink_msg_param_set_get_param_type( msg );
			MAV_PARAM_TYPE known_type = get_param_type( index );

			bool right_type = ( known_type == send_type );
			if ( ( !right_type ) && get_param_uint( PARAM_RELAXED_PARAM_SET ) ) {
				right_type = true;
				mavlink_queue_broadcast_notice(
					"[PARAM] Using relaxed parameter saving" );
			}

			if ( right_type ) {
				bool set_failed = false;

				union {
					float f;
					int32_t i;
					uint32_t u;
				} u;

				u.f = mavlink_msg_param_set_get_param_value( msg );

				switch ( send_type ) {
				case MAV_PARAM_TYPE_UINT32: {
					uint32_t val = 0;

					if ( known_type == MAV_PARAM_TYPE_UINT32 ) {
						val = u.u;
					} else if ( known_type == MAV_PARAM_TYPE_INT32 ) {
						val = (uint32_t)u.i;
					} else if ( known_type == MAV_PARAM_TYPE_REAL32 ) {
						val = (uint32_t)u.f;
					} else { // Must be real
						set_failed = true;
					}

					if ( !set_failed )
						set_param_by_name_uint( param_id, val );

					break;
				}
				case MAV_PARAM_TYPE_INT32: {
					int32_t val = 0;

					if ( known_type == MAV_PARAM_TYPE_UINT32 ) {
						val = u.u;
					} else if ( known_type == MAV_PARAM_TYPE_INT32 ) {
						val = u.i;
					} else if ( known_type == MAV_PARAM_TYPE_REAL32 ) {
						val = u.f;
					} else { // Must be real
						set_failed = true;
					}

					if ( !set_failed )
						set_param_by_name_int( param_id, val );

					break;
				}
				case MAV_PARAM_TYPE_REAL32: {
					float val = 0;

					if ( known_type == MAV_PARAM_TYPE_UINT32 ) {
						val = u.u;
					} else if ( known_type == MAV_PARAM_TYPE_INT32 ) {
						val = u.i;
					} else if ( known_type == MAV_PARAM_TYPE_REAL32 ) {
						val = u.f;
					} else { // Must be real
						set_failed = true;
					}

					if ( !set_failed )
						set_param_by_name_fix16( param_id, fix16_from_float( val ) );

					break;
				}
				default: {
					mavlink_queue_broadcast_notice(
						"[PARAM] Do not know how to handle sent paramater type!" );

					break;
				}
				}

				if ( set_failed )
					mavlink_queue_broadcast_error(
						"[PARAM] Failed to determine known parameter type!" );
			} else {
				// XXX: This may be caused if a GCS sends a UINT32 param as INT32
				mavlink_queue_broadcast_error( "[PARAM] Paramater type mismatch!" );
			}
		}
	} // Else this message is for someone else
}

#ifdef __cplusplus
}
#endif
