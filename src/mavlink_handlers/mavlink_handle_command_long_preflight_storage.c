#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "safety.h"
#include "params.h"

system_status_t _system_status;

MAV_RESULT mavlink_handle_command_long_preflight_storage( mavlink_channel_t chan, float *params ) {
	MAV_RESULT command_result = MAV_RESULT_ENUM_END;

	if( _system_status.state == MAV_STATE_STANDBY) {
		switch( (int)params[0] ) {
			case 0:		//Read from flash
				if( read_params() ) {
					command_result = MAV_RESULT_ACCEPTED;
				} else {
					command_result = MAV_RESULT_FAILED;
				}

				break;
			case 1:	//Write to flash
				if( write_params() ) {
					command_result = MAV_RESULT_ACCEPTED;
				} else {
					command_result = MAV_RESULT_FAILED;
				}

				break;
			case 2:		//Reset to defaults
				set_param_defaults();
				command_result = MAV_RESULT_ACCEPTED;

				break;
			default:	//Not supported
				command_result = MAV_RESULT_UNSUPPORTED;
				break;
		}
	} else {
		command_result = MAV_RESULT_DENIED;
	}

	return command_result;
}

#ifdef __cplusplus
}
#endif
