#include <ros/ros.h>

extern "C" {

#include "run.h"
#include "drivers/posix_common/drv_cmd_args.h"
#include "drivers/posix_common/runtime.h"

static bool _soft_reset;

void posix_soft_reset( void ) {
	_soft_reset = true;
}

//XXX: Not needed for this system
void posix_get_sim_time( uint32_t *secs, uint32_t *nsecs ) {
	*secs = 0;
	*nsecs = 0;
}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robin");
	ros::NodeHandle nh;

	parse_arguments(argc, argv);
	_soft_reset = false;

	while( ros::ok() ) {
		setup();

		while ( ros::ok() ) {
			loop();

			if(_soft_reset) {
				_soft_reset = false;
				break;
			}
		}
	}

	return 0;
}
