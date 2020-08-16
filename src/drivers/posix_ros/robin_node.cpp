#include <ros/ros.h>

extern "C" {

#include "run.h"
#include "drivers/posix_common/drv_cmd_args.h"
#include "drivers/posix_common/runtime.h"

static bool _soft_reset;

void posix_soft_reset( void ) {
	_soft_reset = true;
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

}
