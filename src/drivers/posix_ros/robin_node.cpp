#include <ros/ros.h>

extern "C" {

#include "run.h"
#include "drivers/posix_common/drv_cmd_args.h"

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robin");
	ros::NodeHandle nh;

	parse_arguments(argc, argv);

	setup();

	while( ros::ok() ) {
		loop();
	}

	return 0;
}
