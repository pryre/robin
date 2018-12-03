#include <ros/ros.h>

extern "C" {
	#include "run.h"
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robin");
	ros::NodeHandle nh;

	setup();

	while( ros::ok() ) {
		loop();
	}

	return 0;
}
