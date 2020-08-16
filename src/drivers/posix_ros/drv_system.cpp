#include <ros/ros.h>

#include <stdint.h>
#include <stdbool.h>

#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>

#include <uuid/uuid.h>

#include "drivers/drv_system.h"

static uint16_t vid;
static uint16_t pid;
static uint64_t uid;

void system_init(void) {
	system_debug_print((char *)"--== robin ==--");

	//Generate a Vendor ID and Product ID based off of the ethernet MAC address
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock >= 0) {
		struct ifreq ifr;
		struct ifconf ifc;
		char buf[1024];
		bool success = false;

		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = buf;
		if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { /* handle error */ }

		struct ifreq* it = ifc.ifc_req;
		const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

		for (; it != end; ++it) {
			strcpy(ifr.ifr_name, it->ifr_name);
			if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
				if ( !(ifr.ifr_flags & IFF_LOOPBACK) ) {
					if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
						success = true;
						break;
					}
				}
			}
		}

		if (success) {
			vid = (ifr.ifr_hwaddr.sa_data[2] << 8) + (ifr.ifr_hwaddr.sa_data[3]);
			pid = (ifr.ifr_hwaddr.sa_data[4] << 8) + (ifr.ifr_hwaddr.sa_data[5]);
		} else {
			vid = 0;
			pid = 0;
		}

		close(sock);
	}

	//Generate a Unique ID
	time_t t;
	srand((unsigned) time(&t));
	uid = ((uint64_t)rand() << 32) + (uint64_t)rand();
}

uint32_t system_micros(void) {
	ros::Time currentTime = ros::Time::now();
	return (currentTime.sec * (int)1e6) + (currentTime.nsec / 1000);
}

void system_pause_ms(uint32_t ms) {
	ros::Duration sleep_time;
	sleep_time.sec = 0;
	sleep_time.nsec = 1000000 * ms;	//millis to nanos
	sleep_time.sleep();
}

void system_reset(void) {
	posix_soft_reset();
}

void system_bootloader(void) {
	exit(0);
}

uint16_t system_vendor_id(void) {
	return vid;
}

uint16_t system_product_id(void) {
	return pid;
}

uint64_t system_unique_id(void) {
	return uid;
}

int system_debug_print( const char *format, ... ) {
	char msg[1000];
	int result;
    va_list args;

    va_start(args, format);
    result = vsnprintf(msg, 1000, format, args);
    va_end(args);

	ROS_INFO( "%s", msg );

    return result;
}

void system_debug_print( char *msg ) {

}

void system_rate_limit(void) {
	ros::spinOnce();

	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = (int)1e5;	//sleep for 0.1 millis

	nanosleep(&sleep_time, NULL);
}
