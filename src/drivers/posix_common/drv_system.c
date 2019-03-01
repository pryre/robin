#include <stdbool.h>
#include <stdint.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include <net/if.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <uuid/uuid.h>

#include "drivers/drv_system.h"

static uint16_t vid;
static uint16_t pid;
static uint64_t uid;

void system_init( void ) {
	system_debug_print( "--== robin ==--" );

	// Generate a Vendor ID and Product ID based off of the ethernet MAC address
	int sock = socket( AF_INET, SOCK_DGRAM, IPPROTO_IP );
	if ( sock >= 0 ) {
		struct ifreq ifr;
		struct ifconf ifc;
		char buf[1024];
		bool success = false;

		ifc.ifc_len = sizeof( buf );
		ifc.ifc_buf = buf;
		if ( ioctl( sock, SIOCGIFCONF, &ifc ) == -1 ) { /* handle error */
		}

		struct ifreq* it = ifc.ifc_req;
		const struct ifreq* const end = it + ( ifc.ifc_len / sizeof( struct ifreq ) );

		for ( ; it != end; ++it ) {
			strcpy( ifr.ifr_name, it->ifr_name );
			if ( ioctl( sock, SIOCGIFFLAGS, &ifr ) == 0 ) {
				if ( !( ifr.ifr_flags & IFF_LOOPBACK ) ) {
					if ( ioctl( sock, SIOCGIFHWADDR, &ifr ) == 0 ) {
						success = true;
						break;
					}
				}
			}
		}

		if ( success ) {
			vid = ( ifr.ifr_hwaddr.sa_data[2] << 8 ) + ( ifr.ifr_hwaddr.sa_data[3] );
			pid = ( ifr.ifr_hwaddr.sa_data[4] << 8 ) + ( ifr.ifr_hwaddr.sa_data[5] );
		} else {
			vid = 0;
			pid = 0;
		}

		close( sock );
	}

	// Generate a Unique ID
	time_t t;
	srand( (unsigned)time( &t ) );
	uid = ( (uint64_t)rand() << 32 ) + (uint64_t)rand();
}

uint32_t system_micros( void ) {
	struct timeval currentTime;
	gettimeofday( &currentTime, NULL );
	return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}

void system_pause_us( uint32_t us ) {
	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 1000 * us; // micros to nanos

	nanosleep( &sleep_time, NULL );
}

void system_pause_ms( uint32_t ms ) {
	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 1000000 * ms; // millis to nanos

	nanosleep( &sleep_time, NULL );
}

void system_reset( void ) {
	exit( 0 );
}

void system_bootloader( void ) {
	exit( 0 );
}

uint16_t system_vendor_id( void ) {
	return vid;
}

uint16_t system_product_id( void ) {
	return pid;
}

uint64_t system_unique_id( void ) {
	return uid;
}

int system_debug_print( const char* format, ... ) {
	int result;
	va_list args;

	va_start( args, format );
	result = vprintf( format, args );
	va_end( args );
	printf( "\n" );

	return result;
}

void system_rate_limit( void ) {
	struct timespec sleep_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = (int)1e5; // sleep for 0.1 millis

	nanosleep( &sleep_time, NULL );
}
