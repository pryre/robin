#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include "mavlink_system.h"
#include "params.h"

static int fd_comm_0;
static int fd_comm_1;

static int set_interface_attribs( int fd, int speed ) {
	struct termios tty;

	if ( tcgetattr( fd, &tty ) < 0 ) {
		system_debug_print( "[COMM] Error from tcgetattr: %s", strerror( errno ) );
		return -1;
	}

	cfsetospeed( &tty, (speed_t)speed );
	cfsetispeed( &tty, (speed_t)speed );

	cfmakeraw( &tty );

	if ( tcsetattr( fd, TCSANOW, &tty ) != 0 ) {
		system_debug_print( "[COMM] Error from tcsetattr: %s", strerror( errno ) );
		return -1;
	}
	return 0;
}

int init_serial_port( char* portname, uint32_t buadrate ) {
	int fd;

	fd = open( portname, O_RDWR | O_NOCTTY | O_SYNC );
	if ( fd < 0 ) {
		system_debug_print( "[COMM] Error opening %s: %s", portname,
							strerror( errno ) );
		return -1;
	}

	// [baudrate], 8 bits, no parity, 1 stop bit
	switch ( buadrate ) {
	case 9600: {
		set_interface_attribs( fd, B9600 );
		break;
	}
	case 57600: {
		set_interface_attribs( fd, B57600 );
		break;
	}
	case 115200: {
		set_interface_attribs( fd, B115200 );
		break;
	}
	case 921600: {
		set_interface_attribs( fd, B921600 );
		break;
	}

	default: { return -1; }
	}

	// set_mincount(fd, 0);                /* set to pure timed read */

	return fd;
}

bool comms_init_port( comms_port_t port ) {
	bool success = false;
	char portname[50];

	switch ( port ) {
	case COMM_PORT_0: {
		strncpy( portname, "/dev/ttyROBIN0", 50 );
		fd_comm_0 = init_serial_port( portname, get_param_uint( PARAM_BAUD_RATE_0 ) );
		success = ( fd_comm_0 > 0 );

		break;
	}
	case COMM_PORT_1: {
		strncpy( portname, "/dev/ttyROBIN0", 50 );
		fd_comm_1 = init_serial_port( portname, get_param_uint( PARAM_BAUD_RATE_1 ) );
		success = ( fd_comm_1 > 0 );

		break;
	}
	}

	if ( success ) {
		comms_set_open( port );

		system_debug_print( "[COMM] Openned comm port %s", portname );
	}

	return success;
}

static bool clean_close_serial( int fd_comm ) {
	bool success = true;	//Return true by default if it was already closed

	if ( fd_comm > 0 ) {
		success = (close(fd_comm) == 0);
	}

	return success;
}

bool comms_deinit_port( comms_port_t port ) {
	bool success = false;

	switch ( port ) {
	case COMM_PORT_0: {
		success = clean_close_serial(fd_comm_0);

		if(success)
			fd_comm_0 = 0;

		break;
	}
	case COMM_PORT_1: {
		success = clean_close_serial(fd_comm_1);

		if(success)
			fd_comm_1 = 0;

		break;
	}
	}

	if ( success ) {
		comms_set_closed( port );
		system_debug_print( "[COMMS] Closed comm port: 0x%x", port );
	} else {
		system_debug_print( "[COMMS] Failed to close comm port 0x%x!", port );
	}

	return success;
}
static void comms_send( comms_port_t port, uint8_t ch ) {
	/*
// simple output
wlen = write(fd, "Hello!\n", 7);
if (wlen != 7) {
printf("Error from write: %d, %d\n", wlen, errno);
}
tcdrain(fd);    // delay for output
*/
	uint8_t buf[2] = {ch, 0};

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			// TODO: Could check output was written (wlen)
			write( fd_comm_0, buf, 1 );
			tcdrain( fd_comm_0 );
			break;
		}
		case COMM_PORT_1: {
			write( fd_comm_1, buf, 1 );
			tcdrain( fd_comm_1 );
			break;
		}
		}
	}
}

void comms_send_datagram( comms_port_t port, uint8_t* datagram,
						  uint32_t length ) {
	if ( length <= MAVLINK_MAX_PACKET_LEN )
		for ( uint32_t i = 0; i < length; i++ )
			comms_send( port, datagram[i] );
}

bool comms_waiting( comms_port_t port ) {
	int bytes_available = 0;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			ioctl( fd_comm_0, FIONREAD, &bytes_available );
			break;
		}
		case COMM_PORT_1: {
			ioctl( fd_comm_1, FIONREAD, &bytes_available );
			break;
		}
		}
	}

	return ( bytes_available > 0 );
}

uint8_t comms_recv( comms_port_t port ) {
	uint8_t ch = 0;
	uint8_t buf[2];
	int rdlen = 0;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			rdlen = read( fd_comm_0, buf, 1 );
			break;
		}
		case COMM_PORT_1: {
			rdlen = read( fd_comm_1, buf, 1 );
			break;
		}
		}
	}

	if ( rdlen == 1 )
		ch = buf[0];

	return ch;
}
