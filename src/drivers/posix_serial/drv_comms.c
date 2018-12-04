#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include "params.h"
#include "mavlink_system.h"

static int fd_comm_0;
static int fd_comm_1;

static int set_interface_attribs(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
		char err[100];
        snprintf(err, 100, "[COMM] Error from tcgetattr: %s", strerror(errno));
		system_debug_print(err);
        return -1;
    }

	cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

	cfmakeraw(&tty);

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		char err[100];
        snprintf(err, 100, "[COMM] Error from tcsetattr: %s", strerror(errno));
		system_debug_print(err);
        return -1;
    }
    return 0;
}

/*
static int set_interface_attribs(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
		char err[100];
        snprintf(err, 100, "[COMM] Error from tcgetattr: %s", strerror(errno));
		system_debug_print(err);
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         / 8-bit characters
    tty.c_cflag &= ~PARENB;     // no parity bit
    tty.c_cflag &= ~CSTOPB;     // only need 1 stop bit
    tty.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol

    // setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		char err[100];
        snprintf(err, 100, "[COMM] Error from tcsetattr: %s", strerror(errno));
		system_debug_print(err);
        return -1;
    }
    return 0;
}

static void set_mincount(int fd, int mcount) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
		char err[100];
        snprintf(err, 100, "Error tcgetattr: %s\n", strerror(errno));
		system_debug_print(err);
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        // half second timer

    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
		char err[100];
        snprintf(err, 100, "Error tcsetattr: %s\n", strerror(errno));
		system_debug_print(err);
	}
}
*/

int init_serial_port( char *portname, uint32_t buadrate ) {
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
		char err[100];
        snprintf(err, 100, "[COMM] Error opening %s: %s", portname, strerror(errno));
		system_debug_print(err);
        return -1;
    }

	// [baudrate], 8 bits, no parity, 1 stop bit
	switch(buadrate) {
		case 9600: {
		    set_interface_attribs(fd, B9600);
			break;
		}
		case 57600: {
		    set_interface_attribs(fd, B57600);
			break;
		}
		case 115200: {
		    set_interface_attribs(fd, B115200);
			break;
		}
		case 921600: {
		    set_interface_attribs(fd, B921600);
			break;
		}

		default: {
			return -1;
		}
	}

    //set_mincount(fd, 0);                /* set to pure timed read */

	return fd;
}

bool comms_init_port( comms_port_t port ) {
	bool success = false;
	char portname[50];

	switch(port) {
		case COMM_PORT_0: {
			strncpy(portname, "/dev/ttyROBIN0", 50);
			fd_comm_0 = init_serial_port( portname, get_param_uint(PARAM_BAUD_RATE_0) );
			success = (fd_comm_0 > 0);

			break;
		}
		case COMM_PORT_1: {
			strncpy(portname, "/dev/ttyROBIN0", 50);
			fd_comm_1 = init_serial_port( portname, get_param_uint(PARAM_BAUD_RATE_1) );
			success = (fd_comm_1 > 0);

			break;
		}
	}

	if(success) {
		comms_set_open( port );

		char info[100];
        snprintf(info, 100, "[COMM] Openned comm port %s", portname);
		system_debug_print(info);
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
	uint8_t buf[2] = {ch,0};

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				//TODO: Could check output was written (wlen)
				write(fd_comm_0, buf, 1);
				tcdrain(fd_comm_0);
				break;
			}
			case COMM_PORT_1: {
				write(fd_comm_1, buf, 1);
				tcdrain(fd_comm_1);
				break;
			}
		}
	}
}

void comms_send_datagram( comms_port_t port, uint8_t* datagram, uint32_t length) {
	if(length <= MAVLINK_MAX_PACKET_LEN)
		for(uint32_t i=0; i<length; i++)
			comms_send(port, datagram[i]);
}

bool comms_waiting( comms_port_t port ) {
	int bytes_available = 0;

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				ioctl(fd_comm_0, FIONREAD, &bytes_available);
				break;
			}
			case COMM_PORT_1: {
				ioctl(fd_comm_1, FIONREAD, &bytes_available);
				break;
			}
		}
	}

	return (bytes_available > 0);
}

uint8_t comms_recv( comms_port_t port ) {
	/* simple noncanonical input
    do {
        unsigned char buf[80];
        int rdlen;

        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            buf[rdlen] = 0;
            printf("Read %d: \"%s\"\n", rdlen, buf);
        } else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
        // repeat read to get full message
    } while (1);
	*/

	uint8_t ch = 0;
	uint8_t buf[2];
	int rdlen = 0;

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				rdlen = read(fd_comm_0, buf, 1);
				break;
			}
			case COMM_PORT_1: {
				rdlen = read(fd_comm_1, buf, 1);
				break;
			}
		}
	}


	if(rdlen == 1)
		ch = buf[0];

	return ch;
}


