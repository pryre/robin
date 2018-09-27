#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>

#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include "params.h"
#include "mavlink_system.h"

#define BUFFER_LENGTH 2041

//#define BIND_HOST "127.0.0.1"
#define BIND_PORT 14551   //The port on which to send data
#define REMOTE_HOST "127.0.0.1"
#define REMOTE_PORT 14550   //The port on which to send data


static int sock_comm_0;
static struct sockaddr_in locAddr_comm_0;
static struct sockaddr_in gcAddr_comm_0;
static uint8_t buf_recv_comm_0[BUFFER_LENGTH];
static int read_recv_comm_0;

static int sock_comm_1;
static struct sockaddr_in locAddr_comm_1;
static struct sockaddr_in gcAddr_comm_1;
static uint8_t buf_recv_comm_0[BUFFER_LENGTH];
static int read_recv_comm_1;

/*
static void port_test() {
	char target_ip[100];

	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr;
	struct sockaddr_in locAddr;
	//struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	//int success = 0;
	unsigned int temp = 0;

	// Change the target ip if parameter was given
	strcpy(target_ip, "127.0.0.1");

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);

	// Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    }


	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }


	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);

	while(true) {
		// Send Heartbeat
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

		// Send Status
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));

		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg_in;
			mavlink_status_t status;

			printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg_in, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg_in.sysid, msg_in.compid, msg_in.len, msg_in.msgid);
				}
			}
			printf("\n");
		}
		memset(buf, 0, BUFFER_LENGTH);
		system_pause_ms(1000); // Sleep one second
    }
}
*/
static int init_udp_port( struct sockaddr_in *locAddr, struct sockaddr_in *gcAddr, uint32_t bind_port, char *remote_ip, uint32_t remote_port ) {
	// Change the target ip if parameter was given
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset( locAddr, 0, sizeof(*locAddr) );
	locAddr->sin_family = AF_INET;
	locAddr->sin_addr.s_addr = INADDR_ANY;
	locAddr->sin_port = htons(bind_port);

	// Bind the socket to port BIND_PORT
	if ( bind( sock, (struct sockaddr *)locAddr, sizeof(struct sockaddr) ) < 0 ) {
		char info[100];
        snprintf(info, 100, "[COMM] Error: port bind failed (%i)", bind_port);
		system_debug_print(info);

		close(sock);
		return false;
    }

	if(sock > 0 ) {
		if ( fcntl( sock, F_SETFL, O_NONBLOCK | O_ASYNC ) < 0 ) {
			char info[100];
			snprintf(info, 100, "[COMM] Error: error setting nonblocking (%s)", strerror(errno));
			system_debug_print(info);

			close(sock);
			return false;
		}
	}


	memset( gcAddr, 0, sizeof(*gcAddr) );
	gcAddr->sin_family = AF_INET;
	gcAddr->sin_addr.s_addr = inet_addr(remote_ip);
	gcAddr->sin_port = htons(remote_port);

	return sock;
}

bool comms_init_port( comms_port_t port ) {
	//port_test();

	bool success = false;

	uint32_t bind_port;
	char remote_host[100];
	uint32_t remote_port;

	switch(port) {
		case COMM_PORT_0: {
			//strncpy(portname, "/dev/ttyROBIN0", 50);
			read_recv_comm_0 = - 1;
			bind_port = BIND_PORT;
			strncpy(remote_host, REMOTE_HOST, 100);
			remote_port = REMOTE_PORT;

			sock_comm_0 = init_udp_port( &locAddr_comm_0,
										 &gcAddr_comm_0,
										 bind_port,
										 remote_host,
										 remote_port );

			success = (sock_comm_0 > 0);

			break;
		}
		case COMM_PORT_1: {
			//strncpy(portname, "/dev/ttyROBIN0", 50);
			read_recv_comm_1 = - 1;
			bind_port = BIND_PORT + 1;
			strncpy(remote_host, REMOTE_HOST, 100);
			remote_port = REMOTE_PORT;

			sock_comm_0 = init_udp_port( &locAddr_comm_1,
										 &gcAddr_comm_1,
										 bind_port,
										 remote_host,
										 remote_port );
			success = (sock_comm_1 > 0);

			break;
		}
	}

	if(success) {
		comms_set_open( port );

		char info[100];
        snprintf(info, 100, "[COMM] Openned comm port: udp://:%i@%s:%i", bind_port, remote_host, remote_port);
		system_debug_print(info);
	}

	system_debug_print("[COMM] (initialized)");
	return success;
}

void comms_send( comms_port_t port, uint8_t ch ) {
	//int bytes_sent;

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				//bytes_sent = sendto(sock_comm_0, buf, len, 0, (struct sockaddr*)gcAddr_comm_0, sizeof(struct sockaddr_in));
				break;
			}
			case COMM_PORT_1: {
				//write(fd_comm_1, buf, 1);
				//tcdrain(fd_comm_1);
				break;
			}
		}
	}
}

bool comms_waiting( comms_port_t port ) {
	int bytes_available = 0;
	socklen_t fromlen;	//XXX: Might be able to use fromlen to check for something?

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				//If we have no bytes left in our local buffer,
				//check to see if we have a new message waiting
				if( read_recv_comm_0 < 1 ) {
					read_recv_comm_0 = recvfrom( sock_comm_0,
												 (void *)buf_recv_comm_0,
												 BUFFER_LENGTH,
												 0,
												 (struct sockaddr *)&gcAddr_comm_0,
												 &fromlen );
					//if(read_recv_comm_0 > 0) {
					//	char info[100];
					//	snprintf(info, 100, "[COMM] Read packet (bytes: %i)", read_recv_comm_0);
					//	system_debug_print(info);
					//}
				}

				bytes_available = (read_recv_comm_0 > 0) ? read_recv_comm_0 : 0;
				break;
			}
			case COMM_PORT_1: {
				//ioctl(fd_comm_1, FIONREAD, &bytes_available);
				break;
			}
		}
	}

	return (bytes_available > 0);
}

uint8_t comms_recv( comms_port_t port ) {
	uint8_t ch = 0;

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				if(read_recv_comm_0 > 0) {
					ch = buf_recv_comm_0[read_recv_comm_0 - 1];
					read_recv_comm_0--;

					char info[100];
					snprintf(info, 100, "[COMM] buff-read (remaining: %i)", read_recv_comm_0);
					system_debug_print(info);
				}

				break;
			}
			case COMM_PORT_1: {
				//rdlen = read(fd_comm_1, buf, 1);
				break;
			}
		}
	}

	return ch;
}


