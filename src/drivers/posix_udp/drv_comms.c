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

#define BUFFER_LENGTH 2048

//#define BIND_HOST "127.0.0.1"
#define BIND_PORT 14550   //The port on which to send data
#define REMOTE_HOST "127.0.0.1"
#define REMOTE_PORT 14555   //The port on which to send data


static int sock_comm_0;
static struct sockaddr_in locAddr_comm_0;
static struct sockaddr_in gcAddr_comm_0;
static uint8_t buf_recv_comm_0[BUFFER_LENGTH];
static uint8_t buf_send_comm_0[BUFFER_LENGTH];
static int32_t len_recv_comm_0;
static int32_t read_recv_comm_0;
static uint32_t len_send_comm_0;
static uint32_t time_send_addbuf_comm_0;

static int sock_comm_1;
static struct sockaddr_in locAddr_comm_1;
static struct sockaddr_in gcAddr_comm_1;
static uint8_t buf_recv_comm_1[BUFFER_LENGTH];
static uint8_t buf_send_comm_1[BUFFER_LENGTH];
static int32_t len_recv_comm_1;
static int32_t read_recv_comm_1;
static uint32_t len_send_comm_1;
static uint32_t time_send_addbuf_comm_1;

static void udp_buffer_send( comms_port_t port ) {
	int bytes_sent;
	int expected_send_len;

	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				expected_send_len = len_send_comm_0;
				bytes_sent = sendto( sock_comm_0,
									 buf_send_comm_0,
									 len_send_comm_0,
									 0,
									 (struct sockaddr*)&gcAddr_comm_0,
									 sizeof(struct sockaddr_in) );

				//if(bytes_sent > 0) {
				//	char info[250];
				//	snprintf(info, 250, "[COMMS] Sent packet (bytes: %i)", bytes_sent);
				//	system_debug_print(info);
				//}

				memset( buf_send_comm_0, 0, BUFFER_LENGTH );
				len_send_comm_0 = 0;
				time_send_addbuf_comm_0 = 0;

				break;
			}
			case COMM_PORT_1: {
				expected_send_len = len_send_comm_1;
				bytes_sent = sendto( sock_comm_1,
									 buf_send_comm_1,
									 len_send_comm_1,
									 0,
									 (struct sockaddr*)&gcAddr_comm_1,
									 sizeof(struct sockaddr_in) );

				memset( buf_send_comm_1, 0, BUFFER_LENGTH );
				len_send_comm_1 = 0;
				time_send_addbuf_comm_1 = 0;

				break;
			}
		}
	}

	if( bytes_sent != expected_send_len ) {
		comms_tx_error(port);
	}
}

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
        snprintf(info, 100, "[COMMS] Error: port bind failed (%i)", bind_port);
		system_debug_print(info);

		close(sock);
		return false;
    }

	if(sock > 0 ) {
		if ( fcntl( sock, F_SETFL, O_NONBLOCK | O_ASYNC ) < 0 ) {
			char info[100];
			snprintf(info, 100, "[COMMS] Error: error setting nonblocking (%s)", strerror(errno));
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
	bool success = false;

	uint32_t bind_port;
	char remote_host[100];
	uint32_t remote_port;

	switch(port) {
		case COMM_PORT_0: {
			len_recv_comm_0 = 0;
			read_recv_comm_0 = 0;
			len_send_comm_0 = 0;
			time_send_addbuf_comm_0 = 0;

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
			len_recv_comm_1 = 0;
			read_recv_comm_1 = 0;
			len_send_comm_0 = 0;
			time_send_addbuf_comm_1 = 0;

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
        snprintf(info, 100, "[COMMS] Openned comm port: udp://:%i@%s:%i", bind_port, remote_host, remote_port);
		system_debug_print(info);
	}

	return success;
}

void comms_send( comms_port_t port, uint8_t ch ) {
	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				//If the buffer is full or the last character was added >1ms ago
				//	Then assume we're on a new message and send out the buffer
				//XXX: Maybe do this elsewhere on a proper timer?
				if( (len_send_comm_0 >= (BUFFER_LENGTH - 1) ) ||
					( ( (system_micros() - time_send_addbuf_comm_0) > 10 ) &&
					  ( time_send_addbuf_comm_0 != 0 ) ) ) {
					udp_buffer_send( port );
				}

				//Pack a new character into the buffer
				if( len_send_comm_0 < (BUFFER_LENGTH - 1) ) {
					buf_send_comm_0[len_send_comm_0++] = ch;
					time_send_addbuf_comm_0 = system_micros();
				}

				break;
			}
			case COMM_PORT_1: {
				if( (len_send_comm_1 >= (BUFFER_LENGTH - 1) ) ||
					( ( (system_micros() - time_send_addbuf_comm_1) > 10 ) &&
					  ( time_send_addbuf_comm_1 != 0 ) ) ) {
					udp_buffer_send( port );
				}

				//Pack a new character into the buffer
				if( len_send_comm_1 < (BUFFER_LENGTH - 1) ) {
					buf_send_comm_1[len_send_comm_1++] = ch;
					time_send_addbuf_comm_1 = system_micros();
				}

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
				if( len_recv_comm_0 < 1 ) {
					len_recv_comm_0 = recvfrom( sock_comm_0,
												(void *)buf_recv_comm_0,
												BUFFER_LENGTH,
												0,
												(struct sockaddr *)&gcAddr_comm_0,
												&fromlen );
					/*
					if(len_recv_comm_0 > 0) {
						char info[100];
						snprintf(info, 100, "[COMMS] Read packet (bytes: %i)", len_recv_comm_0);
						system_debug_print(info);
					}
					*/
				}

				bytes_available = (len_recv_comm_0 > 0) ? len_recv_comm_0 : 0;

				break;
			}
			case COMM_PORT_1: {
				if( len_recv_comm_1 < 1 ) {
					len_recv_comm_1 = recvfrom( sock_comm_1,
												(void *)buf_recv_comm_1,
												BUFFER_LENGTH,
												0,
												(struct sockaddr *)&gcAddr_comm_1,
												&fromlen );
				}

				bytes_available = (len_recv_comm_1 > 0) ? len_recv_comm_1 : 0;

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
				if( len_recv_comm_0 > read_recv_comm_0 ) {
					ch = buf_recv_comm_0[read_recv_comm_0++];
				}

				//Reset the buffer reader
				if( (len_recv_comm_0 > 0) && ( read_recv_comm_0 >= len_recv_comm_0) ) {
					read_recv_comm_0 = 0;
					len_recv_comm_0 = 0;
				}

				break;
			}
			case COMM_PORT_1: {
				if(len_recv_comm_1 > read_recv_comm_1) {
					ch = buf_recv_comm_1[read_recv_comm_1++];
				}

				//Reset the buffer reader
				if( (len_recv_comm_1 > 0) && (len_recv_comm_1 <= read_recv_comm_1) ) {
					read_recv_comm_1 = 0;
					len_recv_comm_1 = 0;
				}

				break;
			}
		}
	}

	return ch;
}


