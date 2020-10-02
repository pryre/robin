#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>

#include "mavlink_system.h"
#include "params.h"

#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include "drivers/posix_common/drv_cmd_args.h"

#define BUFFER_LENGTH 2048

//#define BIND_HOST "127.0.0.1"
#define BIND_PORT 14555 // The port on which to send data
#define REMOTE_HOST "127.0.0.1"
#define REMOTE_PORT 14550 // The port on which to send data

static int sock_comm_0;
static struct sockaddr_in locAddr_comm_0;
static struct sockaddr_in gcAddr_comm_0;
static uint8_t buf_recv_comm_0[BUFFER_LENGTH];
static int32_t len_recv_comm_0;
static int32_t read_recv_comm_0;

static int sock_comm_1;
static struct sockaddr_in locAddr_comm_1;
static struct sockaddr_in gcAddr_comm_1;
static uint8_t buf_recv_comm_1[BUFFER_LENGTH];
static int32_t len_recv_comm_1;
static int32_t read_recv_comm_1;

static int init_udp_port( struct sockaddr_in* locAddr,
						  struct sockaddr_in* gcAddr, uint32_t bind_port,
						  char* remote_ip, uint32_t remote_port ) {
	// Change the target ip if parameter was given
	int sock = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );

	memset( locAddr, 0, sizeof( *locAddr ) );
	locAddr->sin_family = AF_INET;
	locAddr->sin_addr.s_addr = INADDR_ANY;
	locAddr->sin_port = htons( bind_port );

	// Bind the socket to port BIND_PORT
	if ( bind( sock, (struct sockaddr*)locAddr, sizeof( struct sockaddr ) ) < 0 ) {
		system_debug_print( "[COMMS] Error: port bind failed (%i)", bind_port );

		close( sock );
		return false;
	}

	if ( sock > 0 ) {
		if ( fcntl( sock, F_SETFL, O_NONBLOCK | O_ASYNC ) < 0 ) {
			system_debug_print( "[COMMS] Error: error setting nonblocking (%s)",
								strerror( errno ) );

			close( sock );
			return false;
		}
	}

	memset( gcAddr, 0, sizeof( *gcAddr ) );
	gcAddr->sin_family = AF_INET;
	gcAddr->sin_addr.s_addr = inet_addr( remote_ip );
	gcAddr->sin_port = htons( remote_port );

	return sock;
}

static bool udp_split( uint32_t* bind_port, char* remote_host,
					   uint32_t* remote_port, const char* const conn_udp ) {
					   
	char udp_head_str[] = "udp://";
	char bind_port_str[100] = {0};
	char remote_host_str[100] = {0};
	char remote_port_str[100] = {0};
	uint32_t bind_port_cnt = 0;
	uint32_t remote_host_cnt = 0;
	uint32_t remote_port_cnt = 0;

	bool found_lhost = false;
	bool found_lport = false;
	bool fount_rhost = false;
	bool success = false;

	//Start our search at either 0 or at the end of the udp header if present
	char* udp_head = strstr(conn_udp, udp_head_str);
	int i = (udp_head == NULL) ? 0 : strlen(udp_head_str);
	
	while ( ( conn_udp[i] != '\0' ) && ( i < 100 ) ) {
		if ( !found_lhost ) {
			// Skip local host
			if ( conn_udp[i] == ':' ) {
				found_lhost = true;
			}
		} else if ( !found_lport ) {
			// Get local port
			if ( conn_udp[i] == '@' ) {
				found_lport = true;
			} else {
				bind_port_str[bind_port_cnt] = conn_udp[i];
				bind_port_cnt++;
			}
		} else if ( !fount_rhost ) {
			// Get remote host
			if ( conn_udp[i] == ':' ) {
				fount_rhost = true;
			} else {
				remote_host_str[remote_host_cnt] = conn_udp[i];
				remote_host_cnt++;
			}
		} else {
			// Get remote port
			remote_port_str[remote_port_cnt] = conn_udp[i];
			remote_port_cnt++;
		}

		i++;
	}

	success = found_lhost && found_lport && fount_rhost;

	if ( success ) {
		*bind_port = atoi( bind_port_str );
		if ( remote_host_cnt == 0 ) {
			strncpy( remote_host, "0.0.0.0", 8 );
		} else {
			strncpy( remote_host, remote_host_str, 100 );
		}
		*remote_port = atoi( remote_port_str );
	}

	return success;
}

bool comms_init_port( comms_port_t port ) {
	bool success = false;

	uint32_t bind_port;
	char remote_host[100];
	uint32_t remote_port;

	switch ( port ) {
	case COMM_PORT_0: {
		len_recv_comm_0 = 0;
		read_recv_comm_0 = 0;

		char conn_udp[100];
		strncpy( conn_udp, &_arguments.conn_telem0[6], 94 );

		if ( udp_split( &bind_port, remote_host, &remote_port, conn_udp ) ) {
			sock_comm_0 = init_udp_port( &locAddr_comm_0, &gcAddr_comm_0, bind_port,
										 remote_host, remote_port );
			success = ( sock_comm_0 > 0 );
		} else {
			system_debug_print( "[COMMS] Failed to parse telem0 comm port: %s",
								_arguments.conn_telem0 );
		}

		break;
	}
	case COMM_PORT_1: {
		len_recv_comm_1 = 0;
		read_recv_comm_1 = 0;

		char conn_udp[100];
		strncpy( conn_udp, &_arguments.conn_telem1[6], 94 );

		if ( udp_split( &bind_port, remote_host, &remote_port, conn_udp ) ) {
			sock_comm_1 = init_udp_port( &locAddr_comm_1, &gcAddr_comm_1, bind_port,
										 remote_host, remote_port );
			success = ( sock_comm_1 > 0 );
		} else {
			system_debug_print( "[COMMS] Failed to parse telem1 comm port: %s",
								_arguments.conn_telem1 );
		}

		break;
	}
	}

	if ( success ) {
		comms_set_open( port );
		system_debug_print( "[COMMS] Openned comm port: udp://:%i@%s:%i", bind_port,
							remote_host, remote_port );
	}

	return success;
}

static bool clean_close_socket( int sock ) {
	bool success = true;	//Return true by default if it was already closed

	if ( sock > 0 ) {
		shutdown(sock, 2);	//Immediately stop both reception and transmission
		success = (close(sock) == 0);
	}

	return success;
}

bool comms_deinit_port( comms_port_t port ) {
	bool success = false;

	switch ( port ) {
	case COMM_PORT_0: {
		success = clean_close_socket(sock_comm_0);

		if(success)
			sock_comm_0 = 0;

		break;
	}
	case COMM_PORT_1: {
		success = clean_close_socket(sock_comm_1);

		if(success)
			sock_comm_1 = 0;

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

void comms_send_datagram( comms_port_t port, uint8_t* datagram,
						  uint32_t length ) {
	int bytes_sent;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			bytes_sent = sendto( sock_comm_0, datagram, length, 0,
								 (struct sockaddr*)&gcAddr_comm_0, sizeof( struct sockaddr_in ) );

			break;
		}
		case COMM_PORT_1: {
			bytes_sent = sendto( sock_comm_1, datagram, length, 0,
								 (struct sockaddr*)&gcAddr_comm_1, sizeof( struct sockaddr_in ) );

			break;
		}
		}
	}

	if ( bytes_sent != length ) {
		comms_tx_error( port );
	}
}

bool comms_waiting( comms_port_t port ) {
	int bytes_available = 0;
	socklen_t fromlen; // XXX: Might be able to use fromlen to check for
	// something?

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			// If we have no bytes left in our local buffer,
			// check to see if we have a new message waiting
			if ( len_recv_comm_0 < 1 ) {
				len_recv_comm_0 = recvfrom( sock_comm_0, (void*)buf_recv_comm_0, BUFFER_LENGTH, 0,
											(struct sockaddr*)&gcAddr_comm_0, &fromlen );
			}

			bytes_available = ( len_recv_comm_0 > 0 ) ? len_recv_comm_0 : 0;

			break;
		}
		case COMM_PORT_1: {
			if ( len_recv_comm_1 < 1 ) {
				len_recv_comm_1 = recvfrom( sock_comm_1, (void*)buf_recv_comm_1, BUFFER_LENGTH, 0,
											(struct sockaddr*)&gcAddr_comm_1, &fromlen );
			}

			bytes_available = ( len_recv_comm_1 > 0 ) ? len_recv_comm_1 : 0;

			break;
		}
		}
	}

	return ( bytes_available > 0 );
}

uint8_t comms_recv( comms_port_t port ) {
	uint8_t ch = 0;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			if ( len_recv_comm_0 > read_recv_comm_0 ) {
				ch = buf_recv_comm_0[read_recv_comm_0++];
			}

			// Reset the buffer reader
			if ( ( len_recv_comm_0 > 0 ) && ( read_recv_comm_0 >= len_recv_comm_0 ) ) {
				read_recv_comm_0 = 0;
				len_recv_comm_0 = 0;
			}

			break;
		}
		case COMM_PORT_1: {
			if ( len_recv_comm_1 > read_recv_comm_1 ) {
				ch = buf_recv_comm_1[read_recv_comm_1++];
			}

			// Reset the buffer reader
			if ( ( len_recv_comm_1 > 0 ) && ( len_recv_comm_1 <= read_recv_comm_1 ) ) {
				read_recv_comm_1 = 0;
				len_recv_comm_1 = 0;
			}

			break;
		}
		}
	}

	return ch;
}
