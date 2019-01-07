#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	COMM_PORT_0 = 0x01, // 0b00000001
	COMM_PORT_1 = 0x02  // 0b00000010
} comms_port_t;

// XXX: To be defined per-implementation
void comms_send_datagram( comms_port_t port, uint8_t* datagram, uint32_t length );

bool comms_init_port( comms_port_t port );

void comms_set_open( comms_port_t port );
void comms_set_closed( comms_port_t port );
bool comms_is_open( comms_port_t port );

bool comms_waiting( comms_port_t port );
uint8_t comms_recv( comms_port_t port );

void comms_tx_error( comms_port_t port );
void comms_rx_error( comms_port_t port );
uint32_t comms_tx_error_num( comms_port_t port );
uint32_t comms_rx_error_num( comms_port_t port );

#ifdef __cplusplus
}
#endif
