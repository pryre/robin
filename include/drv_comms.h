#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	COMM_PORT_0 = 0x01,	//0b00000001
	COMM_PORT_1 = 0x02	//0b00000010
} comms_port_t;

bool comms_init_port( comms_port_t port );

void comms_set_open( comms_port_t port );
void comms_set_closed( comms_port_t port );
bool comms_is_open( comms_port_t port );


void comms_send( comms_port_t port, uint8_t ch );

bool comms_waiting( comms_port_t port );
uint8_t comms_recv( comms_port_t port );
