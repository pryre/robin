#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"

bool comms_init_port( comms_port_t port ) {
	bool success = false;

	switch(port) {
		case COMM_PORT_0: {
			//...
			//success = true;

			break;
		}
		case COMM_PORT_1: {
			//...
			//success = true;

			break;
		}
	}

	if(success)
		comms_set_open( port );

	return success;
}

void comms_send( comms_port_t port, uint8_t ch ) {
	if( comms_is_open( port ) ) {
		switch(port) {
			case COMM_PORT_0: {
				//...
				break;
			}
			case COMM_PORT_1: {
				//...
				break;
			}
		}
	}
}

bool comms_waiting( comms_port_t port ) {
	return false;
}

uint8_t comms_recv( comms_port_t port ) {
	return 0;
}
