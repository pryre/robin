#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"

bool comms_init_port( comms_port_t port ) {
	bool success = false;

	return success;
}

void comms_send( comms_port_t port, uint8_t ch ) {

}

bool comms_waiting( comms_port_t port ) {
	return false;
}

uint8_t comms_recv( comms_port_t port ) {
	return 0;
}
