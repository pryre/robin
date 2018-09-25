#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"

static comms_port_t comms_open_status_ = 0;

void comms_set_open( comms_port_t port ) {
	comms_open_status_ |= port;
}

void comms_set_closed( comms_port_t port ) {
	comms_open_status_ &= ~port;
}

bool comms_is_open( comms_port_t port ) {
	return comms_open_status_ & port;
}
