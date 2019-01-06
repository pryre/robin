#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"

static comms_port_t comms_open_status_ = 0;

static uint32_t comm_0_tx_error_count = 0;
static uint32_t comm_0_rx_error_count = 0;
static uint32_t comm_1_tx_error_count = 0;
static uint32_t comm_1_rx_error_count = 0;

void comms_set_open( comms_port_t port ) {
	comms_open_status_ |= port;
}

void comms_set_closed( comms_port_t port ) {
	comms_open_status_ &= ~port;
}

bool comms_is_open( comms_port_t port ) {
	return comms_open_status_ & port;
}

void comms_tx_error( comms_port_t port ) {
	switch ( port ) {
	case COMM_PORT_0: {
		comm_0_tx_error_count++;
		break;
	}
	case COMM_PORT_1: {
		comm_1_tx_error_count++;
		break;
	}
	}
}

void comms_rx_error( comms_port_t port ) {
	switch ( port ) {
	case COMM_PORT_0: {
		comm_0_rx_error_count++;
		break;
	}
	case COMM_PORT_1: {
		comm_1_rx_error_count++;
		break;
	}
	}
}

uint32_t comms_tx_error_num( comms_port_t port ) {
	uint32_t error_count = 0;

	switch ( port ) {
	case COMM_PORT_0: {
		error_count = comm_0_tx_error_count;
		break;
	}
	case COMM_PORT_1: {
		error_count = comm_1_tx_error_count;
		break;
	}
	}

	return error_count;
}

uint32_t comms_rx_error_num( comms_port_t port ) {
	uint32_t error_count = 0;

	switch ( port ) {
	case COMM_PORT_0: {
		error_count = comm_0_rx_error_count;
		break;
	}
	case COMM_PORT_1: {
		error_count = comm_1_rx_error_count;
		break;
	}
	}

	return error_count;
}

#ifdef __cplusplus
}
#endif
