#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"
#include "params.h"

/*
#include "breezystm32.h"
#include "gpio.h"
#include "serial.h"
#include "serial_uart.h"

serialPort_t* Serial1;
serialPort_t* Serial2;
*/
bool comms_init_port( comms_port_t port ) {
	bool success = false;

	switch ( port ) {
		case COMM_PORT_0: {
			//Serial1 = uartOpen( USART1, NULL, get_param_uint( PARAM_BAUD_RATE_0 ),
			//					MODE_RXTX, SERIAL_NOT_INVERTED );
			success = true;

			break;
		}
		case COMM_PORT_1: {
			//Serial2 = uartOpen( USART2, NULL, get_param_uint( PARAM_BAUD_RATE_1 ),
			//					MODE_RXTX, SERIAL_NOT_INVERTED );
			success = true;

			break;
		}
	}

	if ( success )
		comms_set_open( port );

	return success;
}

static void comms_send( comms_port_t port, uint8_t ch ) {
	if ( comms_is_open( port ) ) {
		switch ( port ) {
			case COMM_PORT_0: {
				// serialWrite(Serial1, ch);
				//uartWrite( Serial1, ch );
				break;
			}
			case COMM_PORT_1: {
				// serialWrite(Serial2, ch);
				//uartWrite( Serial2, ch );
				break;
			}
		}
	}
}

void comms_send_datagram( comms_port_t port, uint8_t* datagram,
						  uint32_t length ) {
	if ( length <= MAVLINK_MAX_PACKET_LEN )
		for ( uint32_t i = 0; i < length; i++ )
			comms_send( port, datagram[i] );
}

bool comms_waiting( comms_port_t port ) {
	bool waiting = false;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
			case COMM_PORT_0: {
				waiting = 0;//uartTotalRxBytesWaiting( Serial1 );
				break;
			}
			case COMM_PORT_1: {
				waiting = 0;//uartTotalRxBytesWaiting( Serial2 );
				break;
			}
		}
	}

	return waiting;
}

uint8_t comms_recv( comms_port_t port ) {
	uint8_t ch = 0;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
		case COMM_PORT_0: {
			ch = 0;	//uartRead( Serial1 );
			break;
		}
		case COMM_PORT_1: {
			ch = 0;	//uartRead( Serial2 );
			break;
		}
		}
	}

	return ch;
}
