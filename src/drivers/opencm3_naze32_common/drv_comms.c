#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_comms.h"
#include "params.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

//===============================================================
// Ring buffer for IO storage
//===============================================================

#define BUFFER_SIZE MAVLINK_MAX_PACKET_LEN

typedef int32_t ring_size_t;

typedef struct {
	uint8_t data[BUFFER_SIZE];
	ring_size_t size;
	uint32_t begin;
	uint32_t end;
} ring_data_t;

static ring_data_t usart_1_ring_in;
static ring_data_t usart_1_ring_out;

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

static void ring_init(ring_data_t *ring) {
	ring->size = BUFFER_SIZE;
	ring->begin = 0;
	ring->end = 0;

	//Clear out buffer
	for (int i = 0; i < ring->size; i++) {
		ring->data[i] = 0;
	}
}

static int32_t ring_write_ch(ring_data_t *ring, uint8_t ch) {
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (uint32_t)ch;
	}

	return -1;
}

static int32_t ring_write(ring_data_t *ring, uint8_t *data, ring_size_t size) {
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static bool ring_data_available(ring_data_t *ring) {
	//TODO: Actually make this give a read out
	return ring->begin != ring->end;
}

static int32_t ring_read_ch(ring_data_t *ring, uint8_t *ch) {
	int32_t ret = -1;

	if ( ring_data_available(ring) ) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

//===============================================================
// Low level comms IO and interrupts
//===============================================================

static int usart_write( comms_port_t port, uint8_t *data, int len) {
	int success = -1;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
			case COMM_PORT_0: {
				success = ring_write(&usart_1_ring_out, data, len);

				if (success < 0)
					success = -success;

				USART_CR1(USART1) |= USART_CR1_TXEIE;

				break;
			}
			case COMM_PORT_1: {
				//TODO

				break;
			}
		}
	}

	//errno = EIO;
	return success;
}

/*
static int usart_write_ch( comms_port_t port, uint8_t ch) {
	return usart_write(port, &ch, 1);
}
*/

void usart1_isr(void) {
	// Check if we were called because of RXNE.
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		// Indicate that we got data.
		//gpio_toggle(GPIOC, GPIO12);

		// Retrieve the data from the peripheral.
		int32_t data = usart_recv(USART1);
		ring_write_ch(&usart_1_ring_in, data);

		// Enable transmit interrupt so it sends back the data (remote echo)
		//ring_write_ch(&usart_1_ring_out, data);
		// USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		int32_t data;
		data = ring_read_ch(&usart_1_ring_out, NULL);

		if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		} else {
			/* Put data into the transmit register. */
			usart_send(USART1, data);
		}
	}
}

//===============================================================
// High level comms
//===============================================================

bool comms_init_port( comms_port_t port ) {
	bool success = false;

	switch ( port ) {
		case COMM_PORT_0: {
			//Set up IO and interrupts
			rcc_periph_clock_enable(RCC_USART1);
			nvic_enable_irq(NVIC_USART1_IRQ);
			gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
						  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
			gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

			ring_init(&usart_1_ring_in);
			ring_init(&usart_1_ring_out);

			//Configure port parameters
			usart_set_baudrate(USART1, get_param_uint( PARAM_BAUD_RATE_0 ));
			usart_set_databits(USART1, 8);
			usart_set_stopbits(USART1, USART_STOPBITS_1);
			usart_set_parity(USART1, USART_PARITY_NONE);
			usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
			usart_set_mode(USART1, USART_MODE_TX_RX);

			// Enable USART1 Receive interrupt and enable USART
			USART_CR1(USART1) |= USART_CR1_RXNEIE;
			usart_enable(USART1);

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

/*
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
*/

void comms_send_datagram( comms_port_t port,
						  uint8_t* datagram,
						  uint32_t length ) {
	if ( length <= MAVLINK_MAX_PACKET_LEN )
		usart_write(port, datagram, length);
	//	for ( uint32_t i = 0; i < length; i++ )
	//		comms_send( port, datagram[i] );
}

bool comms_waiting( comms_port_t port ) {
	bool waiting = false;

	if ( comms_is_open( port ) ) {
		switch ( port ) {
			case COMM_PORT_0: {
				waiting = ring_data_available(&usart_1_ring_in);
				break;
			}
			case COMM_PORT_1: {
				waiting = 0; //TODO: ring_has_data(&usart_1_ring_in);
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
				ch = ring_read_ch(&usart_1_ring_in, NULL);
				break;
			}
			case COMM_PORT_1: {
				ch = 0; //TODO: ring_read_ch(&usart_2_ring_in, NULL);
				break;
			}
		}
	}

	return ch;
}
