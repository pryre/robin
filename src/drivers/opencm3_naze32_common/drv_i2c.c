/*
   i2c.c :  I^2C support for STM32F103

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/i2c.c
   Adapted from https://github.com/BreezySTM32/BreezySTM32
 */

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/cm3/cortex.h>

#include "drivers/opencm3_naze32_common/drv_i2c.h"
#include "drivers/drv_system.h"

static volatile drv_i2c_jb_t i2c1_jobs_;
static volatile drv_i2c_jb_t i2c2_jobs_;

//Setup jobs so that we don't get aliasing between IO
static volatile drv_i2c_job_t i2c1_job_cur_;
static volatile drv_i2c_job_t i2c2_job_cur_;

static volatile uint16_t i2c1_error_count_;
static volatile uint16_t i2c2_error_count_;

static void drv_i2c_unstick( uint32_t port, uint32_t scl, uint32_t sda ) {
	int i;

	// Pull lines high
	gpio_set(port, scl | sda);

	for (i = 0; i < 8; i++) {
		// Wait for any clock stretching to finish
		while( !gpio_get(port, scl) )
			system_pause_us(10);

		// Pull low
		gpio_clear(port, scl); // Set bus low
		system_pause_us(10);
		// Release high again
		gpio_set(port, scl); // Set bus high
		system_pause_us(10);
	}

	// Generate a start then stop condition
	gpio_clear(port, sda); // Set bus data low
	system_pause_us(10);
	gpio_clear(port, scl); // Set bus scl low
	system_pause_us(10);
	gpio_set(port, scl); // Set bus scl high
	system_pause_us(10);
	gpio_set(port, sda); // Set bus sda high
}

static bool drv_i2c_handle_hardware_failure(uint32_t i2c, bool reset) {
	switch(i2c) {
		case I2C1: {
			i2c1_error_count_++;
			break;
		}
		case I2C2: {
			i2c2_error_count_++;
			break;
		}
	}

	if (reset) {
		//say("hardware failure -> resetting");
		drv_i2c_init(i2c);	// reinit peripheral + clock out garbage
	}

	return false;
}

uint32_t drv_i2c_get_error_count(uint32_t i2c) {
	uint32_t count = 0;

	switch(i2c) {
		case I2C1: {
			count = i2c1_error_count_;
			break;
		}
		case I2C2: {
			count = i2c2_error_count_;
			break;
		}
	}

	return count;
}

static bool drv_i2c_handle_fresh_port(uint32_t i2c) {
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;

	// if we are restarting the driver
	if (!(I2C_CR2(i2c) & I2C_CR2_ITEVTEN)) {
		if (!(I2C_CR1(i2c) & I2C_CR1_START)) { // ensure sending a start only once
			// wait for any stop to finish sending
			while (I2C_CR1(i2c) & I2C_CR1_STOP && --timeout > 0)
				__asm__("nop");

			//Throw an error if something messed up
			if (timeout == 0) {
				//say("i2c init timeout!");
				return drv_i2c_handle_hardware_failure(i2c, true);
			}

			i2c_send_start(i2c); // send the start for the new job
		}

		i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
		i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
	}

	return true;
}

static volatile drv_i2c_job_t* drv_i2c_get_job_cur_p(uint32_t i2c) {
	volatile drv_i2c_job_t* job = NULL;

	switch(i2c) {
		case I2C1: {
			job = &i2c1_job_cur_;
			break;
		}
		case I2C2: {
			job = &i2c2_job_cur_;
			break;
		}
	}

	return job;
}

static volatile drv_i2c_jb_t* drv_i2c_get_jb_p(uint32_t i2c) {
	volatile drv_i2c_jb_t* jb = NULL;

	switch(i2c) {
		case I2C1: {
			jb = &i2c1_jobs_;
			break;
		}
		case I2C2: {
			jb = &i2c2_jobs_;
			break;
		}
	}

	return jb;
}

bool drv_i2c_write_buffer(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
	volatile drv_i2c_job_status_t status = I2C_JOB_DEFAULT;

	drv_i2c_queue_job( i2c,
					   I2C_JOB_TYPE_WRITE,
					   addr,
					   reg,
					   data,
					   len,
					   &status,
					   NULL );

	while( ( status != I2C_JOB_DEFAULT ) &&
		   ( status != I2C_JOB_ERROR ) &&
		   ( status != I2C_JOB_COMPLETE ) ) {

		__asm__("nop");
	}

	return (status == I2C_JOB_COMPLETE);
}

bool drv_i2c_read_buffer(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
	volatile drv_i2c_job_status_t status = I2C_JOB_DEFAULT;

	drv_i2c_queue_job( i2c,
					   I2C_JOB_TYPE_READ,
					   addr,
					   reg,
					   data,
					   len,
					   &status,
					   NULL );

	while( ( status != I2C_JOB_DEFAULT ) &&
		   ( status != I2C_JOB_ERROR ) &&
		   ( status != I2C_JOB_COMPLETE ) ) {

		__asm__("nop");
	}

	return (status == I2C_JOB_COMPLETE);

}

bool drv_i2c_read_register(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t *data) {
	return drv_i2c_read_buffer(i2c, addr, reg, data, 1);
}

bool drv_i2c_write_register(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data) {
	return drv_i2c_write_buffer(i2c, addr, reg, &data, 1);
}

static void drv_i2c_job_handler(uint32_t i2c) {
	volatile drv_i2c_jb_t* jb = drv_i2c_get_jb_p(i2c);

	// the queue is no empty
	if(jb->count > 0) {
		volatile drv_i2c_job_t* jobc = drv_i2c_get_job_cur_p(i2c);

		// If the current job is not busy (i.e. no started)
		if(!jobc->busy) {
			CM_ATOMIC_BLOCK() {
			//Get a pointer to the job on the front of the queue
			volatile drv_i2c_job_t* job = jb->buffer + jb->tail;
			job->busy = true;
			job->error = I2C_JOB_ERROR_NONE;

			//Set the current job up for transmission
			jobc->i2c = job->i2c;
			jobc->type = job->type;
			jobc->addr = job->addr;
			jobc->reg = job->reg;
			jobc->data = job->data;
			jobc->length = job->length;
			jobc->status = job->status;
			jobc->CB = job->CB;

			jobc->busy = job->busy;
			jobc->error = job->error;
			(*(jobc->status)) = I2C_JOB_BUSY;

			// Increment the tail
			jb->tail = (jb->tail + 1) % I2C_BUFFER_SIZE;

			// Decrement the number of jobs on the buffer
			jb->count--;
			}

			//Finally we init the job
			//See if we need to do some initialiation
			//Set a job error imidiately if there is a setup issue
			if( !drv_i2c_handle_fresh_port(i2c) ) {
				//say("Job queue error");
				jobc->error = I2C_JOB_ERROR_GENERIC;
			}
		}
	}
}

void drv_i2c_queue_job(uint32_t i2c, drv_i2c_job_type_t type, uint8_t addr, uint8_t reg, volatile uint8_t *data, uint8_t len, volatile drv_i2c_job_status_t* status, void (*CB)(void)) {
	volatile drv_i2c_jb_t* jb = drv_i2c_get_jb_p(i2c);

	//Make sure we have room to add a job
	if(jb->count < I2C_BUFFER_SIZE) {
	CM_ATOMIC_BLOCK() {
		// Get a pointer to the head
		volatile drv_i2c_job_t* job = jb->buffer + jb->head;

		// save the data about the job
		job->i2c = i2c;
		job->type = type;
		job->addr = addr;
		job->reg = reg;
		job->data = data;
		job->length = len;
		job->status = status;
		job->CB = CB;

		job->busy = false;
		job->error = I2C_JOB_ERROR_NONE;

		// change job status
		(*(job->status)) = I2C_JOB_QUEUED;

		// Increment the buffer size
		jb->count++;

		// Increment the buffer head for next call
		jb->head = (jb->head + 1) % I2C_BUFFER_SIZE;
	}
	}

	// If the buffer queue was empty when we added the job, restart i2c job handling
	if(jb->count == 1)
		drv_i2c_job_handler(i2c);
}

static void i2c_er_handler(uint32_t i2c) {
	i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
	bool port_running = true;

	volatile drv_i2c_job_t* job = drv_i2c_get_job_cur_p(i2c);

	//Some basic error determination
	if( I2C_SR1(i2c) & I2C_SR1_AF) {
		job->error = I2C_JOB_ERROR_ACK;
		//say("ic2_er: ack");
	} else if( I2C_SR1(i2c) & I2C_SR1_ARLO) {
		job->error = I2C_JOB_ERROR_ARBITRATION;
		//say("ic2_er: arlo");
	} else if( I2C_SR1(i2c) & I2C_SR1_BERR) {
		job->error = I2C_JOB_ERROR_BUS;
		//say("ic2_er: bus");
	} else {
		job->error = I2C_JOB_ERROR_GENERIC;
		//say("ic2_er: gen");
	}

	// If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
	if ( I2C_SR1(i2c) & ( I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF ) ) {
		__asm__("dmb");
		// read second status register to clear ADDR if it is set
		//(note that BTF will not be set after a NACK)
		(void)I2C_SR2(i2c);

		// Prepare a hardware reset if needed
		// This will "un-stick" the lines as well
		bool do_reset = false;

		// if we dont have an ARLO error, try a start-stop recovery
		if ( !(I2C_SR1(i2c) & I2C_SR1_ARLO) ) {
			// Ensure sending of a stop
			if( !(I2C_CR1(i2c) & I2C_CR1_STOP) ) {
				// We are currently trying to send a start
				// This is very bad as start, stop will hang the peripheral
				if (I2C_CR1(i2c) & I2C_CR1_START) {
					while (I2C_CR1(i2c) & I2C_CR1_START)
						__asm__("nop");	// wait for any start to finish sending

					// send stop to finalise bus transaction
					i2c_send_stop(i2c);

					while (I2C_CR1(i2c) & I2C_CR1_STOP)
						__asm__("nop");	// wait for stop to finish sending

					do_reset = true;
				} else {
					// Just send stop to finalise bus transaction
					i2c_send_stop(i2c);
				}
			}

			// Disable EVT and ERR interrupts while bus inactive
			// Should help with recovery
			port_running = false;
		} else {
			//Otherwise we have an ARLO
			//We might be able to recover if it was a bad clock signal
			do_reset = true;
		}

		//Reset if needed
		if(do_reset)
			drv_i2c_init(i2c);
	}

	I2C_SR1(i2c) &= ~0x0F00;											   // reset all the error bits to clear the interrupt

	if (job->status != NULL)
		(*(job->status)) = I2C_JOB_ERROR;									  // Update job status

	if (job->CB != NULL)
		job->CB();

	job->busy = false;

	//Switch back on the ev interrupt
	if(port_running) {
		i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
	} else {  //Shut down the port
		i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);
	}

	//Kick off the next async job if needed
	drv_i2c_job_handler(i2c);
}

static void i2c_ev_handler(uint32_t i2c) {
	i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);

	volatile drv_i2c_job_t* job = drv_i2c_get_job_cur_p(i2c);

	static uint8_t subaddress_sent, final_stop;						 // flag to indicate if subaddess sent, flag to indicate final bus condition
	static int8_t index;												// index is signed -1 == send the subaddress

	if (I2C_SR1(i2c) & I2C_SR1_SB) {
		// Start send has just completed - EV5 in ref manual
		I2C_CR1(i2c) &= ~I2C_CR1_POS;										   // reset the POS bit so ACK/NACK applied to the current byte
		i2c_enable_ack(i2c);							// make sure ACK is on
		index = 0;													  // reset the index

		if ((job->type == I2C_JOB_TYPE_READ) && (subaddress_sent || 0xFF == job->reg)) {			  // we have sent the subaddr
			subaddress_sent = 1;										// make sure this is set in case of no subaddress, so following code runs correctly

			if (job->length == 2)
				I2C_CR1(i2c) |= I2C_CR1_POS;									// set the POS bit so NACK applied to the final byte in the two byte read

			i2c_send_7bit_address(i2c, job->addr, I2C_READ);	// send the address and set hardware mode
		} else {														// direction is Tx, or we havent sent the sub and rep start
			i2c_send_7bit_address(i2c, job->addr, I2C_WRITE); // send the address and set hardware mode

			if (job->reg != 0xFF)											// 0xFF as subaddress means it will be ignored, in Tx or Rx mode
				index = -1;											 // send a subaddress
		}
	} else if (I2C_SR1(i2c) & I2C_SR1_ADDR) {
		// Address send has just completed - EV6 in ref manual
		// Read SR1,2 to clear ADDR
		__asm__("dmb");	// memory fence to control hardware

		if (job->length == 1 && (job->type == I2C_JOB_TYPE_READ) && subaddress_sent) {
			// we are receiving 1 byte - EV6_3
			i2c_disable_ack(i2c);	// turn off ACK
			__asm__("dmb");
			(void)I2C_SR2(i2c);		// clear ADDR after ACK is turned off
			i2c_send_stop(i2c);		// program the stop

			final_stop = 1;
			i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);	// allow us to have an EV7
		} else {
			// EV6 and EV6_1
			(void)I2C_SR2(i2c);	// clear the ADDR here
			__asm__("dmb");

			if (job->length == 2 && (job->type == I2C_JOB_TYPE_READ) && subaddress_sent) {
				 // rx 2 bytes - EV6_1
				i2c_disable_ack(i2c);							// turn off ACK
				i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);	//disable TXE to allow the buffer to fill
			} else if (job->length == 3 && (job->type == I2C_JOB_TYPE_READ) && subaddress_sent) {
				// rx 3 bytes
				// make sure RXNE disabled so we get a BTF in two bytes time
				i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
			} else {
				// receiving greater than three bytes, sending subaddress, or transmitting
				i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);
			}
		}
	} else if (I2C_SR1(i2c) & I2C_SR1_BTF) {
		// Byte transfer has just been completed - EV7_2, EV7_3 or EV8_2
		final_stop = 1;
		if ((job->type == I2C_JOB_TYPE_READ) && subaddress_sent) {							   // EV7_2, EV7_3
			if (job->length > 2) {											// EV7_2
				i2c_disable_ack(i2c);				   // turn off ACK
				job->data[index++] = (uint8_t)I2C_DR(i2c);					// read data N-2
				i2c_send_stop(i2c);						 // program the Stop
				final_stop = 1;										 // required to fix hardware
				job->data[index++] = (uint8_t)I2C_DR(i2c);					// read data N - 1
				i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);				 // enable TXE to allow the final EV7
			} else {													// EV7_3
				if (final_stop){
					i2c_send_stop(i2c);					 // program the Stop
				} else {
					i2c_send_start(i2c);					// program a rep start
				}
				job->data[index++] = (uint8_t)I2C_DR(i2c);				// read data N - 1
				job->data[index++] = (uint8_t)I2C_DR(i2c);				// read data N
				index++;												// to show job completed
			}
		} else {														// EV8_2, which may be due to a subaddress sent or a write completion
			if (subaddress_sent || (job->type == I2C_JOB_TYPE_WRITE)) {
				if (final_stop)
					i2c_send_stop(i2c);					 // program the Stop
				else
					i2c_send_start(i2c);					// program a rep start
				index++;												// to show that the job is complete
			} else {													// We need to send a subaddress
				i2c_send_start(i2c);						// program the repeated Start
				subaddress_sent = 1;									// this is set back to zero upon completion of the current task
			}
		}
		// we must wait for the start to clear, otherwise we get constant BTF
		while (I2C_CR1(i2c) & I2C_CR1_START) {
			__asm__("nop");
		}
	} else if (I2C_SR1(i2c) & I2C_SR1_RxNE) {
		// Byte received - EV7
		job->data[index++] = (uint8_t)I2C_DR(i2c);
		if (job->length == (index + 3))
			i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);					// disable TXE to allow the buffer to flush so we can get an EV7_2
		if (job->length == index)											 // We have completed a final EV7
			index++;													// to show job is complete
	} else if (I2C_SR1(i2c) & I2C_SR1_TxE) {
		// Byte transmitted EV8 / EV8_1
		if (index != -1) {											  // we dont have a subaddress to send
			I2C_DR(i2c) = job->data[index++];
			if (job->length == index)										 // we have sent all the data
				i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);				// disable TXE to allow the buffer to flush
		} else {
			index++;
			I2C_DR(i2c) = job->reg;											 // send the subaddress
			if ( (job->type == I2C_JOB_TYPE_READ) || !job->length)									  // if receiving or sending 0 bytes, flush now
				i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);				// disable TXE to allow the buffer to flush
		}
	}

	if (index == job->length + 1) {										   // we have completed the current job
		subaddress_sent = 0;											// reset this here

		if (job->status != NULL)
			(*(job->status)) = I2C_JOB_COMPLETE;							   // Update status

		if (job->CB != NULL)
			job->CB();											  // Call the custom callback (we are finished)

		job->busy = false;

		// If there is a final stop and no more jobs, bus is inactive,
		// disable interrupts to prevent BTF
		// Disable EVT and ERR interrupts while bus inactive
		if(final_stop) {
			//Shut down the port
			i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
		} else {
			//Switch back on the er interrupt
			i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
		}

		//Kick off the next async job if needed
		drv_i2c_job_handler(i2c);
	}
}

//XXX: We need to disable interrupts when handling others to avoid conflicts
void i2c1_ev_isr(void) {
	i2c_ev_handler(I2C1);
}

void i2c2_ev_isr(void) {
	i2c_ev_handler(I2C2);
}

void i2c1_er_isr(void) {
	i2c_er_handler(I2C1);
}

void i2c2_er_isr(void) {
	i2c_er_handler(I2C2);
}

static void drv_i2c_memset_volatile(volatile void *s, char c, size_t n) {
    volatile char *p = s;
    while (n-- > 0)
        *p++ = c;
}

static void drv_i2c_init_buffer(uint32_t i2c) {
	volatile drv_i2c_jb_t* jb = drv_i2c_get_jb_p(i2c);

	drv_i2c_memset_volatile(jb->buffer, 0, I2C_BUFFER_SIZE*sizeof(drv_i2c_job_t));
	jb->count = 0;
	jb->head = 0;
	jb->tail = 0;
}

void drv_i2c_init(uint32_t i2c) {
	bool is_valid = false;
	uint32_t rcc_clock = 0;
	uint32_t rcc_reset = 0;
	uint32_t rcc_port = 0;
	uint32_t nvic_ev = 0;
	uint32_t nvic_er = 0;

	uint32_t port = 0;
	uint32_t sda = 0;
	uint32_t scl = 0;

	switch(i2c) {
		case I2C1: {
			rcc_clock = RCC_I2C1;
			rcc_reset = RST_I2C1;
			rcc_port = RCC_GPIOB;
			nvic_ev = NVIC_I2C1_EV_IRQ;
			nvic_er = NVIC_I2C1_ER_IRQ;

			port = GPIOB;
			scl = GPIO_I2C1_SCL;
			sda = GPIO_I2C1_SDA;

			//say("Initializing i2c1");

			is_valid = true;
			i2c1_error_count_ = 0;

			break;
		}
		case I2C2: {
			rcc_clock = RCC_I2C2;
			rcc_reset = RST_I2C2;
			rcc_port = RCC_GPIOB;
			nvic_ev = NVIC_I2C2_EV_IRQ;
			nvic_er = NVIC_I2C2_ER_IRQ;

			port = GPIOB;
			scl = GPIO_I2C2_SCL;
			sda = GPIO_I2C2_SDA;

			//say("Initializing i2c2");

			is_valid = true;
			i2c2_error_count_ = 0;

			break;
		}
	}

	if( is_valid ) {

		rcc_periph_clock_enable(rcc_clock);
		rcc_periph_reset_pulse(rcc_reset);

		nvic_enable_irq(nvic_ev);
		nvic_enable_irq(nvic_er);

		rcc_periph_clock_enable(rcc_port);
		gpio_set_mode(port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, scl | sda);

		drv_i2c_unstick( port, scl, sda );
		drv_i2c_init_buffer(i2c);

		i2c_set_speed(i2c, i2c_speed_fm_400k, I2C_CR2_FREQ_36MHZ);
		//i2c_set_dutycycle(i2c, I2C_CCR_DUTY_DIV2);

		//Clear slave address (not needed for master)
		i2c_set_own_7bit_slave_address(i2c, 0);

		//Eeverything is configured, enable the peripheral.
		//Make sure we don't have the interrupt firing premptively
		i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
		i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);

		i2c_peripheral_enable(i2c);
	}
}
