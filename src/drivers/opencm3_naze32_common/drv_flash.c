#include <stdlib.h>

#include "drivers/drv_flash.h"
#include "params.h"

#include <libopencm3/stm32/flash.h>


#include "mavlink_system.h"
#include "params.h"

#include "drivers/drv_system.h"
#include "fix16.h"
#include "fixextra.h"

#include "robin_itoa.h"
#include <string.h>


#define ASSERT_CONCAT_( a, b ) a##b
#define ASSERT_CONCAT( a, b ) ASSERT_CONCAT_( a, b )
#define ct_assert( e ) \
	enum { ASSERT_CONCAT( assert_line_, __LINE__ ) = 1 / ( !!( e ) ) }

// define this symbol to increase or decrease flash size. not rely on
// flash_size_register.
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE ( (uint16_t)0x400 )
// if sizeof(_params) is over this number, compile-time error will occur. so,
// need to add another page to config data.
#define FLASH_CONFIG_SIZE ( FLASH_PAGE_SIZE * 3 )

#define FLASH_MAGIC_BE 0xBE
#define FLASH_MAGIC_EF 0xEF


params_t _params;


static const uint32_t FLASH_OPERATION_ADDRESS = 0x08000000 + ( FLASH_PAGE_SIZE * ( FLASH_PAGE_COUNT - ( FLASH_CONFIG_SIZE / 1024 ) ) );
static uint64_t _eeprom_version;


static bool drv_flash_valid( void ) {
	const params_t* pm_ptr = (const params_t*)FLASH_OPERATION_ADDRESS;

	// check version number
	if( _eeprom_version != pm_ptr->version )
		return false;

	// check size and magic numbers
	if( ( pm_ptr->size != sizeof( params_t ) ) ||
		( pm_ptr->magic_be != FLASH_MAGIC_BE ) ||
		( pm_ptr->magic_ef != FLASH_MAGIC_EF ) ) {

		return false;
	}

	// verify integrity of temporary copy
	const uint8_t* p;
	uint8_t chk = 0;

	for( p = (const uint8_t*)pm_ptr; p < ( (const uint8_t*)pm_ptr + sizeof( params_t ) ); p++ )
		chk ^= *p;

	// checksum failed
	if ( chk != 0 )
		return false;

	// looks good, let's roll!
	return true;
}

void drv_flash_init( void ) {
	// make sure (at compile time) that params_t
	// struct doesn't overflow allocated flash pages
	ct_assert( sizeof( _params ) < FLASH_CONFIG_SIZE );
	_eeprom_version = strtoll( EEPROM_CONF_VERSION_STR, NULL, 16 );
}

bool drv_flash_read( void ) {
	// Sanity check
	if ( !drv_flash_valid() )
		return false;

	// Read flash
	memcpy( &_params, (char*)FLASH_OPERATION_ADDRESS, sizeof( params_t ) );

	return true;
}

bool drv_flash_write( void ) {
	// Prepare checksum/version constants
	_params.version = _eeprom_version;
	_params.size = sizeof( params_t );
	_params.magic_be = FLASH_MAGIC_BE;
	_params.magic_ef = FLASH_MAGIC_EF;
	_params.chk = 0;

	// Recalculate checksum before writing
	const uint8_t* p;
	uint8_t chk = 0;

	for ( p = (const uint8_t*)&_params;
		  p < ( (const uint8_t*)&_params + sizeof( params_t ) ); p++ )
		chk ^= *p;

	_params.chk = chk;

	// Perform the flash
	//uint16_t i;
	//uint32_t current_address = start_address;
	//uint32_t page_address = start_address;
	uint32_t flash_status = 0;

	flash_unlock();

	// XXX: Could try multiple times for the rest

	flash_wait_for_last_operation();
	flash_clear_status_flags();

	flash_erase_page(FLASH_OPERATION_ADDRESS);
	flash_status = flash_get_status_flags();
	bool keep_going = true;

	//Flash is either busy or in error state
	if(flash_status != FLASH_SR_EOP) {
		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[FLASH] Invalid write-start status: 0x";
		char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
		robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, flash_status, 16);
		strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
		mavlink_queue_broadcast_debug( text );

		keep_going = false;
	}

	if(keep_going) {
		// programming flash memory
		uint16_t i = 0;
		for(i=0; i<sizeof( params_t ); i += 4) 	{
			// Programming word data
			flash_program_word( FLASH_OPERATION_ADDRESS+i, *(uint32_t*)( (char*)&_params + i ) );
			flash_status = flash_get_status_flags();

			// Check to see if there was an issue
			if(flash_status != FLASH_SR_EOP) {
				keep_going = false;
				break;
			}
		}

		if(!keep_going) {
			char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[FLASH] Write error: 0x";
			char numtext[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
			robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, flash_status, 16);
			strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
			strncat(text," (0x",MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
			robin_itoa(numtext, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, FLASH_OPERATION_ADDRESS+i, 16);
			strncat(text,numtext,MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
			strncat(text,")",MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN-1);
			mavlink_queue_broadcast_debug( text );
		}
	}

	if(keep_going) {
		// programming flash memory
		for(uint16_t i=0; i<sizeof( params_t ); i += 4) 	{
			// Verify programmed data is correct
			if( *((uint32_t*)(FLASH_OPERATION_ADDRESS+i)) != *(uint32_t*)( (char*)&_params + i ) ) {
				keep_going = false;
				break;
			}
		}

		if(!keep_going) {
			mavlink_queue_broadcast_debug( "[FLASH] Verify write error" );
		}
	}

	flash_lock();

	/*
	FLASH_Status status;

	for ( unsigned int tries = 3; tries; tries-- ) {
		FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );

		FLASH_ErasePage( FLASH_OPERATION_ADDRESS );
		status = FLASH_ErasePage( FLASH_OPERATION_ADDRESS + FLASH_PAGE_SIZE );

		for ( unsigned int i = 0; i < sizeof( params_t ) && status == FLASH_COMPLETE;
			  i += 4 )
			status = FLASH_ProgramWord( FLASH_WRITE_ADDR + i,
										*(uint32_t*)( (char*)&_params + i ) );

		if ( status == FLASH_COMPLETE )
			break;
	}

	flash_Lock();

	// Flash write failed - just die now
	if ( status != FLASH_COMPLETE || !validEEPROM() ) {
		return false;
	}
	*/

	return keep_going;
}
