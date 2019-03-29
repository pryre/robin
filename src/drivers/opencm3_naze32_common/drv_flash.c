#include <stdlib.h>

#include "drivers/drv_flash.h"
#include "params.h"

#include <libopencm3/stm32/flash.h>


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

	flash_erase_page(FLASH_OPERATION_ADDRESS);
	flash_status = flash_get_status_flags();

	//XXX: Could return full status here for better handling
	if(flash_status != FLASH_SR_EOP)
		return false;	//flash_status

	// programming flash memory
	for(uint16_t i=0; i<sizeof( params_t ); i += 4) 	{
		/*programming word data*/
		flash_program_word( FLASH_OPERATION_ADDRESS+i, *(uint32_t*)( (char*)&_params + i ) );
		flash_status = flash_get_status_flags();

		//Check to see if there was an issue
		if(flash_status != FLASH_SR_EOP)
			return false; //flash_status

		// Verify programmed data is correct
		if( *((uint32_t*)(FLASH_OPERATION_ADDRESS+i)) != *(uint32_t*)( (char*)&_params + i ) )
			return false;	//FLASH_WRONG_DATA_WRITTEN;
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

	return true;
}
