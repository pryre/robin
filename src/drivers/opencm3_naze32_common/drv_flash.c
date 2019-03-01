#include <stdlib.h>

#include "drivers/drv_flash.h"
#include "params.h"

/*
#include "breezystm32.h"

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
#define CONFIG_SIZE ( FLASH_PAGE_SIZE * 3 )
*/
params_t _params;
/*
static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + ( FLASH_PAGE_SIZE * ( FLASH_PAGE_COUNT - ( CONFIG_SIZE / 1024 ) ) );
static uint64_t
	_eeprom_version; // XXX: static const uint8_t EEPROM_CONF_VERSION = 76;

static void initEEPROM( void ) {
	// make sure (at compile time) that config struct doesn't overflow allocated
	// flash pages
	ct_assert( sizeof( _params ) < CONFIG_SIZE );
	_eeprom_version = strtoll( EEPROM_CONF_VERSION_STR, NULL, 16 );
}

static bool validEEPROM( void ) {
	const params_t* temp = (const params_t*)FLASH_WRITE_ADDR;

	// check version number
	if ( _eeprom_version != temp->version )
		return false;

	// check size and magic numbers
	if ( temp->size != sizeof( params_t ) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF )
		return false;

	// verify integrity of temporary copy
	const uint8_t* p;
	uint8_t chk = 0;

	for ( p = (const uint8_t*)temp;
		  p < ( (const uint8_t*)temp + sizeof( params_t ) ); p++ )
		chk ^= *p;

	// checksum failed
	if ( chk != 0 )
		return false;

	// looks good, let's roll!
	return true;
}

static bool readEEPROM( void ) {
	// Sanity check
	if ( !validEEPROM() )
		return false;

	// Read flash
	memcpy( &_params, (char*)FLASH_WRITE_ADDR, sizeof( params_t ) );

	return true;
}

static bool writeEEPROM( void ) {
	FLASH_Status status;

	// Prepare checksum/version constants
	_params.version = _eeprom_version;
	_params.size = sizeof( params_t );
	_params.magic_be = 0xBE;
	_params.magic_ef = 0xEF;
	_params.chk = 0;

	// Recalculate checksum before writing
	const uint8_t* p;
	uint8_t chk = 0;

	for ( p = (const uint8_t*)&_params;
		  p < ( (const uint8_t*)&_params + sizeof( params_t ) ); p++ )
		chk ^= *p;

	_params.chk = chk;

	// write it
	FLASH_Unlock();

	for ( unsigned int tries = 3; tries; tries-- ) {
		FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );

		FLASH_ErasePage( FLASH_WRITE_ADDR );
		status = FLASH_ErasePage( FLASH_WRITE_ADDR + FLASH_PAGE_SIZE );

		for ( unsigned int i = 0; i < sizeof( params_t ) && status == FLASH_COMPLETE;
			  i += 4 )
			status = FLASH_ProgramWord( FLASH_WRITE_ADDR + i,
										*(uint32_t*)( (char*)&_params + i ) );

		if ( status == FLASH_COMPLETE )
			break;
	}

	FLASH_Lock();

	// Flash write failed - just die now
	if ( status != FLASH_COMPLETE || !validEEPROM() ) {
		return false;
	}

	return true;
}
*/
void drv_flash_init( void ) {
	//initEEPROM();
}

bool drv_flash_read( void ) {
	return false;	//readEEPROM();
}

bool drv_flash_write( void ) {
	return true;	//writeEEPROM();
}
