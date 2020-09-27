#ifdef __cplusplus
extern "C" {
#endif

#include <libgen.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "drivers/drv_system.h"
#include "params.h"

static uint64_t eeprom_version_;
static const char* eeprom_file = "/tmp/robin.params";

void drv_flash_init( void ) {
	eeprom_version_ = strtoll( EEPROM_CONF_VERSION_STR, NULL, 16 );
}

static bool validEEPROM( uint8_t* buffer ) {
	const params_t* temp = (const params_t*)buffer;

	// check version number
	if ( eeprom_version_ != temp->version ) {
		system_debug_print( "[PARAM] EEPROM not valid (version)" );
		return false;
	}

	// check size and magic numbers
	if ( temp->size != sizeof( params_t ) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF ) {
		system_debug_print( "[PARAM] EEPROM not valid (size/magic)" );
		return false;
	}

	// verify integrity of temporary copy
	const uint8_t* p;
	uint8_t chk = 0;

	for ( p = (const uint8_t*)temp;
		  p < ( (const uint8_t*)temp + sizeof( params_t ) ); p++ )
		chk ^= *p;

	// checksum failed
	if ( chk != 0 ) {
		system_debug_print( "[PARAM] EEPROM not valid (checksum)" );
		return false;
	}

	// looks good, let's roll!
	return true;
}

bool drv_flash_read( void ) {
	bool success = false;

	uint8_t buffer[sizeof( params_t )];
	FILE* fp;
	fp = fopen( eeprom_file, "rb" );

	if ( fp != NULL ) {
		int num = fread( buffer, 1, sizeof( params_t ), fp );

		if ( num == sizeof( params_t ) ) {
			if ( validEEPROM( buffer ) ) {
				memcpy( &_params, buffer, sizeof( params_t ) );
				success = true;
			} else {
				system_debug_print( "[PARAM] Read parameters not valid" );
			}
		} else {
			system_debug_print( "[PARAM] File not read correctly" );
		}

		fclose( fp );
	} else {
		system_debug_print( "[PARAM] Unable to read parameter file" );
	}

	return success;
}

bool drv_flash_write( void ) {
	bool success = false;

	// Prepare checksum/version constants
	_params.version = eeprom_version_;
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

	// Write to file
	FILE* fp;
	fp = fopen( eeprom_file, "wb" );

	if ( fp != NULL ) {
		uint8_t buffer[sizeof( params_t )];
		memcpy( buffer, &_params, sizeof( params_t ) );

		int num = fwrite( buffer, 1, sizeof( params_t ), fp );

		if ( num == sizeof( params_t ) ) {
			success = true;
			system_debug_print( "[PARAM] Parameter file written successfully: %s", eeprom_file );
		} else {
			system_debug_print( "[PARAM] File not written correctly" );
		}

		fclose( fp );
	} else {
		system_debug_print( "[PARAM] Unable to write to parameter file: %s", eeprom_file );
	}

	return success;
}

#ifdef __cplusplus
}
#endif

