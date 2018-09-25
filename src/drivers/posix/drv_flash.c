#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "params.h"

params_t _params;

static uint64_t _eeprom_version;

void drv_flash_init( void ) {
	_eeprom_version = strtoll(EEPROM_CONF_VERSION_STR, NULL, 16);
}

bool drv_flash_read( void ) {
	bool success = false;

	unsigned char buffer[sizeof(params_t)];
	FILE *fp;

	fp = fopen("robin_posix_eeprom.bin","rb");  // r for read, b for binary

	if(fp != NULL) {
		int num = fread(buffer, sizeof(params_t), sizeof(unsigned char), fp); // read 10 bytes to our buffer
		if( num == sizeof(params_t) ) {
			memcpy( &_params, buffer, sizeof(params_t) );
			success = true;
		}

		fclose(fp);
	}

	return success;
}

bool drv_flash_write( void ) {
	bool success = false;

	FILE *fp;

	fp = fopen("robin_posix_eeprom.bin","wb");  // r for read, b for binary

	if(fp != NULL) {
		unsigned char buffer[sizeof(params_t)];
		memcpy( buffer, &_params, sizeof(params_t) );

		int num = fwrite(buffer, sizeof(unsigned char), sizeof(params_t), fp); // read 10 bytes to our buffer

		if( num == sizeof(params_t) ) {
			success = true;
		}

		fclose(fp);
	}

	return success;
}
