#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/drv_system.h"
#include "params.h"

params_t _params;

static uint64_t _eeprom_version;

void drv_flash_init( void ) {
	_eeprom_version = strtoll(EEPROM_CONF_VERSION_STR, NULL, 16);
}

static bool validEEPROM(uint8_t *buffer) {
	const params_t *temp = (const params_t *)buffer;
	const uint8_t *p;
	uint8_t chk = 0;

	// check version number
	if (_eeprom_version != temp->version) {
		system_debug_print("[PARAM] EEPROM not valid (version)");
		return false;
	}

	// check size and magic numbers
	if (temp->size != sizeof(params_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF) {
		system_debug_print("[PARAM] EEPROM not valid (size/magic)");
		return false;
	}

	// verify integrity of temporary copy
	for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(params_t)); p++)
		chk ^= *p;

	// checksum failed
	if (chk != 0) {
		system_debug_print("[PARAM] EEPROM not valid (checksum)");
		return false;
	}
	// looks good, let's roll!
	return true;
}

bool drv_flash_read( void ) {
	bool success = false;

	uint8_t buffer[sizeof(params_t)];
	FILE *fp;

	fp = fopen("robin_posix_eeprom.bin","rb");

	if(fp != NULL) {
		int num = fread(buffer, 1, sizeof(params_t), fp);

		if( num == sizeof(params_t) ) {
			if( validEEPROM(buffer) ) {
				memcpy( &_params, buffer, sizeof(params_t) );
				success = true;
			} else {
				system_debug_print("[PARAM] Read parameters not valid");
			}
		} else {
			system_debug_print("[PARAM] File not read correctly");
		}

		fclose(fp);
	} else {
		system_debug_print("[PARAM] Unable to read parameter file");
	}

	return success;
}

bool drv_flash_write( void ) {
	bool success = false;

	// Recalculate checksum before writing
	uint8_t chk = 0;
	const uint8_t *p;

	// Prepare checksum/version constants
	_params.version = _eeprom_version;
	_params.size = sizeof(params_t);
	_params.magic_be = 0xBE;
	_params.magic_ef = 0xEF;

	for (p = (const uint8_t *)&_params; p < ((const uint8_t *)&_params + sizeof(params_t)); p++)
		chk ^= *p;

	_params.chk = chk;

	// Write to file
	FILE *fp;

	fp = fopen("robin_posix_eeprom.bin","wb");

	if(fp != NULL) {
		uint8_t buffer[sizeof(params_t)];
		memcpy( buffer, &_params, sizeof(params_t) );

		int num = fwrite(buffer, 1, sizeof(params_t), fp);

		if( num == sizeof(params_t) ) {
			success = true;
		} else {
			system_debug_print("[PARAM] File not written correctly");
		}

		fclose(fp);
	} else {
		system_debug_print("[PARAM] Unable to write to parameter file");
	}

	return success;
}
