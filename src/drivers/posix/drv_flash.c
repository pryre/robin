#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

static uint64_t _eeprom_version;

void drv_flash_init( void ) {
	_eeprom_version = strtoll(EEPROM_CONF_VERSION_STR, NULL, 16);
}

bool drv_flash_read( void ) {
	return false;
}

bool drv_flash_write( void ) {
	return true;
}
