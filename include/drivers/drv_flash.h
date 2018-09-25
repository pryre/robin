#pragma once

#include <stdbool.h>
#include <stdint.h>

void drv_flash_init( void );
bool drv_flash_read( void );
bool drv_flash_write( void );
