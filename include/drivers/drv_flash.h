#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void drv_flash_init( void );
bool drv_flash_read( void );
bool drv_flash_write( void );

#ifdef __cplusplus
}
#endif
