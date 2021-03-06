#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRV_PPM_MAX_INPUTS 8

bool drv_ppm_init( void );
bool drv_ppm_ready( void );
bool drv_ppm_read_frame( uint16_t *frame );	//XXX: Frame should be of size DRV_PWM_MAX_INPUTS
bool drv_ppm_read_rssi( uint16_t *pulse );	//XXX: Pulse should be of a single variable

#ifdef __cplusplus
}
#endif
