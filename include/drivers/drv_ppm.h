#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRV_PPM_MAX_INPUTS 8

#define DRV_PPM_MIN 900
#define DRV_PPM_MAX 2100
#define DRV_PPM_SYNC_MIN 2700
#define DRV_PPM_SYNC_MAX 15000

bool drv_ppm_init( void );
bool drv_ppm_ready( void );
bool drv_ppm_read_frame( uint16_t *frame );	//XXX: Frame should be of size DRV_PWM_MAX_INPUTS

#ifdef __cplusplus
}
#endif
