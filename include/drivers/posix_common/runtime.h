#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void posix_soft_reset( void );
void posix_get_sim_time( uint32_t *secs, uint32_t *nsecs );

#ifdef __cplusplus
}
#endif
