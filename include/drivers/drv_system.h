#pragma once

#include <stdint.h>

void system_init(void);

uint32_t system_micros(void);
void system_pause_ms(uint32_t ms);

void system_reset(void);
void system_bootloader(void);

void system_debug_print( char *msg );

//XXX: Used in systems (such as posix) to not overload the CPU
void system_rate_limit(void);
