#pragma once

#include <stdint.h>

void system_init(void);

uint32_t system_micros(void);
void system_pause_ms(uint32_t ms);

void system_reset(void);
void system_bootloader(void);
