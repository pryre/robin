#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
This file contains the primary runtime loops for the system
The sofware itself can be implemented similarly to:

#include <run.h>
#include <stdbool.h>

int main(void) {
	setup();

	while (true)
		loop();
}
*/

void setup( void );

void loop( void );

#ifdef __cplusplus
}
#endif
