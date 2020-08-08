#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "io_type.h"

#include "fix16.h"
#include "fixextra.h"

// XXX: Mixer free is the same as "none" but will allow for full actuator
// control
const mixer_t mixer_free = {
	.mixer_ok = true,

	// Ch1, Ch2,  Ch3,  Ch4,  Ch5,  Ch6,  Ch7,  Ch8
	.output_type = {IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N,
	 IO_TYPE_N, IO_TYPE_N} // output_type

	// XXX: Free mixer is a map with zeros size (invalid to use)
};

#ifdef __cplusplus
}
#endif
