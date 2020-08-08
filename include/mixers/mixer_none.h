#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "io_type.h"

#include "fix16.h"
#include "fixextra.h"

const mixer_t mixer_none = {
	//XXX: i.e. mixer has not been set
	.mixer_ok = false,

	// Ch1, Ch2,  Ch3,  Ch4,  Ch5,  Ch6,  Ch7,  Ch8
	.output_type = {IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N,
	 IO_TYPE_N, IO_TYPE_N} // output_type

	// XXX: "None" mixer is a map with zeros size (invalid to use)
};

#ifdef __cplusplus
}
#endif
