#pragma once

#include "io_type.h"

#include "fix16.h"
#include "fixextra.h"

//XXX: Mixer free is the same as "none" but will allow for full actuator control
const mixer_t mixer_free = {
	true,

	//Ch1, Ch2,  Ch3,  Ch4,  Ch5,  Ch6,  Ch7,  Ch8
	{IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N, IO_TYPE_N}, // output_type

	//Channels
	//1, 2, 3, 4, 5, 6, 7, 8
	{ 0, 0, 0, 0, 0, 0, 0, 0}, // F Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}, // X Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}, // Y Mix
	{ 0, 0, 0, 0, 0, 0, 0, 0}  // Z Mix
};
