#pragma once

#include "mixer.h"

#include "fix16.h"
#include "fixextra.h"

const mixer_t mixer_plane_standard = {
	{MT_M, MT_S, MT_S, MT_S, MT_NONE, MT_NONE, MT_NONE, MT_NONE}, // output_type

	{ _fc_1, 0, 0, 0, 0, 0, 0, 0}, // F Mix
	{ 0, _fc_1, 0, 0, 0, 0, 0, 0}, // X Mix
	{ 0, 0, _fc_1, 0, 0, 0, 0, 0}, // Y Mix
	{ 0, 0, 0, _fc_1, 0, 0, 0, 0}  // Z Mix

	//TODO: Should double check this, but it would allow stabilize control with motor throughput and a servo out for ailerons, elevator and rudder
};
