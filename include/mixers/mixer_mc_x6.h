#pragma once

#include "mixer.h"

#include "fix16.h"
#include "fixextra.h"

const mixer_t mixer_hexarotor_x = {
	{MT_M, MT_M, MT_M, MT_M, MT_M, MT_M, MT_NONE, MT_NONE}, // output_type

	{ _fc_1, _fc_1,   _fc_1,   _fc_1,    _fc_1,   _fc_1, 0, 0}, // F Mix
	{-_fc_1, _fc_1, _fc_0_5,-_fc_0_5, -_fc_0_5, _fc_0_5, 0, 0}, // X Mix
	{    0,    0,     _fc_1,  -_fc_1,    _fc_1,  -_fc_1, 0, 0}, // Y Mix
	{-_fc_1, _fc_1,  -_fc_1,   _fc_1,    _fc_1,  -_fc_1, 0, 0}  // Z Mix
};

