#pragma once

#include "mixer.h"

#include "fix16.h"
#include "fixextra.h"

//TODO: Double check
static mixer_t mixer_quadrotor_plus = {
	{M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

	{ _fc_1, _fc_1, _fc_1, _fc_1, 0, 0, 0, 0}, // F Mix
	{-_fc_1, _fc_1, 0,    0,    0, 0, 0, 0}, // X Mix
	{ 0,    0,   -_fc_1, _fc_1, 0, 0, 0, 0}, // Y Mix
	{ _fc_1, _fc_1,-_fc_1,-_fc_1, 0, 0, 0, 0}  // Z Mix
};

