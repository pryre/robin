#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "io_type.h"

#include "fix16.h"
#include "fixextra.h"

const mixer_t mixer_plane_standard = {
	true,

	{IO_TYPE_OM, IO_TYPE_OS, IO_TYPE_OS, IO_TYPE_OS, IO_TYPE_N, IO_TYPE_N,
	 IO_TYPE_N, IO_TYPE_N}, // output_type

	{_fc_1, 0, 0, 0, 0, 0, 0, 0}, // F Mix
	{0, _fc_1, 0, 0, 0, 0, 0, 0}, // X Mix
	{0, 0, _fc_1, 0, 0, 0, 0, 0}, // Y Mix
	{0, 0, 0, _fc_1, 0, 0, 0, 0}  // Z Mix

	// TODO: Should double check this, but it would allow stabilize control with
	// motor throughput and a servo out for ailerons, elevator and rudder
};

#ifdef __cplusplus
}
#endif
