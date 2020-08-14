#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "io_type.h"

#include "fix16.h"
#include "fixextra.h"

#define nackT _fc_0_15 //Robin Gazebo Default ~= [la: 0.225; m = 4; theta_b: pi/2; Tmax: 1.0kg]
#define nackD _fc_0_3

const mixer_t mixer_quadrotor_x_nac = {
	.mixer_ok = true,

	.output_type = {IO_TYPE_OM, IO_TYPE_OM, IO_TYPE_OM, IO_TYPE_OM, IO_TYPE_N, IO_TYPE_N,
	 IO_TYPE_N, IO_TYPE_N}, // output_type

	//wx, wy, wz, vz

	.map = {.rows = 4, .columns = 4, .errors = 0,
			.data = {{-nackT, nackT, nackD, _fc_1},
					 { nackT,-nackT, nackD, _fc_1},
					 { nackT, nackT,-nackD, _fc_1},
					 {-nackT,-nackT,-nackD, _fc_1}}}
	/*
	{_fc_1, _fc_1, _fc_1, _fc_1, 0, 0, 0, 0},   // F Mix
	{-_fc_1, _fc_1, _fc_1, -_fc_1, 0, 0, 0, 0}, // X Mix
	{_fc_1, -_fc_1, _fc_1, -_fc_1, 0, 0, 0, 0}, // Y Mix
	{_fc_1, _fc_1, -_fc_1, -_fc_1, 0, 0, 0, 0}  // Z Mix
	*/
};

#ifdef __cplusplus
}
#endif
