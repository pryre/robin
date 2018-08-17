#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "fixvector3d.h"
#include "fixquat.h"

typedef struct {
  fix16_t ax;		//X Acceleration
  fix16_t ay;		//Y Acceleration
  fix16_t az;		//Z Acceleration
  fix16_t p;		//Roll Rate
  fix16_t q;		//Pitch Rate
  fix16_t r;		//Yaw Rate
  qf16 attitude;	//Attitude Quaternion
} state_t;

extern state_t _state_estimator;	//Current state estimation
extern v3d _adaptive_gyro_bias;		//Adds in an adaptive measurements for gyro

//TODO: Redo these stats
// PARAM_EST_USE_ACC_COR <- if using angle mode, this is required, adds ~70 us
// PARAM_EST_USE_MAT_EXP <- greater accuracy, but adds ~90 us
// PARAM_EST_USE_QUAD_INT <- some additional accuracy, adds ~20 us
void estimator_init();
//void estimator_update(uint32_t now, v3d *accel, v3d *gyro);
void estimator_update_sensors(uint32_t now);
void estimator_update_hil(uint32_t now);
void reset_adaptive_gyro_bias();

void estimator_calc_lvl_horz(qf16* q_lh);

#ifdef __cplusplus
}
#endif
