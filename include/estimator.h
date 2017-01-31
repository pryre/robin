#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fix16.h"
#include "fixvector3d.h"

typedef struct {
  fix16_t p;
  fix16_t q;
  fix16_t r;
  fix16_t phi;
  fix16_t theta;
  fix16_t psi;
  fix16_t altitude;
} state_t;

extern state_t _state_estimator;	//_current_state
extern v3d _adaptive_gyro_bias;


// mat_exp <- greater accuracy, but adds ~90 us
// quadratic_integration <- some additional accuracy, adds ~20 us
// accelerometer correction <- if using angle mode, this is required, adds ~70 us
void estimator_init(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
void estimator_update(uint32_t now);

#ifdef __cplusplus
}
#endif
