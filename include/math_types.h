#pragma once


typedef struct {
	float w;
	float x;
	float y;
	float z;
} quaternion_t;

typedef struct {
	float x;
	float y;
	float z;
} vector3_t;

static inline void euler_from_quat(quaternion_t *q, float *roll, float *pitch, float *yaw) {

}
