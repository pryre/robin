#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <mavlink_system.h>
#include <mavlink/common/common.h>
#include <stdbool.h>
#include <stdint.h>

#include "param_generator/param_gen.h"
#include "fix16.h"

//This needs to be 1 less, as we need to allow space for '\n'
//MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1
#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

// type definitions
typedef struct {
	uint64_t version;
	uint16_t size;
	uint8_t magic_be;                       // magic number, should be 0xBE

	uint32_t values[PARAMS_COUNT];
	MAV_PARAM_TYPE types[PARAMS_COUNT];

	uint8_t magic_ef;                       // magic number, should be 0xEF
	uint8_t chk;                            // XOR checksum
} params_t;

// global variable declarations
extern params_t _params;

// function declarations
void set_param_defaults(void);
void init_param_uint(param_id_t id, const uint32_t value);
void init_param_int(param_id_t id, const int32_t value);
void init_param_fix16(param_id_t id, const fix16_t value);

bool read_params(void);
bool write_params(void);

MAV_PARAM_TYPE get_param_type(param_id_t id);
void get_param_name(param_id_t id, char *name);
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

uint32_t get_param_uint(param_id_t id);
int32_t get_param_int(param_id_t id);
fix16_t get_param_fix16(param_id_t id);

bool set_param_uint(param_id_t id, uint32_t value);
bool set_param_int(param_id_t id, int32_t value);
bool set_param_fix16(param_id_t id, fix16_t value);

bool set_param_by_name_uint(const char name[PARAMS_NAME_LENGTH], uint32_t value);
bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value);
bool set_param_by_name_fix16(const char name[PARAMS_NAME_LENGTH], fix16_t value);

#ifdef __cplusplus
}
#endif
