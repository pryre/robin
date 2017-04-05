#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "fix16.h"

//#include "mavlink.h"

//This needs to be 1 less, as we need to allow space for '\n'
//MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1
#define PARAMS_NAME_LENGTH 17

typedef enum {
	//==-- System
	PARAM_BOARD_REVISION,
	PARAM_VERSION_FIRMWARE,
	PARAM_VERSION_SOFTWARE,
	PARAM_BAUD_RATE,

	//==-- Mavlink
	PARAM_SYSTEM_ID,
	PARAM_COMPONENT_ID,
	PARAM_STREAM_HEARTBEAT_RATE,

	PARAM_STREAM_ATTITUDE_RATE,
	PARAM_STREAM_IMU_RATE,
	PARAM_STREAM_MAG_RATE,
	PARAM_STREAM_BARO_RATE,
	PARAM_STREAM_SONAR_RATE,
	PARAM_STREAM_SERVO_OUTPUT_RAW_RATE,

	//==-- Sensors
	PARAM_SENSOR_DIFF_PRESS_UPDATE,
	PARAM_SENSOR_BARO_UPDATE,
	PARAM_SENSOR_SONAR_UPDATE,
	PARAM_SENSOR_MAG_UPDATE,
	PARAM_SENSOR_OFFB_CTRL_UPDATE,

	PARAM_SENSOR_DIFF_PRESS_TIMEOUT,
	PARAM_SENSOR_BARO_TIMEOUT,
	PARAM_SENSOR_SONAR_TIMEOUT,
	PARAM_SENSOR_MAG_TIMEOUT,
	PARAM_SENSOR_OFFB_CTRL_TIMEOUT,

	//==-- Estimator
	PARAM_INIT_TIME,
	PARAM_FILTER_KP,
	PARAM_FILTER_KI,
	PARAM_GYRO_ALPHA,
	PARAM_ACC_ALPHA,
	PARAM_STREAM_ADJUSTED_GYRO,
	PARAM_GYRO_X_BIAS,
	PARAM_GYRO_Y_BIAS,
	PARAM_GYRO_Z_BIAS,
	PARAM_ACC_X_BIAS,
	PARAM_ACC_Y_BIAS,
	PARAM_ACC_Z_BIAS,
	PARAM_ACC_X_TEMP_COMP,
	PARAM_ACC_Y_TEMP_COMP,
	PARAM_ACC_Z_TEMP_COMP,

	//==-- Control
	PARAM_PID_ROLL_RATE_P,
	PARAM_PID_ROLL_RATE_I,
	PARAM_PID_ROLL_RATE_D,
	PARAM_MAX_ROLL_RATE,

	PARAM_PID_PITCH_RATE_P,
	PARAM_PID_PITCH_RATE_I,
	PARAM_PID_PITCH_RATE_D,
	PARAM_MAX_PITCH_RATE,

	PARAM_PID_YAW_RATE_P,
	PARAM_PID_YAW_RATE_I,
	PARAM_PID_YAW_RATE_D,
	PARAM_MAX_YAW_RATE,

	PARAM_PID_ROLL_ANGLE_P,
	PARAM_MAX_ROLL_ANGLE,

	PARAM_PID_PITCH_ANGLE_P,
	PARAM_MAX_PITCH_ANGLE,

	PARAM_PID_YAW_ANGLE_P,

	PARAM_MAX_COMMAND,
	PARAM_PID_TAU,

	//==-- Output
	PARAM_MOTOR_PWM_SEND_RATE,
	PARAM_MOTOR_PWM_IDLE,			//TODO: Note that this should be set to 0 to disable
	PARAM_MOTOR_PWM_MIN,
	PARAM_MOTOR_PWM_MAX,
	PARAM_MIXER,

	//==-- Number of Parameters
	PARAMS_COUNT
} param_id_t;

typedef enum {
	PARAM_TYPE_INT32,
	PARAM_TYPE_FIX16,
	PARAM_TYPE_INVALID
} param_type_t;

// type definitions
typedef struct {
	uint8_t version;
	uint16_t size;
	uint8_t magic_be;                       // magic number, should be 0xBE

	int32_t values[PARAMS_COUNT];
	char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
	param_type_t types[PARAMS_COUNT];

	uint8_t magic_ef;                       // magic number, should be 0xEF
	uint8_t chk;                            // XOR checksum
} params_t;

// global variable declarations
extern params_t _params;

// function declarations

/**
 * @brief Initialize parameter values
 */
void init_params(void);

/**
 * @brief Set all parameters to default values
 */
void set_param_defaults(void);

/**
 * @brief Read parameter values from non-volatile memory
 * @return True if successful, false otherwise
 */
bool read_params(void);

/**
 * @brief Write current parameter values to non-volatile memory
 * @return True if successful, false otherwise
 */
bool write_params(void);

/**
 * @brief Callback for executing actions that need to be taken when a parameter value changes
 * @param id The ID of the parameter that was changed
 */
void param_change_callback(param_id_t id);

/**
 * @brief Gets the id of a parameter from its name
 * @param name The name of the parameter
 * @return The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
 */
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

/**
 * @brief Get the value of an integer parameter by id
 * @param id The ID of the parameter
 * @return The value of the parameter
 */
int32_t get_param_int(param_id_t id);

/**
 * @brief Get the value of a fixed point parameter by id
 * @param id The ID of the parameter
 * @return The value of the parameter
 */
fix16_t get_param_fix16(param_id_t id);

/**
 * @brief Get the name of a parameter
 * @param id The ID of the parameter
 * @return The name of the parameter
 */
char * get_param_name(param_id_t id);

/**
 * @brief Get the type of a parameter
 * @param id The ID of the parameter
 * @return The type of the parameter
 */
param_type_t get_param_type(param_id_t id);

/**
 * @brief Sets the value of a parameter by ID and calls the parameter change callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_int(param_id_t id, int32_t value);

/**
 * @brief Sets the value of a fixed point parameter by ID and calls the parameter callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return  True if a parameter was changed, false otherwise
 */
bool set_param_fix16(param_id_t id, fix16_t value);

/**
 * @brief Sets the value of a parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value);

/**
 * @brief Sets the value of a fixed point parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name_fix16(const char name[PARAMS_NAME_LENGTH], fix16_t value);
