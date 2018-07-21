#pragma once

#include <mavlink/common/common.h>

typedef enum {
	PARAM_BOARD_REVISION,
	PARAM_VERSION_FIRMWARE,
	PARAM_VERSION_SOFTWARE,
	PARAM_BAUD_RATE_0,
	PARAM_BAUD_RATE_1,
	PARAM_WAIT_FOR_HEARTBEAT,
	PARAM_TIMESYNC_ALPHA,
	PARAM_SYSTEM_ID,
	PARAM_COMPONENT_ID,
	PARAM_STRICT_GCS_MATCH,
	PARAM_GCS_SYSTEM_ID,
	PARAM_GCS_COMPONENT_ID,
	PARAM_RELAXED_PARAM_SET,
	PARAM_STREAM_RATE_HEARTBEAT_0,
	PARAM_STREAM_RATE_SYS_STATUS_0,
	PARAM_STREAM_RATE_HIGHRES_IMU_0,
	PARAM_STREAM_RATE_ATTITUDE_0,
	PARAM_STREAM_RATE_ATTITUDE_QUATERNION_0,
	PARAM_STREAM_RATE_ATTITUDE_TARGET_0,
	PARAM_STREAM_RATE_RC_CHANNELS_RAW_0,
	PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_0,
	PARAM_STREAM_RATE_TIMESYNC_0,
	PARAM_STREAM_RATE_BATTERY_STATUS_0,
	PARAM_STREAM_RATE_LOW_PRIORITY_0,
	PARAM_STREAM_RATE_HEARTBEAT_1,
	PARAM_STREAM_RATE_SYS_STATUS_1,
	PARAM_STREAM_RATE_HIGHRES_IMU_1,
	PARAM_STREAM_RATE_ATTITUDE_1,
	PARAM_STREAM_RATE_ATTITUDE_QUATERNION_1,
	PARAM_STREAM_RATE_ATTITUDE_TARGET_1,
	PARAM_STREAM_RATE_RC_CHANNELS_RAW_1,
	PARAM_STREAM_RATE_SERVO_OUTPUT_RAW_1,
	PARAM_STREAM_RATE_TIMESYNC_1,
	PARAM_STREAM_RATE_BATTERY_STATUS_1,
	PARAM_STREAM_RATE_LOW_PRIORITY_1,
	PARAM_SENSOR_IMU_CBRK,
	PARAM_SENSOR_SAFETY_CBRK,
	PARAM_SENSOR_MAG_UPDATE_RATE,
	PARAM_SENSOR_BARO_UPDATE_RATE,
	PARAM_SENSOR_SONAR_UPDATE_RATE,
	PARAM_SENSOR_IMU_STRM_COUNT,
	PARAM_SENSOR_BARO_STRM_COUNT,
	PARAM_SENSOR_SONAR_STRM_COUNT,
	PARAM_SENSOR_RC_INPUT_STRM_COUNT,
	PARAM_SENSOR_EXT_POSE_STRM_COUNT,
	PARAM_SENSOR_MAG_STRM_COUNT,
	PARAM_SENSOR_OFFB_HRBT_STRM_COUNT,
	PARAM_SENSOR_OFFB_CTRL_STRM_COUNT,
	PARAM_SENSOR_PWM_CTRL_STRM_COUNT,
	PARAM_SENSOR_IMU_TIMEOUT,
	PARAM_SENSOR_MAG_TIMEOUT,
	PARAM_SENSOR_BARO_TIMEOUT,
	PARAM_SENSOR_SONAR_TIMEOUT,
	PARAM_SENSOR_RC_INPUT_TIMEOUT,
	PARAM_SENSOR_EXT_POSE_TIMEOUT,
	PARAM_SENSOR_OFFB_HRBT_TIMEOUT,
	PARAM_SENSOR_OFFB_CTRL_TIMEOUT,
	PARAM_SENSOR_PWM_CTRL_TIMEOUT,
	PARAM_CAL_IMU_PASSES,
	PARAM_INIT_TIME,
	PARAM_EST_USE_ACC_COR,
	PARAM_EST_USE_MAT_EXP,
	PARAM_EST_USE_QUAD_INT,
	PARAM_EST_USE_ADPT_BIAS,
	PARAM_EST_USE_LEVEL_HORIZON,
	PARAM_EST_LEVEL_HORIZON_W,
	PARAM_EST_LEVEL_HORIZON_X,
	PARAM_EST_LEVEL_HORIZON_Y,
	PARAM_EST_LEVEL_HORIZON_Z,
	PARAM_EST_FILTER_KP,
	PARAM_EST_FILTER_KI,
	PARAM_GYRO_ALPHA,
	PARAM_ACC_ALPHA,
	PARAM_FUSE_EXT_HDG_W,
	PARAM_FUSE_MAG_HDG_W,
	PARAM_GYRO_X_BIAS,
	PARAM_GYRO_Y_BIAS,
	PARAM_GYRO_Z_BIAS,
	PARAM_ACC_X_BIAS,
	PARAM_ACC_Y_BIAS,
	PARAM_ACC_Z_BIAS,
	PARAM_ACC_X_SCALE_POS,
	PARAM_ACC_X_SCALE_NEG,
	PARAM_ACC_Y_SCALE_POS,
	PARAM_ACC_Y_SCALE_NEG,
	PARAM_ACC_Z_SCALE_POS,
	PARAM_ACC_Z_SCALE_NEG,
	PARAM_ACC_X_TEMP_COMP,
	PARAM_ACC_Y_TEMP_COMP,
	PARAM_ACC_Z_TEMP_COMP,
	PARAM_RATE_CONTROL,
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
	PARAM_BATTERY_TYPE,
	PARAM_BATTERY_FUNCTION,
	PARAM_BATTERY_CELL_NUM,
	PARAM_BATTERY_CELL_MIN,
	PARAM_BATTERY_CELL_MAX,
	PARAM_BATTERY_READING_FILTER,
	PARAM_BATTERY_DIVIDER,
	PARAM_BATTERY_CHARGE_STATE_LOW,
	PARAM_BATTERY_CHARGE_STATE_CRITICAL,
	PARAM_BATTERY_CHARGE_STATE_EMERGENCY,
	PARAM_MOTOR_PWM_SEND_RATE,
	PARAM_MOTOR_PWM_IDLE,
	PARAM_MOTOR_PWM_MIN,
	PARAM_MOTOR_PWM_MAX,
	PARAM_RC_MAP_ROLL,
	PARAM_RC_MAP_PITCH,
	PARAM_RC_MAP_YAW,
	PARAM_RC_MAP_THROTTLE,
	PARAM_RC_MAP_MODE_SW,
	PARAM_RC_MAP_PASSTHROUGH_AUX1,
	PARAM_RC_MAP_PASSTHROUGH_AUX2,
	PARAM_RC_MAP_PASSTHROUGH_AUX3,
	PARAM_RC_MAP_PASSTHROUGH_AUX4,
	PARAM_RC_MODE_DEFAULT,
	PARAM_RC_MODE_PWM_RANGE,
	PARAM_RC_MODE_PWM_STAB,
	PARAM_RC_MODE_PWM_ACRO,
	PARAM_RC_MODE_PWM_OFFBOARD,
	PARAM_RC_ARM_TIMER,
	PARAM_RC1_MIN,
	PARAM_RC1_MID,
	PARAM_RC1_MAX,
	PARAM_RC1_REV,
	PARAM_RC1_DZ,
	PARAM_RC2_MIN,
	PARAM_RC2_MID,
	PARAM_RC2_MAX,
	PARAM_RC2_REV,
	PARAM_RC2_DZ,
	PARAM_RC3_MIN,
	PARAM_RC3_MID,
	PARAM_RC3_MAX,
	PARAM_RC3_REV,
	PARAM_RC3_DZ,
	PARAM_RC4_MIN,
	PARAM_RC4_MID,
	PARAM_RC4_MAX,
	PARAM_RC4_REV,
	PARAM_RC4_DZ,
	PARAM_RC5_MIN,
	PARAM_RC5_MID,
	PARAM_RC5_MAX,
	PARAM_RC5_REV,
	PARAM_RC5_DZ,
	PARAM_RC6_MIN,
	PARAM_RC6_MID,
	PARAM_RC6_MAX,
	PARAM_RC6_REV,
	PARAM_RC6_DZ,
	PARAM_RC7_MIN,
	PARAM_RC7_MID,
	PARAM_RC7_MAX,
	PARAM_RC7_REV,
	PARAM_RC7_DZ,
	PARAM_RC8_MIN,
	PARAM_RC8_MID,
	PARAM_RC8_MAX,
	PARAM_RC8_REV,
	PARAM_RC8_DZ,
	PARAM_DO_ESC_CAL,
	PARAM_FAILSAFE_THROTTLE,
	PARAM_THROTTLE_TIMEOUT,
	PARAM_MAV_TYPE,
	PARAM_MIXER,
	PARAM_RESET_PARAMS,
	PARAMS_COUNT
} param_id_t;

extern const char _param_names[PARAMS_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];

void params_init(void);
void param_change_callback(param_id_t id);
