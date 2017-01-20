#pragma once

//This will contain functions and variables to keep track of critical system status'
//as well as making sure that if a failure occurs, it is handled correctly

//Used to initialize all of the structs and timeouts
void init_safety(void);

//Used to initialize all of the structs and timeouts
void check_safety_monitor(void);
//TODO: Check MAV_MODE_FLAG

typedef struct {
	bool status;
	int32_t last_read;
} timeout_status_t;

typedef struct {
	safety_data_status_t heartbeat;
	safety_data_status_t offboard_control;
} mavlink_stream_status_t;

typedef struct {
	safety_data_status_t imu;
	safety_data_status_t mag;
	safety_data_status_t sonar;
} sensor_status_t;

//General UAV health status
typedef enum {
	SYSTEM_HEALTH_PERFECT,	//Operation is fine
	SYSTEM_HEALTH_WARNING,	//Send status warnings to the GCS
	SYSTEM_HEALTH_FAILSAFE,	//Go into failsafe mode, land immidiately
	SYSTEM_HEALTH_CRITICAL	//Halt all operations as quickly as possible
} safety_health_t;

//List of failures
//True means system is OK
typedef struct {
	uint32_t health;
	bool parameters;
	bool arm_status;
	mavlink_status mavlink;
	sensor_status_t sensors
} system_t;

extern system_t _system_status;
