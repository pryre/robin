#pragma once

//This will contain functions and variables to keep track of critical system status'
//as well as making sure that if a failure occurs, it is handled correctly

//Used to initialize all of the structs and timeouts
void init_safety(void);

//Used to initialize all of the structs and timeouts
void check_safety_monitor(void);

//General UAV health status
typedef enum {
	SYSTEM_HEALTH_PERFECT,	//Operation is fine
	SYSTEM_HEALTH_ERROR,	//Operation is fine
	SYSTEM_HEALTH_INVALID
} safety_sensor_health_t;

typedef struct {
	bool health;	//Set with safety_sensor_health_t
	int32_t last_read;
} timeout_status_t;

typedef struct {
	timeout_status_t heartbeat;
	timeout_status_t offboard_control;
} mavlink_stream_status_t;

typedef struct {
	timeout_status_t imu;
	timeout_status_t mag;
	timeout_status_t sonar;
} sensor_status_t;


//TODO: Integrate this properly in main()
typedef enum {
	SYSTEM_MODE_BOOT,		//Set as an initial value until boot completes
	SYSTEM_MODE_STANDBY,	//The default value for the mav while waiting and not commanded
	SYSTEM_MODE_OFFBOARD,	//The mav is receiving offboard commands and may be flying
	SYSTEM_MODE_FAILSAFE,	//Something has gone wrong, graceful failsafe
	SYSTEM_MODE_CRITICAL,	//Something has gone wrong, halt all operations
	SYSTEM_MODE_NUM
} safety_modes_t;

//List of failures
//True means system is OK
typedef struct {
	uint8_t health;	//Set with safety_health_t //TODO: Must be implemented
	uint8_t mode;	//Set with safety_modes_t //TODO: Must be implemented
	uint8_t state;	//Set with MAV_STATE //TODO: Must be implemented
	bool parameters;
	bool arm_status;
	mavlink_stream_status_t mavlink;
	sensor_status_t sensors;
} system_t;

typedef enum {
	SYSTEM_OPERATION_RUN,
	SYSTEM_OPERATION_REBOOT,
	SYSTEM_OPERATION_REBOOT_BOOTLOADER
} system_operation_t;

extern system_t _system_status;
extern uint8_t _system_operation_control;
