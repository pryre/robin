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

//List of failures
//True means system is OK
typedef struct {
	uint8_t health;	//Set with safety_health_t	//TODO: Must be implemented

	//TODO: Make sure this is fully implemented
	//Note: Mirrors MAVLINK configuration
		//MAV_STATE_UNINIT:			System start
		//MAV_STATE_BOOT:			System booting/initializing
		//MAV_STATE_CALIBRATING:	System calibration active
		//MAV_STATE_STANDBY:		System ready, waiting for arming and/or command input
		//MAV_STATE_ACTIVE:			System fully armed (and may be flying) (Don't allow switch to without MAV_MODE_FLAG_SAFETY_ARMED set)
		//MAV_STATE_CRITICAL:		Something is giving errors, should be attempting graceful failsafe maneouvres
		//MAV_STATE_EMERGENCY:		System failure, should be attempting hard failsafe maneouvres, keep comms up, but should not allow recover without reboot
		//MAV_STATE_POWEROFF:		Shutting down (write EEPROM/SD card logs, alert GCS?)
	uint8_t state;	//Set with MAV_STATE		//TODO: Must be implemented

	//TODO: Make sure this is fully implemented
	//Note:
		//Boot: MAV_MODE_PREFLIGHT
		//Standby/Failsafe/Emergency: MAV_MODE_STABILIZE_DISARMED/MAV_MODE_STABILIZE_ARMED
		//Active: MAV_MODE_GUIDED_DISARMED/MAV_MODE_GUIDED_ARMED
	uint8_t mode;	//Set with MAV_MODE_FLAG	//TODO: Must be implemented
	bool parameters;
	bool arm_status;
	mavlink_stream_status_t mavlink;
	sensor_status_t sensors;
} system_t;

//TODO: This could almost be merged with mav_state
typedef enum {
	SYSTEM_OPERATION_RUN,
	SYSTEM_OPERATION_SHUTDOWN,
	SYSTEM_OPERATION_REBOOT,
	SYSTEM_OPERATION_REBOOT_BOOTLOADER
} system_operation_t;

extern system_t _system_status;
extern uint8_t _system_operation_control;
