#pragma once

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"
#include "breezystm32.h"
#include "sensors.h"
#include "params.h"
#include "safety.h"
#include "fix16.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

extern bool request_all_params;
extern int32_t param_to_send;

mavlink_system_t mavlink_system;
system_t _system_status;
sensor_readings_t _sensors;
params_t _params;

//Firmware identifier, first 8 bytes of current BreezySTM32 commit
static const uint8_t FW_HASH [8] = {0xb7, 0xa7, 0x59, 0x4e, 0xa7, 0xa2, 0x98, 0xb0};

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */

static inline void communications_init(void) {
	mavlink_system.sysid = get_param_int(PARAM_SYSTEM_ID); // System ID, 1-255
	mavlink_system.compid = get_param_int(PARAM_COMPONENT_ID); // Component/Subsystem ID, 1-255

	request_all_params = false;
	param_to_send = -1;
}

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	if (chan == MAVLINK_COMM_0) {
		serialWrite(Serial1, ch);
	} else if (chan == MAVLINK_COMM_1) {
		serialWrite(Serial1, ch);
	}
}

#include "mavlink/common/mavlink.h"

//==-- Sends
static inline void mavlink_stream_heartbeat(void) {
	uint8_t mav_base_mode = 0;

	if(_system_status.arm_status)
		mav_base_mode += MAV_MODE_FLAG_SAFETY_ARMED;

	//TODO: Document mode meanings
	if(_system_status.mode == SYSTEM_MODE_OFFBOARD) { //This is the "main" mode
			mav_base_mode += MAV_MODE_FLAG_GUIDED_ENABLED;
	} else if((_system_status.mode == SYSTEM_MODE_STANDBY) ||
				(_system_status.mode == SYSTEM_MODE_FAILSAFE)) {
			mav_base_mode += MAV_MODE_FLAG_STABILIZE_ENABLED;
	} //Other modes will show as 0, and should be cause for alarm if they are seen anyway, and thus unique

	//We don't use custom_mode
	//TODO: MAV_TYPE should be dynamically set
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, mav_base_mode, 0, _system_status.state);
	LED1_TOGGLE;
}

//TODO: Quite a lot here
static inline void mavlink_stream_sys_status(void) {
	uint32_t onboard_control_sensors_present = 0;
	uint32_t onboard_control_sensors_enabled = 0;
	uint32_t onboard_control_sensors_health = 0;
	uint16_t load = 0;
	uint16_t voltage_battery = 0;
	uint16_t current_battery = 0;
	uint8_t battery_remaining = 0;
	uint16_t drop_rate_comm = 0;
	uint16_t errors_comm = 0;
	uint16_t errors_count1 = 0;
	uint16_t errors_count2 = 0;
	uint16_t errors_count3 = 0;
	uint16_t errors_count4 = 0;

	load = _sensors.time.average_time/_sensors.time.counter;

	_sensors.time.counter = 0;
	_sensors.time.average_time = 0;
	_sensors.time.max = 0;
	_sensors.time.min = 1000;



	//TODO: This should be dynamic, probably in safety.h or sensors.h
	onboard_control_sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO &
										MAV_SYS_STATUS_SENSOR_3D_ACCEL &
										MAV_SYS_STATUS_SENSOR_3D_MAG &
										MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
	onboard_control_sensors_enabled = MAV_SYS_STATUS_SENSOR_3D_GYRO &
										MAV_SYS_STATUS_SENSOR_3D_ACCEL &
										MAV_SYS_STATUS_SENSOR_3D_MAG &
										MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
	onboard_control_sensors_health = MAV_SYS_STATUS_SENSOR_3D_GYRO &
										MAV_SYS_STATUS_SENSOR_3D_ACCEL &
										MAV_SYS_STATUS_SENSOR_3D_MAG &
										MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;


	//TODO: The rest of the sensors

	//mavlink_msg_sys_status_send(MAVLINK_COMM_0, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
	mavlink_msg_sys_status_send(MAVLINK_COMM_0,
								onboard_control_sensors_present,
								onboard_control_sensors_enabled,
								onboard_control_sensors_health,
								load,
								voltage_battery,
								current_battery,
								battery_remaining,
								drop_rate_comm,
								errors_comm,
								errors_count1,
								errors_count2,
								errors_count3,
								errors_count4);
}

//==-- Sends the latest IMU reading
//TODO: neaten, and make a proper define for the updated_data field
static inline void mavlink_stream_highres_imu(void) {
	mavlink_msg_highres_imu_send(MAVLINK_COMM_0,
									_sensors.imu.time,
									fix16_to_float(_sensors.imu.accel.x),
									fix16_to_float(_sensors.imu.accel.y),
									fix16_to_float(_sensors.imu.accel.z),
									fix16_to_float(_sensors.imu.gyro.x),
									fix16_to_float(_sensors.imu.gyro.y),
									fix16_to_float(_sensors.imu.gyro.z),
									0, 0, 0,
									0, 0, 0,
									0,
									((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)) );
}

//==-- Sends the autopilot version details
static inline void mavlink_stream_autopilot_version(void) {
	const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT +
									MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET +
									MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET +
									MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;

	const uint8_t blank_array[8] = {0,0,0,0,0,0,0,0};

	mavlink_msg_autopilot_version_send(MAVLINK_COMM_0,
									capabilities,
									get_param_int(PARAM_VERSION_SOFTWARE),
									0,
									get_param_int(PARAM_VERSION_FIRMWARE),
									get_param_int(PARAM_BOARD_REVISION),
									blank_array,	//TODO: This should probably be a commit as well
									blank_array,
									FW_HASH,
									0x10c4,	//TODO: This is the serial vendor and product ID, should be dynamic?
									0xea60,
									U_ID_0);
}

//==-- Sends the requested parameter
static inline void mavlink_stream_param_value(uint32_t index) {
	char param_name[16];
	uint8_t param_type = 0;

	union {
		float f;
		uint32_t i;
	} u;	//The bytes are translated to the right unit on receiving, but need to be sent as a "float"

	if(_params.types[index] == PARAM_TYPE_INT32) {
		param_type = MAV_PARAM_TYPE_UINT32;

		u.i = get_param_int(index);

	} else if (_params.types[index] == PARAM_TYPE_FIX16) {
		param_type = MAV_PARAM_TYPE_REAL32;

		u.f = fix16_to_float(get_param_fix16(index));
	}

	memcpy(param_name, _params.names[index], MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

	mavlink_msg_param_value_send(MAVLINK_COMM_0,
									param_name,		//String of name
									u.f,			//Value (always as float)
									param_type,		//From MAV_PARAM_TYPE
									PARAMS_COUNT,	//Total number of parameters
									index);
}

//==-- List of supported mavlink messages
	//o Optional
	//- Needed
	//+ Done!

	//Receive
	//- command_int.h
	//- command_long.h
	//- heartbeat.h
	//- manual_setpoint.h
	//- message_interval.h
	//- param_set.h
	//+ param_request_list.h
	//+ param_request_read.h
	//- ping.h
	//- raw_imu.h
	//- raw_pressure.h
	//- set_attitude_target.h
	//- timesync.h
	//o att_pos_mocap.h
	//o power_status.h
	//o radio_status.h

	//Transmit
	//- attitude_quaternion.h
	//- attitude_target.h
	//- autopilot_version.h
	//- battery_status.h
	//- command_ack.h
	//- debug.h
	//- debug_vect.h
	//- distance_sensor.h
	//+ heartbeat.h
	//- named_value_float.h
	//- named_value_int.h
	//+ param_value.h
	//- ping.h
	//- scaled_imu.h
	//- scaled_pressure.h
	//- statustext.h
	//+ sys_status.h
	//- servo_output_raw.h
	//- timesync.h
	//o attitude.h
	//o attitude_quaternion_cov.h
	//o highres_imu.h
	//o optical_flow.h
	//o optical_flow_rad.h
