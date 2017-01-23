#pragma once

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"
#include "breezystm32.h"
#include "sensors.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

mavlink_system_t mavlink_system;

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */

static inline void communications_init(void) {
	mavlink_system.sysid = 1; // System ID, 1-255
	mavlink_system.compid = 1; // Component/Subsystem ID, 1-255
}

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	if (chan == MAVLINK_COMM_0) {
		serialWrite(Serial1, ch);
	} else if (chan == MAVLINK_COMM_1) {
		serialWrite(Serial1, ch);
	}
}

#include "mavlink/common/mavlink.h"

static inline void mavlink_stream_heartbeat(void) {
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_STANDBY);
	LED1_TOGGLE;
}

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

	load = _sensor_time.average_time/_sensor_time.counter;

	_sensor_time.counter = 0;
	_sensor_time.average_time = 0;
	_sensor_time.max = 0;
	_sensor_time.min = 1000;



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

/*
static inline void mavlink_stream_scaled_imu(void) {


mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,
	time_us,
	(uint16_t)(accel_data[0]*accel_scale*1000.0f),
	(uint16_t)(accel_data[1]*accel_scale*1000.0f),
	(uint16_t)(accel_data[2]*accel_scale*1000.0f),
	(uint16_t)(gyro_data[0]*MPU_GYRO_SCALE*1000.0f),
	(uint16_t)(gyro_data[1]*MPU_GYRO_SCALE*1000.0f),
	(uint16_t)(gyro_data[2]*MPU_GYRO_SCALE*1000.0f),
	0, 0, 0);
}
*/

//==-- List of supported mavlink messages
	//x Ignored
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
	//- heartbeat.h
	//- named_value_float.h
	//- named_value_int.h
	//- param_request_list.h
	//- param_request_read.h
	//- param_value.h
	//- ping.h
	//- scaled_imu.h
	//- scaled_pressure.h
	//- statustext.h
	//- sys_status.h
	//- servo_output_raw.h
	//- timesync.h
	//o attitude.h
	//o attitude_quaternion_cov.h
	//o highres_imu.h
	//o optical_flow.h
	//o optical_flow_rad.h
