#include "breezystm32.h"
#include "gpio.h"
#include "serial.h"
#include "serial_uart.h"

#include "mavlink_system.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "mavlink_transmit.h"

#include "fix16.h"
#include "fixextra.h"
#include "params.h"
#include "param_generator/param_gen.h"
#include "safety.h"
#include "sensors.h"
#include "estimator.h"
#include "controller.h"
#include "mixer.h"

#include <stdio.h>

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

mavlink_queue_t _lpq_port_0;
//XXX: mavlink_queue_t _lpq_port_1;

mavlink_system_t _mavlink_gcs;
mavlink_system_t mavlink_system;

system_status_t _system_status;
sensor_readings_t _sensors;
params_t _params;
const char _param_names[PARAMS_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
state_t _state_estimator;

command_input_t _control_input;
int32_t _pwm_output[8];

static uint8_t comms_open_status_ = 0;
static const uint8_t blank_array_[8] = {0,0,0,0,0,0,0,0};
static const uint8_t blank_array_uid_[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void communications_system_init(void) {
	mavlink_system.sysid = get_param_uint(PARAM_SYSTEM_ID); // System ID, 1-255
	mavlink_system.compid = get_param_uint(PARAM_COMPONENT_ID); // Component/Subsystem ID, 1-255

	_mavlink_gcs.sysid = get_param_uint(PARAM_GCS_SYSTEM_ID); // System ID, 1-255
	_mavlink_gcs.compid = get_param_uint(PARAM_GCS_COMPONENT_ID); // Component/Subsystem ID, 1-255

	_lpq_port_0.port = MAVLINK_COMM_0;
	_lpq_port_0.position = 0;
	_lpq_port_0.length = 0;
	_lpq_port_0.request_all_params = -1;
	_lpq_port_0.timer_warn_full = 0;

	//XXX: _lpq_port_1.port = MAVLINK_COMM_1;
	//XXX: _lpq_port_1.position = 0;
	//XXX: _lpq_port_1.length = 0;
	//XXX: _lpq_port_1.request_all_params = -1;
	//XXX: _lpq_port_1.timer_warn_full = 0;

	if( get_param_uint( PARAM_BAUD_RATE_0 ) > 0 ) {
		Serial1 = uartOpen( USART1, NULL, get_param_uint(PARAM_BAUD_RATE_0 ), MODE_RXTX, SERIAL_NOT_INVERTED );
		comm_set_open( COMM_CH_0 );
	}

	//XXX: if( get_param_uint( PARAM_BAUD_RATE_1 ) > 0 ) {
	//XXX: 	Serial2 = uartOpen( USART2, NULL, get_param_uint( PARAM_BAUD_RATE_1 ), MODE_RXTX, SERIAL_NOT_INVERTED );
	//XXX: 	comm_set_open( COMM_CH_1 );
	//XXX: }


	for(mavlink_stream_id_t i = 0; i < MAVLINK_STREAM_COUNT; i++)
		communication_calc_period_update(COMM_CH_0, i);

	//XXX: for(mavlink_stream_id_t i = 0; i < MAVLINK_STREAM_COUNT; i++)
	//XXX: 	communication_calc_period_update(COMM_CH_1, i);


	mavlink_set_proto_version(MAVLINK_COMM_0, 1);
	//XXX: mavlink_set_proto_version(MAVLINK_COMM_1, 1);
}

bool comm_is_open( uint8_t ch ) {
	return comms_open_status_ & ch;
}

void comm_set_open( uint8_t ch ) {
	comms_open_status_ |= ch;
}

void comm_set_closed( uint8_t ch ) {
	comms_open_status_ &= ~ch;
}

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	if( !get_param_uint( PARAM_WAIT_FOR_HEARTBEAT ) || ( _system_status.sensors.offboard_heartbeat.health == SYSTEM_HEALTH_OK ) ) {
		if( (chan == MAVLINK_COMM_0 ) && comm_is_open( COMM_CH_0 ) ) {
			serialWrite(Serial1, ch);
		//XXX: } else if( (chan == MAVLINK_COMM_1 ) && comm_is_open( COMM_CH_1 ) ) {
		//XXX: 	serialWrite(Serial2, ch);
		}
	}
}

//==-- On-demand messages
void mavlink_send_statustext(uint8_t port, uint8_t severity, char* text) {
	mavlink_msg_statustext_send(port,
								severity,
								&text[0]);
}

void mavlink_send_broadcast_statustext(uint8_t severity, char* text) {
	if( comm_is_open( COMM_CH_0 ) )
		mavlink_send_statustext(MAVLINK_COMM_0, severity, text);

	//XXX: if( comm_is_open( COMM_CH_1 ) )
	//XXX: 	mavlink_send_statustext(MAVLINK_COMM_1, severity, text);
}

void mavlink_send_timesync(uint8_t port, uint64_t tc1, uint64_t ts1) {
	mavlink_msg_timesync_send(port,
							  tc1,
							  ts1);
}

//==-- Low priority message queue
static uint16_t get_lpq_next_slot(mavlink_queue_t* queue) {
	uint16_t next_slot = 0;

	next_slot = queue->position + queue->length;

	//See if the queue needs to wrap around the circular buffer
	if (next_slot >= LOW_PRIORITY_QUEUE_SIZE)
		next_slot = next_slot - LOW_PRIORITY_QUEUE_SIZE;

	return next_slot;
}

static bool check_lpq_space_free(mavlink_queue_t* queue) {
	//If the count is at the queue size limit, return false
	return (queue->length < LOW_PRIORITY_QUEUE_SIZE);
}

static void remove_current_lpq_message(mavlink_queue_t* queue) {
	queue->position++;

	if(queue->position >= LOW_PRIORITY_QUEUE_SIZE)
		queue->position = 0;

	if(queue->length > 0)
		queue->length--;
}

bool lpq_queue_msg(uint8_t port, mavlink_message_t *msg) {
	bool success = false;
	mavlink_queue_t* queue;

	if( port == _lpq_port_0.port ) {
		queue = &_lpq_port_0;
	//XXX: } else if( port == _lpq_port_1.port ) {
	//XXX: 	queue = &_lpq_port_1;
	}

	if( check_lpq_space_free( queue ) && ( queue != NULL) ) {
		uint8_t i = get_lpq_next_slot(queue);
		queue->buffer_len[i] = mavlink_msg_to_send_buffer(queue->buffer[i], msg);	//Copy message struct data to buffer;
		queue->length++;

		success = true;
	} else {
		if( micros() - queue->timer_warn_full > 1000000) {	//XXX: Only outout the error at 1/s maximum otherwise buffer will never catch up
			mavlink_send_statustext(queue->port, MAV_SEVERITY_ERROR, "[COMMS] LPQ message dropped!");
			queue->timer_warn_full = micros();
		}
	}

	return success;
}

static void lpq_queue_all_params(mavlink_queue_t* queue) {
	mavlink_message_t msg;
	mavlink_prepare_param_value( &msg, queue->request_all_params );

	//Don't flood the buffer
	if( lpq_queue_msg( queue->port, &msg ) ) {
		queue->request_all_params++;

		if( queue->request_all_params >= PARAMS_COUNT ) {
			queue->request_all_params = -1;
		}
	}
}

void lpq_queue_broadcast_msg(mavlink_message_t *msg) {
	if( comm_is_open( COMM_CH_0 ) )
		lpq_queue_msg(MAVLINK_COMM_0, msg);

	//XXX: if( comm_is_open( COMM_CH_1 ) )
	//XXX: 	lpq_queue_msg(MAVLINK_COMM_1, msg);
}

void lpq_send(mavlink_queue_t* queue) {
	//If there are messages in the queue
	if(queue->length > 0) {
		//Transmit the message
		for(uint16_t i = 0; i < queue->buffer_len[queue->position]; i++) {
			comm_send_ch(queue->port, queue->buffer[queue->position][i]);
		}

		//Move the queue along
		remove_current_lpq_message(queue);
	}
}

//==-- Streams
void mavlink_stream_low_priority(uint8_t port) {
	if( port == _lpq_port_0.port ) {
		if( _lpq_port_0.request_all_params >= 0 )
			lpq_queue_all_params(&_lpq_port_0);

		lpq_send(&_lpq_port_0);
	//XXX: } else if( port == _lpq_port_1.port ) {
	//XXX: 	if( _lpq_port_1.request_all_params >= 0 )
	//XXX: 		lpq_queue_all_params(&_lpq_port_1);
	//XXX:
	//XXX: 	lpq_send(&_lpq_port_1);
	}
}

void mavlink_stream_heartbeat(uint8_t port) {
	mavlink_msg_heartbeat_send(port,
							   get_param_uint(PARAM_MAV_TYPE),
							   MAV_AUTOPILOT_PX4,	//XXX: This is to get compatibility for offboard software
							   _system_status.mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//XXX: Set custom mode to allow for the pretend mode
							   compat_encode_px4_main_mode( MAIN_MODE_OFFBOARD ),	//We don't use custom_mode, but pretend to match px4 custom_mode for OFFBOARD
							   _system_status.state);
}

void mavlink_stream_sys_status(uint8_t port) {
	uint32_t onboard_control_sensors_present = 0;
	uint32_t onboard_control_sensors_enabled = 0;
	uint32_t onboard_control_sensors_health = 0;
	uint16_t load = 0;
	uint16_t voltage_battery = fix16_to_int(fix16_mul(_fc_1000,_sensors.voltage_monitor.state_filtered));
	uint16_t current_battery = -1;
	uint8_t battery_remaining = _sensors.voltage_monitor.precentage;
	uint16_t drop_rate_comm = 0;
	uint16_t errors_comm = 0;	//TODO: Make an alert to say if the UART overflows
	uint16_t errors_count1 = _lpq_port_0.length;
	uint16_t errors_count2 = 0;
	uint16_t errors_count3 = _sensors.clock.min;
	uint16_t errors_count4 = _sensors.clock.max;

	load = _sensors.clock.average_time/_sensors.clock.counter;

	_sensors.clock.counter = 0;
	_sensors.clock.average_time = 0;
	_sensors.clock.max = 0;
	_sensors.clock.min = 1000;

	//==-- Sensors Present
	if(_sensors.imu.status.present) {
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
	}

	if(_sensors.mag.status.present)
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;

	if(_sensors.baro.status.present)
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

	onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
	onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
	onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

	//TODO: We should have params for enabling specific sensors
	//==-- Sensors Present
	onboard_control_sensors_enabled = MAV_SYS_STATUS_SENSOR_3D_GYRO |
									  MAV_SYS_STATUS_SENSOR_3D_ACCEL |
									  MAV_SYS_STATUS_SENSOR_3D_MAG |
									  MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
									  MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
									  MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
									  MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

	//==-- Sensor Health
	if(_system_status.sensors.imu.health == SYSTEM_HEALTH_OK) {
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
	}

	if(_system_status.sensors.mag.health == SYSTEM_HEALTH_OK)
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;

	if(_system_status.sensors.baro.health == SYSTEM_HEALTH_OK)
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

	onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
	onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
	onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

	//TODO: Other sensors?
	// MAV_SYS_STATUS_SENSOR_BATTERY, MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, MAV_SYS_STATUS_SENSOR_VISION_POSITION ...?

	//mavlink_msg_sys_status_send(MAVLINK_COMM_0, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
	mavlink_msg_sys_status_send(port,
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
void mavlink_stream_highres_imu(uint8_t port) {
	//TODO: Need to add temp, pressure measurements
	mavlink_msg_highres_imu_send(port,
								 _sensors.imu.status.time_read,
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

void mavlink_stream_attitude(uint8_t port) {
	fix16_t roll;
	fix16_t pitch;
	fix16_t yaw;

	//Extract Euler Angles for controller
	euler_from_quat(&_state_estimator.attitude, &roll, &pitch, &yaw);

	mavlink_msg_attitude_send(port,
							  _sensors.imu.status.time_read,
							  fix16_to_float(roll),
							  fix16_to_float(pitch),
							  fix16_to_float(yaw),
							  fix16_to_float(_state_estimator.p),
							  fix16_to_float(_state_estimator.q),
							  fix16_to_float(_state_estimator.r));
}

void mavlink_stream_attitude_quaternion(uint8_t port) {
	mavlink_msg_attitude_quaternion_send(port,
										 _sensors.imu.status.time_read,
										 fix16_to_float(_state_estimator.attitude.a),
										 fix16_to_float(_state_estimator.attitude.b),
										 fix16_to_float(_state_estimator.attitude.c),
										 fix16_to_float(_state_estimator.attitude.d),
										 fix16_to_float(_state_estimator.p),
										 fix16_to_float(_state_estimator.q),
										 fix16_to_float(_state_estimator.r));
}

void mavlink_stream_attitude_target(uint8_t port) {
	float q[4] = {fix16_to_float(_control_input.q.a),
				  fix16_to_float(_control_input.q.b),
				  fix16_to_float(_control_input.q.c),
				  fix16_to_float(_control_input.q.d)};

	//Use the control output for some of these commands as they reflect the actual goals
	// The input mask applied is included, but the information will still potentially be useful
	// The timestamp used is the one that is used to generate the commands
	mavlink_msg_attitude_target_send(port,
									 sensors_clock_ls_get(),
									 _control_input.input_mask,
									 &q[0],
									 fix16_to_float(_control_input.r),
									 fix16_to_float(_control_input.p),
									 fix16_to_float(_control_input.y),
									 fix16_to_float(_control_input.T));
}

void mavlink_stream_servo_output_raw(uint8_t port) {
	mavlink_msg_servo_output_raw_send(port,
									  sensors_clock_ls_get(),
									  0,	//Port 0
									  _pwm_output[0],
									  _pwm_output[1],
									  _pwm_output[2],
									  _pwm_output[3],
									  _pwm_output[4],
									  _pwm_output[5],
									  _pwm_output[6],
									  _pwm_output[7],
									  0,0,0,0,0,0,0,0);	//XXX: We don't even have enough ports
}

void mavlink_stream_timesync(uint8_t port) {
	mavlink_msg_timesync_send(port,
							  0,
							  ( (uint64_t)micros() ) * 1000);
}

void mavlink_stream_battery_status(uint8_t port) {
	uint8_t batt_id = 0;

	uint16_t voltages[10];
	fix16_t voltage_cell = fix16_div( _sensors.voltage_monitor.state_filtered, fix16_from_int(get_param_uint(PARAM_BATTERY_CELL_NUM)) );
	uint16_t voltage_cell_int = fix16_to_int( fix16_mul( _fc_1000, voltage_cell ) );
	for(int i = 0; i < 10; i++) {
		if(i < (int)get_param_uint(PARAM_BATTERY_CELL_NUM)) {
			voltages[i] = voltage_cell_int;
		} else {
			voltages[i] = UINT16_MAX;
		}
	}

	//TODO: This could be done better
	uint8_t charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;

	if(get_param_uint(PARAM_BATTERY_CELL_NUM) > 0) {
		if(_sensors.voltage_monitor.precentage > 100) {
			charge_state = MAV_BATTERY_CHARGE_STATE_UNHEALTHY;
		} else if(_sensors.voltage_monitor.precentage > (int)get_param_uint(PARAM_BATTERY_CHARGE_STATE_OK)) {
			charge_state = MAV_BATTERY_CHARGE_STATE_OK;
		} else if(_sensors.voltage_monitor.precentage > (int)get_param_uint(PARAM_BATTERY_CHARGE_STATE_LOW)) {
			charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
		} else if(_sensors.voltage_monitor.precentage > (int)get_param_uint(PARAM_BATTERY_CHARGE_STATE_CRITICAL)) {
			charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
		} else if(_sensors.voltage_monitor.precentage > 0) {
			charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
		} else {
			charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
		}
	}

	mavlink_msg_battery_status_send(port,
								   batt_id,
								   get_param_uint(PARAM_BATTERY_FUNCTION),
								   get_param_uint(PARAM_BATTERY_TYPE),
								   INT16_MAX,
								   &voltages[0],
								   -1,
								   -1,
								   -1,
								   _sensors.voltage_monitor.precentage,
								   0,
								   charge_state);
}

//==-- Low Priority Messages

//Sends a status text message
void mavlink_prepare_statustext(mavlink_message_t *msg, uint8_t severity, char* text) {
	mavlink_msg_statustext_pack(mavlink_system.sysid,
								 mavlink_system.compid,
								 msg,
								 severity,
								 text);
}

//Broadcasts an notice to all open comm channels
void mavlink_queue_broadcast_info(char* text) {
	mavlink_message_t msg;
	mavlink_prepare_statustext(&msg, MAV_SEVERITY_INFO, text);

	lpq_queue_broadcast_msg(&msg);
}

//Broadcasts an notice to all open comm channels
void mavlink_queue_broadcast_notice(char* text) {
	mavlink_message_t msg;
	mavlink_prepare_statustext(&msg, MAV_SEVERITY_NOTICE, text);

	lpq_queue_broadcast_msg(&msg);
}

//Broadcasts an error to all open comm channels
void mavlink_queue_broadcast_error(char* text) {
	mavlink_message_t msg;
	mavlink_prepare_statustext(&msg, MAV_SEVERITY_ERROR, text);

	lpq_queue_broadcast_msg(&msg);
}

//Sends a debug parameter
void mavlink_prepare_debug(mavlink_message_t *msg, uint32_t stamp, uint8_t index, uint32_t value) {
	union {
		float f;
		uint32_t i;
	} u;	//The bytes are translated to the right unit on receiving, but need to be sent as a "float"

	u.i = value;

	mavlink_msg_debug_pack(mavlink_system.sysid,
							mavlink_system.compid,
							msg,
							stamp,	//Timestamo
							index,	//Variable index
							u.f);	//Value (always as float)
}

//Prepares an the mavlink protocol version message
void mavlink_prepare_protocol_version(mavlink_message_t *msg) {
	mavlink_msg_protocol_version_pack(mavlink_system.sysid,
									   mavlink_system.compid,
									   msg,
									   MAVLINK_VERSION_MAX,
									   MAVLINK_VERSION_MIN,
									   MAVLINK_VERSION_MAX,
									   &blank_array_[0],
									   (uint8_t*)GIT_VERSION_MAVLINK_STR);
}

//Prepares an autopilot version details
void mavlink_prepare_autopilot_version(mavlink_message_t *msg) {
	//TODO: Update capabilities
	const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT +
								  MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET +
								  MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET +
								  MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION +
							  	  MAV_PROTOCOL_CAPABILITY_MAVLINK2;

	mavlink_msg_autopilot_version_pack(mavlink_system.sysid,
									   mavlink_system.compid,
									   msg,
									   capabilities,
									   get_param_uint(PARAM_VERSION_FIRMWARE),
									   0,
									   get_param_uint(PARAM_VERSION_SOFTWARE),
									   get_param_uint(PARAM_BOARD_REVISION),
									   &blank_array_[0],
									   &blank_array_[0],
									   &blank_array_[0],
									   0x10c4,	//TODO: This is the serial vendor and product ID, should be dynamic?
									   0xea60,
									   U_ID_0,
									   &blank_array_uid_[0]);
}

//Sends a command acknowledgement
void mavlink_prepare_command_ack(mavlink_message_t *msg, uint16_t command, uint8_t result, uint8_t sender_sysid, uint8_t sender_compid, uint8_t progress) {
	mavlink_msg_command_ack_pack(mavlink_system.sysid,
								 mavlink_system.compid,
								 msg,
								 command,
								 result,
								 progress, 0xff, sender_sysid, sender_compid);
}

//Sends the requested parameter
void mavlink_prepare_param_value(mavlink_message_t *msg, uint32_t index) {
	//char param_name[PARAMS_NAME_LENGTH];
	bool param_ok = false;

	union {
		float f;
		int32_t i;
		uint32_t u;
	} u;	//The bytes are translated to the right unit on receiving, but need to be sent as a "float"

	switch(_params.types[index]) {
		case MAVLINK_TYPE_UINT32_T: {
			u.u = get_param_uint(index);
			param_ok = true;

			break;
		}
		case MAVLINK_TYPE_INT32_T : {
			u.i = get_param_int(index);
			param_ok = true;

			break;
		}
		case MAVLINK_TYPE_FLOAT: {
			u.f = fix16_to_float(get_param_fix16(index));
			param_ok = true;

			break;
		}
		default: {
			break;
		}
	}

	if( param_ok ) {
		mavlink_msg_param_value_pack(mavlink_system.sysid,
									 mavlink_system.compid,
									 msg,
									 &_param_names[index][0],		//String of name
									 u.f,			//Value (always as float)
									 _params.types[index],		//From MAV_PARAM_TYPE
									 PARAMS_COUNT,	//Total number of parameters
									 index);
	} else {
		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "[PARAM] Unknown paramater type found: ";
		char bad_param_id[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
		itoa(index, bad_param_id, 10);
		strncat(text, bad_param_id, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN -1); //XXX: Stops overflow warnings
		mavlink_queue_broadcast_error(text);
	}
}
