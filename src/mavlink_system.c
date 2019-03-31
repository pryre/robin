#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"
#include "mavlink_transmit.h"

#include "drivers/drv_comms.h"
#include "drivers/drv_system.h"

#include "controller.h"
#include "drivers/drv_status_io.h"
#include "drivers/drv_pwm.h"
#include "estimator.h"
#include "fix16.h"
#include "fixextra.h"
#include "mixer.h"
#include "param_gen.h"
#include "params.h"
#include "safety.h"
#include "sensors.h"
#include "profiler.h"

#include <stdlib.h>

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */

mavlink_queue_t _lpq;

mavlink_system_t _mavlink_gcs;
mavlink_system_t mavlink_system;
bool _ch_0_have_heartbeat;
bool _ch_1_have_heartbeat;

params_t _params;
const char _param_names[PARAMS_COUNT]
					   [MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];

system_status_t _system_status;
sensor_readings_t _sensors;
state_t _state_estimator;
fix16_t _io_pin_states[8];

command_input_t _control_input;

static mavlink_message_t mavlink_msg_buf_port0_;
static mavlink_message_t mavlink_msg_buf_port1_;
static const uint8_t blank_array_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t blank_array_uid_[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
											 0, 0, 0, 0, 0, 0, 0, 0, 0};

void communications_system_init( void ) {
	mavlink_system.sysid = get_param_uint( PARAM_SYSTEM_ID );	 // System ID, 1-255
	mavlink_system.compid = get_param_uint( PARAM_COMPONENT_ID ); // Component/Subsystem ID, 1-255

	_mavlink_gcs.sysid = get_param_uint( PARAM_GCS_SYSTEM_ID );		// System ID, 1-255
	_mavlink_gcs.compid = get_param_uint( PARAM_GCS_COMPONENT_ID ); // Component/Subsystem ID, 1-255

	_lpq.position = 0;
	_lpq.length = 0;
	_lpq.request_all_params_port0 = -1;
	_lpq.request_all_params_port1 = -1;
	_lpq.param_position = 0;
	_lpq.param_length = 0;
	_lpq.timer_warn_full = 0;
	_lpq.timer_param_warn_full = 0;

	if ( get_param_uint( PARAM_BAUD_RATE_0 ) > 0 )
		comms_init_port( COMM_PORT_0 );

	if ( get_param_uint( PARAM_BAUD_RATE_1 ) > 0 )
		comms_init_port( COMM_PORT_1 );

	_ch_0_have_heartbeat = false;
	_ch_1_have_heartbeat = false;

	for ( mavlink_stream_id_t i = 0; i < MAVLINK_STREAM_COUNT; i++ ) {
		communication_calc_period_update( COMM_PORT_0, i );
		communication_calc_period_update( COMM_PORT_1, i );
	}

	mavlink_set_proto_version( MAVLINK_COMM_0, 1 );
	mavlink_set_proto_version( MAVLINK_COMM_1, 1 );
}

/*
static void comm_wait_ready( mavlink_channel_t chan ) {
        bool ready = false;

        while(!ready) {
                if( chan == MAVLINK_COMM_0) {
                        ready = isUartTransmitBufferEmpty(Serial1);
                } else if( chan == MAVLINK_COMM_1) {
                        ready = isUartTransmitBufferEmpty(Serial2);
                }
        }
}
                system_debug_print("Attempted to send data with
comm_send_ch(MAVLINK_COMM_0,ch)");
*/

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void comm_send_ch( mavlink_channel_t chan, uint8_t ch ) {
	/*
if( chan == MAVLINK_COMM_0 ) {
comms_send(COMM_PORT_0, ch);
} else if( chan == MAVLINK_COMM_1 ) {
comms_send(COMM_PORT_1, ch);
}
*/

	if ( chan == MAVLINK_COMM_0 ) {
		system_debug_print(
			"Attempted to send data with comm_send_ch(MAVLINK_COMM_0,ch)" );
	} else if ( chan == MAVLINK_COMM_1 ) {
		system_debug_print(
			"Attempted to send data with comm_send_ch(MAVLINK_COMM_1,ch)" );
	}
}

mavlink_message_t* get_channel_buf( mavlink_channel_t chan ) {
	mavlink_message_t* ch_buf = NULL;

	if ( chan == MAVLINK_COMM_0 ) {
		ch_buf = &mavlink_msg_buf_port0_;
	} else {
		ch_buf = &mavlink_msg_buf_port1_;
	}
	return ch_buf;
}

// Sends out the current message buffer
void comms_send_msg( mavlink_channel_t chan ) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	if ( chan == MAVLINK_COMM_0 ) {
		uint32_t len = mavlink_msg_to_send_buffer( buf, &mavlink_msg_buf_port0_ );
		comms_send_datagram( COMM_PORT_0, buf, len );
	} else if ( chan == MAVLINK_COMM_1 ) {
		uint32_t len = mavlink_msg_to_send_buffer( buf, &mavlink_msg_buf_port1_ );
		comms_send_datagram( COMM_PORT_1, buf, len );
	}
}

static void mavlink_debug_cli_message( uint8_t severity, char* text ) {
	system_debug_print( text );
}

//==-- On-demand messages
void mavlink_send_statustext( mavlink_channel_t chan, uint8_t severity,
							  char* text ) {
	// comm_wait_ready(port);
	mavlink_msg_statustext_pack( mavlink_system.sysid, mavlink_system.compid,
								 get_channel_buf( chan ), severity, &text[0] );
	comms_send_msg( chan );
}

void mavlink_send_broadcast_statustext( uint8_t severity, char* text ) {
	mavlink_debug_cli_message( severity, text );

	if ( comms_is_open( COMM_PORT_0 ) )
		mavlink_send_statustext( MAVLINK_COMM_0, severity, text );

	if ( comms_is_open( COMM_PORT_1 ) )
		mavlink_send_statustext( MAVLINK_COMM_1, severity, text );
}

void mavlink_send_timesync( mavlink_channel_t chan, uint64_t tc1, uint64_t ts1 ) {
	// comm_wait_ready(port);

	mavlink_msg_timesync_pack( mavlink_system.sysid, mavlink_system.compid,
							   get_channel_buf( chan ), tc1, ts1 );
	comms_send_msg( chan );
}

//==-- Low priority message queue
static uint16_t get_lpq_next_slot(void) {
	uint16_t next_slot = 0;

	next_slot = _lpq.position + _lpq.length;

	// See if the queue needs to wrap around the circular buffer
	if ( next_slot >= LOW_PRIORITY_QUEUE_SIZE )
		next_slot = next_slot - LOW_PRIORITY_QUEUE_SIZE;

	return next_slot;
}

static bool check_lpq_space_free(void) {
	// If the count is at the queue size limit, return false
	return ( _lpq.length < LOW_PRIORITY_QUEUE_SIZE );
}

static void remove_current_lpq_message(void) {
	_lpq.position++;

	if ( _lpq.position >= LOW_PRIORITY_QUEUE_SIZE )
		_lpq.position = 0;

	if ( _lpq.length > 0 )
		_lpq.length--;
}

static uint16_t get_lpq_param_next_slot(void) {
	uint16_t next_slot = 0;

	next_slot = _lpq.param_position + _lpq.param_length;

	// See if the queue needs to wrap around the circular buffer
	if ( next_slot >= LOW_PRIORITY_QUEUE_PARAMS_SIZE )
		next_slot = next_slot - LOW_PRIORITY_QUEUE_PARAMS_SIZE;

	return next_slot;
}

static bool check_lpq_param_space_free(void) {
	// If the count is at the queue size limit, return false
	return ( _lpq.param_length < LOW_PRIORITY_QUEUE_PARAMS_SIZE );
}

static void remove_current_lpq_param_message(void) {
	_lpq.param_position++;

	if ( _lpq.param_position >= LOW_PRIORITY_QUEUE_PARAMS_SIZE )
		_lpq.param_position = 0;

	if ( _lpq.param_length > 0 )
		_lpq.param_length--;
}

static bool lpq_queue_msg_port( uint8_t port, mavlink_message_t* msg ) {
	bool success = false;

	// Only add in port if it is open
	if ( !comms_is_open( port & COMM_PORT_0 ) )
		port &= ~COMM_PORT_0;

	if ( !comms_is_open( port & COMM_PORT_1 ) )
		port &= ~COMM_PORT_1;

	// Only add param if port is valid and there is room
	if ( port && check_lpq_space_free() ) {
		uint8_t i = get_lpq_next_slot();
		_lpq.buffer_len[i] = mavlink_msg_to_send_buffer(
			_lpq.buffer[i], msg ); // Copy message struct data to buffer
		_lpq.buffer_port[i] = port;
		_lpq.length++;

		success = true;
	} else if ( !port ) {
		// Return an accept, even though no ports were open
		success = true;
	} else {
		if ( system_micros() - _lpq.timer_warn_full > 1000000 ) { // XXX: Only outout the error at 1/s maximum otherwise buffer
			// will never catch up
			mavlink_send_broadcast_statustext( MAV_SEVERITY_ERROR,
											   "[COMMS] LPQ message dropped!" );
			_lpq.timer_warn_full = system_micros();
		}
	}

	return success;
}

bool lpq_queue_msg( mavlink_channel_t chan, mavlink_message_t* msg ) {
	uint8_t port = 0;

	if ( chan == MAVLINK_COMM_0 ) {
		port = COMM_PORT_0;
	} else if ( chan == MAVLINK_COMM_1 ) {
		port = COMM_PORT_1;
	}

	return lpq_queue_msg_port( port, msg );
}

void lpq_queue_broadcast_msg( mavlink_message_t* msg ) {
	uint8_t port = 0;

	if ( comms_is_open( COMM_PORT_0 ) )
		port |= COMM_PORT_0;

	if ( comms_is_open( COMM_PORT_1 ) )
		port |= COMM_PORT_1;

	lpq_queue_msg_port( port, msg );
}

static bool lpq_queue_param_port( uint8_t port, uint32_t index ) {
	bool success = false;

	// Only add in port if it is open
	if ( !comms_is_open( port & COMM_PORT_0 ) )
		port &= ~COMM_PORT_0;

	if ( !comms_is_open( port & COMM_PORT_1 ) )
		port &= ~COMM_PORT_1;

	// Only add param if port is valid and there is room
	if ( port && check_lpq_param_space_free() ) {
		uint8_t i = get_lpq_param_next_slot();
		_lpq.param_buffer[i] = index;
		_lpq.param_buffer_port[i] = port;
		_lpq.param_length++;

		success = true;
	} else if ( !port ) {
		// Return an accept, even though no ports were open
		success = true;
	}
	/*
} else {
if( micros() - _lpq.timer_param_warn_full > 1000000) {	//XXX:
Only outout the error at 1/s maximum otherwise buffer will never catch up
      mavlink_send_broadcast_statustext(MAV_SEVERITY_ERROR, "[COMMS]
LPQ param message dropped!");
      _lpq.timer_param_warn_full = micros();
}
}
*/
	return success;
}

bool lpq_queue_param_broadcast( uint32_t index ) {
	return lpq_queue_param_port( ( COMM_PORT_0 | COMM_PORT_1 ),
								 index ); // XXX: Needs all comms listed here
}

static void lpq_queue_all_params( mavlink_channel_t chan ) {
	int32_t* req;
	uint8_t port = 0;

	if ( chan == MAVLINK_COMM_0 ) {
		req = &_lpq.request_all_params_port0;
		port = COMM_PORT_0;
	} else if ( chan == MAVLINK_COMM_1 ) {
		req = &_lpq.request_all_params_port1;
		port = COMM_PORT_1;
	}

	// Don't increase count if the add failed
	if ( lpq_queue_param_port( port, *req ) ) {
		( *req )++;

		if ( *req >= PARAMS_COUNT ) {
			*req = -1;
		}
	}
}

static void lpq_send( mavlink_channel_t chan ) {
	// comm_wait_ready(port);

	// If there are messages in the queue
	if ( _lpq.length > 0 ) {
		if ( ( chan == MAVLINK_COMM_0 ) && ( _lpq.buffer_port[_lpq.position] | COMM_PORT_0 ) ) {
			// Transmit the message to channel 0
			comms_send_datagram( COMM_PORT_0, _lpq.buffer[_lpq.position],
								 _lpq.buffer_len[_lpq.position] );
			// for(uint16_t i = 0; i < _lpq.buffer_len[_lpq.position]; i++) {
			//	comm_send_ch(chan, _lpq.buffer[_lpq.position][i]);
			//}

			// Unset this channel
			_lpq.buffer_port[_lpq.position] &= ~COMM_PORT_0;
		} else if ( ( chan == MAVLINK_COMM_1 ) && ( _lpq.buffer_port[_lpq.position] | COMM_PORT_1 ) ) {
			// Transmit the message to channel 1
			comms_send_datagram( COMM_PORT_1, _lpq.buffer[_lpq.position],
								 _lpq.buffer_len[_lpq.position] );
			// for(uint16_t i = 0; i < _lpq.buffer_len[_lpq.position]; i++) {
			//	comm_send_ch(chan, _lpq.buffer[_lpq.position][i]);
			//}

			// Unset this channel
			_lpq.buffer_port[_lpq.position] &= ~COMM_PORT_1;
		}

		// If there are no more ports left to send this message to
		// Move the queue along
		if ( !_lpq.buffer_port[_lpq.position] )
			remove_current_lpq_message();
	} else if ( _lpq.param_length > 0 ) {
		// Check to see if there's parameters to send out
		mavlink_message_t msg_out;
		mavlink_prepare_param_value( &msg_out,
									 _lpq.param_buffer[_lpq.param_position] );
		uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t msg_len = mavlink_msg_to_send_buffer( msg_buf, &msg_out );

		if ( ( chan == MAVLINK_COMM_0 ) && ( _lpq.param_buffer_port[_lpq.param_position] | COMM_PORT_0 ) ) {
			// Transmit the message to channel 0
			comms_send_datagram( COMM_PORT_0, msg_buf, msg_len );
			// for(uint16_t i = 0; i < msg_len; i++) {
			//	comm_send_ch(chan, msg_buf[i]);
			//}

			// Unset this channel
			_lpq.param_buffer_port[_lpq.param_position] &= ~COMM_PORT_0;
		} else if ( ( chan == MAVLINK_COMM_1 ) && ( _lpq.param_buffer_port[_lpq.param_position] | COMM_PORT_1 ) ) {
			// Transmit the message to channel 1
			comms_send_datagram( COMM_PORT_1, msg_buf, msg_len );
			// for(uint16_t i = 0; i < msg_len; i++) {
			//	comm_send_ch(chan, msg_buf[i]);
			//}

			// Unset this channel
			_lpq.param_buffer_port[_lpq.param_position] &= ~COMM_PORT_1;
		}

		// If there are no more ports left to send this message to
		// Move the queue along
		if ( !_lpq.param_buffer_port[_lpq.param_position] ) {
			// mavlink_send_broadcast_statustext(MAV_SEVERITY_INFO, "[COMMS] REMOVE
			// PARAM!");
			remove_current_lpq_param_message();
		}
	}
}

//==-- Streams

bool mavlink_stream_ready( mavlink_channel_t chan ) {
	bool ready = !get_param_uint( PARAM_WAIT_FOR_HEARTBEAT );

	if ( !ready ) {
		if ( chan == MAVLINK_COMM_0 ) {
			ready = _ch_0_have_heartbeat;
		} else if ( chan == MAVLINK_COMM_1 ) {
			ready = _ch_1_have_heartbeat;
		}
	}

	// if(ready)
	//	comm_wait_ready(port);

	return ready;
}

void mavlink_stream_low_priority( mavlink_channel_t chan ) {
	// XXX: Always send LPQ

	// If there is params to queue, and the queue is empty
	//  we can afford to interject more parameters this pass
	if ( chan == MAVLINK_COMM_0 ) {
		if ( ( _lpq.request_all_params_port0 >= 0 ) && ( _lpq.length == 0 ) && ( _lpq.param_length == 0 ) ) {
			lpq_queue_all_params( chan );
		}
	} else if ( chan == MAVLINK_COMM_1 ) {
		if ( ( _lpq.request_all_params_port1 >= 0 ) && ( _lpq.length == 0 ) && ( _lpq.param_length == 0 ) ) {
			lpq_queue_all_params( chan );
		}
	}

	lpq_send( chan );
}

void mavlink_stream_heartbeat( mavlink_channel_t chan ) {
	// XXX: Always send heartbeat
	// comm_wait_ready(port);

	mavlink_msg_heartbeat_pack(
		mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
		get_param_uint( PARAM_MAV_TYPE ),
		MAV_AUTOPILOT_PX4, // XXX: This is to get compatibility for offboard
		// software
		_system_status.mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // XXX: Set custom mode to allow
		// for the pretend mode
		compat_encode_px4_main_mode(
			_system_status.control_mode ), // We don't use custom_mode, but pretend
		// to match px4 custom_mode for OFFBOARD
		_system_status.state );
	comms_send_msg( chan );
}

void mavlink_stream_status_io( mavlink_channel_t chan ) {
	float status_io[8];
	for ( int i = 0; i < 8; i++ )
		status_io[i] = fix16_to_float( _io_pin_states[i] );

	mavlink_msg_actuator_control_target_pack(
		mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
		system_micros(), IO_PIN_STATE_GROUP_MIX, &status_io[0] );
	comms_send_msg( chan );
}

void mavlink_stream_sys_status( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {

		uint32_t loop_min = 0;
		uint32_t loop_mean = 0;
		uint32_t loop_max = 0;

		uint32_t loop_min_t = 0;
		uint32_t loop_mean_t = 0;
		uint32_t loop_max_t = 0;

		if( profiler_read(PROFILER_ID_LOOP, &loop_min_t,  &loop_mean_t,  &loop_max_t) ) {
			loop_min = loop_min_t;
			loop_mean = loop_mean_t;
			loop_max = loop_max_t;
		}

		uint32_t onboard_control_sensors_present = 0;
		uint32_t onboard_control_sensors_enabled = 0;
		uint32_t onboard_control_sensors_health = 0;

		uint16_t load = (100 * loop_mean) / PROFILER_IDEAL_MAX_LOOP;
		uint16_t voltage_battery = fix16_to_int(
			fix16_mul( _fc_1000, _sensors.voltage_monitor.state_filtered ) );
		int16_t current_battery = -1;
		int8_t battery_remaining = fix16_to_int( fix16_mul( _fc_100, _sensors.voltage_monitor.precentage ) );

		uint32_t num_packets_drop = mavlink_get_channel_status( MAVLINK_COMM_0 )->packet_rx_drop_count + mavlink_get_channel_status( MAVLINK_COMM_1 )->packet_rx_drop_count;
		uint32_t num_packets_success = mavlink_get_channel_status( MAVLINK_COMM_0 )->packet_rx_success_count + mavlink_get_channel_status( MAVLINK_COMM_1 )->packet_rx_success_count;
		uint32_t num_packets = num_packets_drop + num_packets_success;
		uint16_t drop_rate_comm = 0;
		if ( num_packets > 0 ) {
			drop_rate_comm = ( 100 * num_packets_drop ) / num_packets;
		}
		uint16_t errors_comm = comms_tx_error_num( COMM_PORT_0 ) + comms_rx_error_num( COMM_PORT_0 ) + comms_tx_error_num( COMM_PORT_1 ) + comms_rx_error_num( COMM_PORT_1 );

		uint16_t errors_count1 = _lpq.length;
		uint16_t errors_count2 = fix16_to_int( _control_timing.average_update );
		uint16_t errors_count3 = loop_min;
		uint16_t errors_count4 = loop_max;

		//==-- Sensors Present
		if ( _sensors.imu.status.present ) {
			onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
			onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
		}

		if ( _sensors.mag.status.present )
			onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;

		if ( _sensors.baro.status.present )
			onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
		onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

		//==-- Sensors Enabled
		onboard_control_sensors_enabled = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

		//==-- Sensor Health
		if ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
			onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
			onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
		}

		if ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK )
			onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;

		if ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK )
			onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
		onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

		// TODO: Other sensors?
		// MAV_SYS_STATUS_SENSOR_BATTERY, MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW,
		// MAV_SYS_STATUS_SENSOR_VISION_POSITION ...?

		mavlink_msg_sys_status_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			onboard_control_sensors_present, onboard_control_sensors_enabled,
			onboard_control_sensors_health, load, voltage_battery, current_battery,
			battery_remaining, drop_rate_comm, errors_comm, errors_count1,
			errors_count2, errors_count3, errors_count4 );
		comms_send_msg( chan );
	}
}

//==-- Sends the latest IMU reading
void mavlink_stream_highres_imu( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		// TODO: Need to add temp, pressure measurements
		float xacc = 0;
		float yacc = 0;
		float zacc = 0;
		float xgyro = 0;
		float ygyro = 0;
		float zgyro = 0;
		float xmag = 0;
		float ymag = 0;
		float zmag = 0;
		float abs_pressure = 0;
		float diff_pressure = 0;
		float pressure_alt = 0;
		float temperature = 0;
		uint16_t fields_updated = 0;

		if ( _system_status.sensors.imu.health == SYSTEM_HEALTH_OK ) {
			/*
xacc = fix16_to_float(_sensors.imu.accel.x);
yacc = fix16_to_float(_sensors.imu.accel.y);
zacc = fix16_to_float(_sensors.imu.accel.z);
xgyro = fix16_to_float(_sensors.imu.gyro.x);
ygyro = fix16_to_float(_sensors.imu.gyro.y);
zgyro = fix16_to_float(_sensors.imu.gyro.z);
*/
			// Output our estimated values here
			xacc = fix16_to_float( _state_estimator.ax );
			yacc = fix16_to_float( _state_estimator.ay );
			zacc = fix16_to_float( _state_estimator.az );
			xgyro = fix16_to_float( _state_estimator.p );
			ygyro = fix16_to_float( _state_estimator.q );
			zgyro = fix16_to_float( _state_estimator.r );

			fields_updated |= ( 1 << 0 ) | ( 1 << 1 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) | ( 1 << 5 );
		}

		if ( _system_status.sensors.mag.health == SYSTEM_HEALTH_OK ) {
			xmag = fix16_to_float( _sensors.mag.mag.x );
			ymag = fix16_to_float( _sensors.mag.mag.y );
			zmag = fix16_to_float( _sensors.mag.mag.z );

			fields_updated |= ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 8 );
		}

		if ( _system_status.sensors.baro.health == SYSTEM_HEALTH_OK ) {
			abs_pressure = fix16_to_float( _sensors.baro.raw_press );

			fields_updated |= ( 1 << 9 );
		}

		mavlink_msg_highres_imu_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			_sensors.imu.status.time_read, xacc, yacc, zacc, xgyro, ygyro, zgyro,
			xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt,
			temperature, fields_updated );
		comms_send_msg( chan );
	}
}

void mavlink_stream_attitude( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		fix16_t roll;
		fix16_t pitch;
		fix16_t yaw;

		// Extract Euler Angles for controller
		euler_from_quat( &_state_estimator.attitude, &roll, &pitch, &yaw );

		mavlink_msg_attitude_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			_sensors.imu.status.time_read, fix16_to_float( roll ),
			fix16_to_float( pitch ), fix16_to_float( yaw ),
			fix16_to_float( _state_estimator.p ), fix16_to_float( _state_estimator.q ),
			fix16_to_float( _state_estimator.r ) );
		comms_send_msg( chan );
	}
}

void mavlink_stream_attitude_quaternion( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		mavlink_msg_attitude_quaternion_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			_sensors.imu.status.time_read,
			fix16_to_float( _state_estimator.attitude.a ),
			fix16_to_float( _state_estimator.attitude.b ),
			fix16_to_float( _state_estimator.attitude.c ),
			fix16_to_float( _state_estimator.attitude.d ),
			fix16_to_float( _state_estimator.p ), fix16_to_float( _state_estimator.q ),
			fix16_to_float( _state_estimator.r ) );
		comms_send_msg( chan );
	}
}

void mavlink_stream_attitude_target( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		float q[4] = {
			fix16_to_float( _control_input.q.a ), fix16_to_float( _control_input.q.b ),
			fix16_to_float( _control_input.q.c ), fix16_to_float( _control_input.q.d )};

		// Use the control output for some of these commands as they reflect the
		// actual goals
		// The input mask applied is included, but the information will still
		// potentially be useful
		// The timestamp used is the one that is used to generate the commands
		mavlink_msg_attitude_target_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			profiler_get_loop_start(), _control_input.input_mask, &q[0],
			fix16_to_float( _control_input.r ), fix16_to_float( _control_input.p ),
			fix16_to_float( _control_input.y ), fix16_to_float( _control_input.T ) );
		comms_send_msg( chan );
	}
}

void mavlink_stream_rc_channels_raw( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		mavlink_msg_rc_channels_raw_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			profiler_get_loop_start(),
			0, // Port 0
			_sensors.rc_input.raw[0], _sensors.rc_input.raw[1],
			_sensors.rc_input.raw[2], _sensors.rc_input.raw[3],
			_sensors.rc_input.raw[4], _sensors.rc_input.raw[5],
			_sensors.rc_input.raw[6], _sensors.rc_input.raw[7], 255 );
		comms_send_msg( chan );
	}
}

void mavlink_stream_servo_output_raw( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		mavlink_msg_servo_output_raw_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			profiler_get_loop_start(),
			0, // Port 0
			drv_pwm_get_current(0),
			drv_pwm_get_current(1),
			drv_pwm_get_current(2),
			drv_pwm_get_current(3),
			drv_pwm_get_current(4),
			drv_pwm_get_current(5),
			drv_pwm_get_current(6),
			drv_pwm_get_current(7),
			0, 0, 0, 0, 0, 0, 0, 0 ); //TODO: Expand max ports?
		comms_send_msg( chan );
	}
}

void mavlink_stream_timesync( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		mavlink_msg_timesync_pack( mavlink_system.sysid, mavlink_system.compid,
								   get_channel_buf( chan ), 0,
								   ( (uint64_t)system_micros() ) * 1000 );
		comms_send_msg( chan );
	}
}

void mavlink_stream_battery_status( mavlink_channel_t chan ) {
	if ( mavlink_stream_ready( chan ) ) {
		uint8_t batt_id = 0;

		uint16_t voltages[10];
		fix16_t voltage_cell = fix16_div( _sensors.voltage_monitor.state_filtered,
										  fix16_from_int( get_param_uint( PARAM_BATTERY_CELL_NUM ) ) );
		uint16_t voltage_cell_int = fix16_to_int( fix16_mul( _fc_1000, voltage_cell ) );
		for ( int i = 0; i < 10; i++ ) {
			if ( i < (int)get_param_uint( PARAM_BATTERY_CELL_NUM ) ) {
				voltages[i] = voltage_cell_int;
			} else {
				voltages[i] = UINT16_MAX;
			}
		}

		// TODO: This could be done better
		uint8_t charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;

		if ( get_param_uint( PARAM_BATTERY_CELL_NUM ) > 0 ) {
			if ( _sensors.voltage_monitor.precentage > _fc_1 ) {
				charge_state = MAV_BATTERY_CHARGE_STATE_UNHEALTHY;
			} else if ( _sensors.voltage_monitor.precentage > get_param_fix16( PARAM_BATTERY_CHARGE_STATE_LOW ) ) {
				charge_state = MAV_BATTERY_CHARGE_STATE_OK;
			} else if ( _sensors.voltage_monitor.precentage > get_param_fix16( PARAM_BATTERY_CHARGE_STATE_CRITICAL ) ) {
				charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
			} else if ( _sensors.voltage_monitor.precentage > get_param_fix16( PARAM_BATTERY_CHARGE_STATE_EMERGENCY ) ) {
				charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
			} else if ( _sensors.voltage_monitor.precentage > 0 ) {
				charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
			} else {
				charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
			}
		}

		mavlink_msg_battery_status_pack(
			mavlink_system.sysid, mavlink_system.compid, get_channel_buf( chan ),
			batt_id, get_param_uint( PARAM_BATTERY_FUNCTION ),
			get_param_uint( PARAM_BATTERY_TYPE ), INT16_MAX, &voltages[0], -1, -1, -1,
			fix16_to_int( fix16_mul( _fc_100, _sensors.voltage_monitor.precentage ) ),
			0, charge_state );
		comms_send_msg( chan );
	}
}

//==-- Low Priority Messages

// Sends a status text message
void mavlink_prepare_statustext( mavlink_message_t* msg, uint8_t severity,
								 char* text ) {
	mavlink_msg_statustext_pack( mavlink_system.sysid, mavlink_system.compid, msg,
								 severity, text );
}

// Broadcasts an notice to all open comm channels
void mavlink_queue_broadcast_info( char* text ) {
	mavlink_debug_cli_message( MAV_SEVERITY_INFO, text );

	mavlink_message_t msg;
	mavlink_prepare_statustext( &msg, MAV_SEVERITY_INFO, text );

	lpq_queue_broadcast_msg( &msg );
}

// Broadcasts an notice to all open comm channels
void mavlink_queue_broadcast_notice( char* text ) {
	mavlink_debug_cli_message( MAV_SEVERITY_NOTICE, text );

	mavlink_message_t msg;
	mavlink_prepare_statustext( &msg, MAV_SEVERITY_NOTICE, text );

	lpq_queue_broadcast_msg( &msg );
}

// Broadcasts an error to all open comm channels
void mavlink_queue_broadcast_error( char* text ) {
	mavlink_debug_cli_message( MAV_SEVERITY_ERROR, text );

	mavlink_message_t msg;
	mavlink_prepare_statustext( &msg, MAV_SEVERITY_ERROR, text );

	lpq_queue_broadcast_msg( &msg );
}

// Broadcasts an debug message to all open comm channels
void mavlink_queue_broadcast_debug( char* text ) {
	mavlink_debug_cli_message( MAV_SEVERITY_DEBUG, text );

	mavlink_message_t msg;
	mavlink_prepare_statustext( &msg, MAV_SEVERITY_DEBUG, text );

	lpq_queue_broadcast_msg( &msg );
}

// Sends a debug parameter
void mavlink_prepare_debug( mavlink_message_t* msg, uint32_t stamp,
							uint8_t index, uint32_t value ) {
	union {
		float f;
		uint32_t i;
	} u; // The bytes are translated to the right unit on receiving, but need to
	// be sent as a "float"

	u.i = value;

	mavlink_msg_debug_pack( mavlink_system.sysid, mavlink_system.compid, msg,
							stamp, // Timestamo
							index, // Variable index
							u.f ); // Value (always as float)
}

// Prepares an the mavlink protocol version message
void mavlink_prepare_protocol_version( mavlink_message_t* msg ) {
	mavlink_msg_protocol_version_pack(
		mavlink_system.sysid, mavlink_system.compid, msg, MAVLINK_VERSION_MAX,
		MAVLINK_VERSION_MIN, MAVLINK_VERSION_MAX, &blank_array_[0],
		(uint8_t*)GIT_VERSION_MAVLINK_STR );
}

// Prepares an autopilot version details
void mavlink_prepare_autopilot_version( mavlink_message_t* msg ) {
	// TODO: Update capabilities
	const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET | MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET | MAV_PROTOCOL_CAPABILITY_MAVLINK2;

	mavlink_msg_autopilot_version_pack(
		mavlink_system.sysid, mavlink_system.compid, msg, capabilities,
		get_param_uint( PARAM_VERSION_FIRMWARE ), 0,
		get_param_uint( PARAM_VERSION_SOFTWARE ),
		0, // XXX: get_param_uint(PARAM_BOARD_REVISION),
		&blank_array_[0], &blank_array_[0], &blank_array_[0], system_vendor_id(),
		system_product_id(), system_unique_id(), &blank_array_uid_[0] );
}

// Sends a command acknowledgement
void mavlink_prepare_command_ack( mavlink_message_t* msg, uint16_t command,
								  uint8_t result, uint8_t sender_sysid,
								  uint8_t sender_compid, uint8_t progress ) {
	mavlink_msg_command_ack_pack( mavlink_system.sysid, mavlink_system.compid, msg,
								  command, result, progress, 0xff, sender_sysid,
								  sender_compid );
}

// Sends the requested parameter
void mavlink_prepare_param_value( mavlink_message_t* msg, uint32_t index ) {
	// char param_name[PARAMS_NAME_LENGTH];
	bool param_ok = false;

	union {
		float f;
		int32_t i;
		uint32_t u;
	} u; // The bytes are translated to the right unit on receiving, but need to
	// be sent as a "float"

	switch ( _params.types[index] ) {
	case MAVLINK_TYPE_UINT32_T: {
		u.u = get_param_uint( index );
		param_ok = true;

		break;
	}
	case MAVLINK_TYPE_INT32_T: {
		u.i = get_param_int( index );
		param_ok = true;

		break;
	}
	case MAVLINK_TYPE_FLOAT: {
		u.f = fix16_to_float( get_param_fix16( index ) );
		param_ok = true;

		break;
	}
	default: { break; }
	}

	if ( param_ok ) {
		mavlink_msg_param_value_pack( mavlink_system.sysid, mavlink_system.compid,
									  msg,
									  &_param_names[index][0], // String of name
									  u.f,					   // Value (always as float)
									  _params.types[index],	// From MAV_PARAM_TYPE
									  PARAMS_COUNT,			   // Total number of parameters
									  index );
	} else {
		char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
		snprintf( text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN,
				  "[PARAM] Unknown paramater type found: %lu", (unsigned long)index );
		mavlink_queue_broadcast_error( text );
	}
}

void mavlink_prepare_home_position( mavlink_message_t* msg ) {
	// XXX: Just give a false message to have it handled if it is requested
	float blank_quaternion[4] = {1.0, 0.0, 0.0, 0.0};

	mavlink_msg_home_position_pack( mavlink_system.sysid, mavlink_system.compid,
									msg, 0, 0, 0,  // Lat, long, alt
									0.0, 0.0, 0.0, // X (m), Y (m), Z (m)
									&blank_quaternion[0], 0.0, 0.0,
									0.0, // Appraoch vector (m)
									system_micros() );
}

/*
void mavlink_prepare_scaled_pressure(mavlink_message_t *msg) {
        mavlink_msg_scaled_pressure_pack(mavlink_system.sysid,
                                                                   mavlink_system.compid,
                                                                   msg,
                                                                   _sensors.baro.status.time_read,
                                                                   _sensors.baro.raw_press,
//TODO: Should be a scaled value
                                                                   0.0,
//diff pressure
                                                                   _sensors.baro.raw_temp);
//TODO: Should be a scaled value
}
*/

#ifdef __cplusplus
}
#endif
