#include "mavlink_system.h"
#include "mavlink_receive.h"

#include "sensors.h"

sensor_readings_t _sensors;

void mavlink_handle_hil_sensor( uint8_t port, mavlink_message_t *msg, mavlink_status_t *status ) {
	if(_sensors.hil.status.present) {
		//Accelerometer
		//XXX: TODO: THIS SHOULDN'T NEED TO BE INVERSED
		_sensors.hil.accel.x = -fix16_from_int(mavlink_msg_hil_sensor_get_xacc(msg));
		_sensors.hil.accel.y = -fix16_from_int(mavlink_msg_hil_sensor_get_yacc(msg));
		_sensors.hil.accel.z = -fix16_from_int(mavlink_msg_hil_sensor_get_zacc(msg));
		//Gyroscope
		_sensors.hil.gyro.x = fix16_from_float(mavlink_msg_hil_sensor_get_xgyro(msg));
		_sensors.hil.gyro.y = fix16_from_float(mavlink_msg_hil_sensor_get_ygyro(msg));
		_sensors.hil.gyro.z = fix16_from_float(mavlink_msg_hil_sensor_get_zgyro(msg));
		//Magnetometer
		_sensors.hil.mag.x = fix16_from_float(mavlink_msg_hil_sensor_get_xmag(msg));
		_sensors.hil.mag.y = fix16_from_float(mavlink_msg_hil_sensor_get_ymag(msg));
		_sensors.hil.mag.z = fix16_from_float(mavlink_msg_hil_sensor_get_zmag(msg));
		//Differential Pressure
		_sensors.hil.pressure_abs = fix16_from_float(mavlink_msg_hil_sensor_get_abs_pressure(msg));
		_sensors.hil.pressure_diff = fix16_from_float(mavlink_msg_hil_sensor_get_diff_pressure(msg));
		_sensors.hil.pressure_alt = fix16_from_float(mavlink_msg_hil_sensor_get_pressure_alt(msg));
		//Temperature
		_sensors.hil.temperature = fix16_from_float(mavlink_msg_hil_sensor_get_temperature(msg));

		safety_update_sensor(&_system_status.sensors.hil);
		_sensors.hil.status.new_data = true;
	}	//else no in HIL mode, discard
}
