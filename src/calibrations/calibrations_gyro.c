#include "calibration.h"
#include "sensors.h"
#include "mavlink_system.h"
#include "drv_status_io.h"

#include "estimator.h"
#include "params.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_gyro(void) {
	bool failed = false;

	_calibrations.data.gyro.sum_x += _sensors.imu.gyro_raw.x;
	_calibrations.data.gyro.sum_y += _sensors.imu.gyro_raw.y;
	_calibrations.data.gyro.sum_z += _sensors.imu.gyro_raw.z;

	_calibrations.data.gyro.count++;

	if (_calibrations.data.gyro.count >= get_param_uint(PARAM_CAL_IMU_PASSES)) {
		int gyro_x_bias = _calibrations.data.gyro.sum_x / _calibrations.data.gyro.count;
		int gyro_y_bias = _calibrations.data.gyro.sum_y / _calibrations.data.gyro.count;
		int gyro_z_bias = _calibrations.data.gyro.sum_z / _calibrations.data.gyro.count;

		if( ( abs(gyro_x_bias) > CAL_GYRO_HIGH_BIAS ) ||
			( abs(gyro_y_bias) > CAL_GYRO_HIGH_BIAS ) ||
			( abs(gyro_z_bias) > CAL_GYRO_HIGH_BIAS ) ) {
			mavlink_queue_broadcast_error("[SENSOR] Warning: high gyro biases detected!");
		}

		set_param_int(PARAM_GYRO_X_BIAS, gyro_x_bias);
		set_param_int(PARAM_GYRO_Y_BIAS, gyro_y_bias);
		set_param_int(PARAM_GYRO_Z_BIAS, gyro_z_bias);

		reset_adaptive_gyro_bias();

		calibration_done();
		mavlink_queue_broadcast_notice("[SENSOR] Gyro calibration complete!");
	}

	return !failed;
}
