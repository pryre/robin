#include "calibration.h"
#include "sensors.h"
#include "mavlink_system.h"
#include "drv_status_io.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_magnetometer(void) {
	bool failed = false;

	//TODO CAL MAG
	calibration_done();

	return !failed;
}
