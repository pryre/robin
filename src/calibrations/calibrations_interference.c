#include "calibration.h"
#include "sensors.h"
#include "mavlink_system.h"
#include "drv_status_io.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_interference(void) {
	bool failed = false;

	//TODO CAL INTERFERENCE
	calibration_done();

	return !failed;
}
