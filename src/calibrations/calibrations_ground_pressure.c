#include "calibration.h"
#include "sensors.h"
#include "mavlink_system.h"
#include "drivers/drv_status_io.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_ground_pressure(void) {
	bool failed = false;

	//TODO CAL BAROMETER
	calibration_done();

	return !failed;
}
