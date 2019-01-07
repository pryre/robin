#ifdef __cplusplus
extern "C" {
#endif

#include "calibration.h"
#include "drivers/drv_status_io.h"
#include "mavlink_system.h"
#include "sensors.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_interference( void ) {
	bool failed = false;

	// TODO CAL INTERFERENCE
	calibration_done();

	return !failed;
}

#ifdef __cplusplus
}
#endif
