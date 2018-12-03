#ifdef __cplusplus
extern "C" {
#endif

#include "calibration.h"
#include "sensors.h"
#include "mavlink_system.h"
#include "drivers/drv_status_io.h"

#include "estimator.h"
#include "fix16.h"
#include "fixextra.h"
#include "params.h"

calibration_data_t _calibrations;
sensor_readings_t _sensors;

bool calibrate_level_horizon(void) {
	bool failed = false;

	qf16 q_lh;
	estimator_calc_lvl_horz(&q_lh);

	set_param_fix16( PARAM_EST_LEVEL_HORIZON_W, q_lh.a );
	set_param_fix16( PARAM_EST_LEVEL_HORIZON_X, q_lh.b );
	set_param_fix16( PARAM_EST_LEVEL_HORIZON_Y, q_lh.c );
	set_param_fix16( PARAM_EST_LEVEL_HORIZON_Z, q_lh.d );

	calibration_done();

	return !failed;
}

#ifdef __cplusplus
}
#endif
