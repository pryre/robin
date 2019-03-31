#include <stdbool.h>

#include "drivers/drv_ppm.h"
#include "drivers/drv_sensors.h"

bool drv_sensors_rc_input_init( void ) {
	return drv_ppm_init();
}

bool drv_sensors_rc_input_read( uint16_t* readings ) {
	return drv_ppm_read_frame(readings);
}
