#include "drivers/drv_sensors.h"
#include "drivers/naze32_common/drv_bmp280.h"
#include "sensors.h"

bool drv_sensors_baro_init(void) {
	return bmp280Init( 6 );	//Naze32 Rev.6
}
