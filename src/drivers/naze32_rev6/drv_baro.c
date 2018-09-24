#include "drv_baro.h"
#include "drv_bmp280.h"
#include "sensor.h"

bool drv_sensors_baro_init(void) {
	return bmp280Init( 6 );	//Naze32 Rev.6
}
