#include "drivers/drv_sensors.h"
#include "drivers/naze32_common/drv_bmp280.h"
#include "sensors.h"

bool drv_sensors_baro_init( uint32_t i2c ) {
	return bmp280Init( 5 ); // Naze32 Rev.6
}
