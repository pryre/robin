#include "drivers/drv_sensors.h"
#include "drivers/naze32_common/drv_hmc5883l.h"
#include "sensors.h"

bool drv_sensors_mag_init( uint32_t i2c ) {
	return hmc5883lInit( 6 ); // Naze32 Rev.6
}
