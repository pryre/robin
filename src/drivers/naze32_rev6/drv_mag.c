#include "drv_baro.h"
#include "drv_hmc5883l.h"
#include "sensor.h"

bool drv_sensors_mag_init(void) {
	return hmc5883lInit( 6 );	//Naze32 Rev.6
}
