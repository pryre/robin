#include "drivers/opencm3_naze32_common/drv_clock_select.h"
#include <libopencm3/stm32/rcc.h>

void hs_clock_select( void ) {
	rcc_clock_setup_in_hse_12mhz_out_72mhz();
}
