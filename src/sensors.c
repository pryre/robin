#include <stdint.h>
#include <stdbool.h>

void init_sensors(void) {

}

bool update_sensors(uint32_t time_us) {
	bool update_success = false;

	if( time_us > 0) {
		update_success = true;
	}

	return update_success;
}
