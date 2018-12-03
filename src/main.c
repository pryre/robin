#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <run.h>

int main(void) {
	setup();

	while(true)
		loop();
}

#ifdef __cplusplus
}
#endif
