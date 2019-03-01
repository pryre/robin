#ifdef __cplusplus
extern "C" {
#endif

#include <run.h>
#include <stdbool.h>

int main( void ) {
	setup();

	while ( true )
		loop();
}

#ifdef __cplusplus
}
#endif
