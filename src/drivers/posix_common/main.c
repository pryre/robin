#ifdef __cplusplus
extern "C" {
#endif

#include "run.h"
#include "drivers/posix_common/drv_cmd_args.h"

#include <stdbool.h>
#include <string.h>

int main(int argc, char **argv) {
	parse_arguments(argc, argv);

	setup();

	while(true)
		loop();
}

#ifdef __cplusplus
}
#endif
