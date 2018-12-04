#ifdef __cplusplus
extern "C" {
#endif

#include <run.h>
#include "drivers/posix_common/drv_cmd_args.h"

#include <stdbool.h>
#include <string.h>

arguments_t _arguments;

int main(int argc, char **argv) {

	_arguments.bind_port = 14555;
	strncpy(_arguments.remote_host, "127.0.0.1", 100);
	_arguments.remote_port = 14550;

	parse_arguments(&_arguments, argc, argv);

	setup();

	while(true)
		loop();
}

#ifdef __cplusplus
}
#endif
