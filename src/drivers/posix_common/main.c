#ifdef __cplusplus
extern "C" {
#endif

#include <run.h>
#include "drivers/posix_common/drv_cmd_args.h"

#include <stdbool.h>
#include <string.h>

arguments_t _arguments;

int main(int argc, char **argv) {

	strncpy(_arguments.conn_telem0, "udp://:14555@:14550", 100);
	strncpy(_arguments.conn_telem1, "udp://:14556@:14551", 100);

	parse_arguments(&_arguments, argc, argv);

	setup();

	while(true)
		loop();
}

#ifdef __cplusplus
}
#endif
