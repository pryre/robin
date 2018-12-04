#ifdef __cplusplus
extern "C" {
#endif

#include <argp.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "drivers/posix_common/drv_cmd_args.h"

const char *argp_program_version = GIT_VERSION_FLIGHT_STR;
const char *argp_program_bug_address = "<pryre.dev@outlook.com>";
static char doc[] = "Robin POSIX flight control software. ";
static char args_doc[] = "";
static struct argp_option options[] = { 
	{ "bind-port", 'b', 0, 0, "Bind port to use use for comm port 1 connection.", 0},
	{ "remote-host", 'h', 0, 0, "Remote host to use use for comm port 1 connection.", 0},
	{ "remote-port", 'r', 0, 0, "Remote port to use use for comm port 1 connection.", 0},
	{ 0 } 
};

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
	arguments_t *arguments = state->input;
	switch (key) {
	case 'b': arguments->bind_port = atoi(arg); break;
	case 'h': strncpy(arguments->remote_host, arg, 100); break;
	case 'r': arguments->remote_port = atoi(arg); break;
	case ARGP_KEY_ARG: return 0;
	default: return ARGP_ERR_UNKNOWN;
	}   
	return 0;
}

static struct argp argp = { options, parse_opt, args_doc, doc, 0, 0, 0 };


void parse_arguments(arguments_t* arguments, int argc, char** argv) {
	//TODO: argp_parse(&argp, argc, argv, 0, 0, &arguments);
}