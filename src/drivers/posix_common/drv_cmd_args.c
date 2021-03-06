#ifdef __cplusplus
extern "C" {
#endif

#include <argp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "drivers/drv_system.h"
#include "drivers/posix_common/drv_cmd_args.h"

const char* argp_program_version = GIT_VERSION_FLIGHT_STR;
const char* argp_program_bug_address = "<pryre.dev@outlook.com>";
static char doc[] = "Robin POSIX flight control software. ";
static char args_doc[] = "";
static struct argp_option options[] = {
	{"telem0", '0', "udp://:@:", 0, "Connection to use for telem0.", 0},
	{"telem1", '1', "udp://:@:", 0, "Connection to use for telem1.", 0},
	{"eeprom", 'e', "FILE", 0, "File to use for the eeprom.", 0},
	{0}};

arguments_t _arguments;

static error_t parse_opt( int key, char* arg, struct argp_state* state ) {
	arguments_t* arguments = state->input;
	switch ( key ) {
	case '0':
		strncpy( arguments->conn_telem0, arg, 100 );
		break;
	case '1':
		strncpy( arguments->conn_telem1, arg, 100 );
		break;
	case 'e':
		strncpy( arguments->eeprom_file, arg, 100 );
		break;
	case ARGP_KEY_ARG:
		return 0;
	default:
		return ARGP_ERR_UNKNOWN;
	}
	return 0;
}

static struct argp argp = {options, parse_opt, args_doc, doc, 0, 0, 0};

void parse_arguments( int argc, char** argv ) {
	// Defaults
	strncpy( _arguments.conn_telem0, "udp://:14555@:14550", 100 );
	strncpy( _arguments.conn_telem1, "udp://:14556@:14551", 100 );
	strncpy( _arguments.eeprom_file, "robin_posix_eeprom.bin", 1000 );

	// Parse arguments
	argp_parse( &argp, argc, argv, 0, 0, &_arguments );
}
