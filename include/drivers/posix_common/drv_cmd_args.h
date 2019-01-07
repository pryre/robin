#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	char conn_telem0[100];
	char conn_telem1[100];
	char eeprom_file[1000];
} arguments_t;

extern arguments_t _arguments;

void parse_arguments( int argc, char** argv );

#ifdef __cplusplus
}
#endif
