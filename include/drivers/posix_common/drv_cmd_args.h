#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	//enum { CHARACTER_MODE, WORD_MODE, LINE_MODE } mode;
	//bool isCaseInsensitive;
	unsigned int bind_port;
	char remote_host[100];
	unsigned int remote_port;
} arguments_t;

extern arguments_t _arguments;

void parse_arguments(arguments_t* arguments, int argc, char** argv);

#ifdef __cplusplus
}
#endif
