#ifndef USER_PROCESS_H_
#define USER_PROCESS_H_

#include <stdint.h>

#define PROC_SUCCESS 1
#define PROC_ERROR   0
#define PROC_NOT_FOUND 127

typedef uint32_t(*MainFunc)(uint32_t, uint8_t**);

uint32_t run_process(uint32_t argc, uint8_t **argv);

#endif
