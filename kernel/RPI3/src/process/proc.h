#ifndef PROCESS_H_
#define PROCESS_H_

#define PROC_SUCCESS 1
#define PROC_ERROR   0

typedef uint32_t(*MainFunc)(uint32_t, uint8_t**);

uint32_t proc_load_bin(uint8_t *filename_full, MainFunc *out_main);

#endif
