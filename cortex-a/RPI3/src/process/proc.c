#include <stdint.h>
#include <stddef.h>
#include "fat.h"
#include "proc.h"
#include "kprintf.h"

#define MEM_ALIGN 0x1000
static uint8_t* process_address_space = 0x80000+0x4000000; //Right at the end of the OS address space

// maybe move this in some kind of util file
// Reads a bin file and puts entry point into out_main
uint32_t proc_load_bin(uint8_t *filename_full, MainFunc *out_main) {
    FATFile file;
    if(fat_file_open(&file, filename_full, "BIN") == FAT_SUCCESS){
        // Read to buffer
        fat_file_read(&file, (uint8_t*)(process_address_space));
        // Give caller address to main
        *out_main = (MainFunc) process_address_space;
        // move address space pointer so we can load another process after this one
        // yes, there's currently no way to move the pointer back, i.e. you may eventually run out of space
        process_address_space += file.size + (MEM_ALIGN - file.size % MEM_ALIGN);
        return PROC_SUCCESS;
    }

    return PROC_ERROR;
}
