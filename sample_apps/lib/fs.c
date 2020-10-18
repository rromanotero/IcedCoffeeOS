#include "syscall/syscall.h"
#include "fs.h"

FATDirectory directory_entries(uint8_t *path) {
    FATDirectory dir;
    syscall(SyscallDirContent, (uint32_t)&dir, (uint32_t)(path), (uint32_t)0, (uint32_t)0);
    return dir;
}

uint32_t open_file(FATFile *file, char *fileName) {
    uint32_t status;
    syscall(SyscallOpenFile, (uint32_t)&status, (uint32_t)file, (uint32_t)fileName, (uint32_t)0);
    return status;
}

void read_file(FATFile *file, uint8_t *buffer){
    syscall(SyscallReadFile, (uint32_t)file, (uint32_t)buffer, (uint32_t)0, (uint32_t)0);
}
