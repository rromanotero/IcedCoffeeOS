#include <stdint.h>

enum {
    SyscallPutc = 0,
    SyscallPutPixel = 1,
    SyscallGetc = 2,
    SyscallDummy = 11,
    SyscallLoadProcBinEntry = 13,
    SyscallDirContent = 14,
    SyscallOpenFile = 15,
    SyscallReadFile = 16,
};

void syscall( uint32_t syscall_num, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3 );
