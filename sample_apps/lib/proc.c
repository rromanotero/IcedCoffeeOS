#include "syscall/syscall.h"
#include "proc.h"

// Runs the argv[0] process' main function
uint32_t run_process(uint32_t argc, uint8_t **argv) {
    uint32_t success;
    MainFunc proc_main;
    syscall(SyscallLoadProcBinEntry, &success, (uint32_t)argv[0], (uint32_t)(&proc_main), (uint32_t)0);
    if (!success) {
        printf("Executable not found: %s\n", argv[0]);
        return PROC_NOT_FOUND;
    } else {
        return proc_main(argc, argv);
    }
}

