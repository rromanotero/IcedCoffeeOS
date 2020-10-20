/**
 * @file    syscalls.c
 * @author
 * @version
 *
 * @brief System Calls Interface
 *
 */
#include <stddef.h>
#include <stdbool.h>
#include "hal.h"
#include "syscalls.h"
#include "kprintf.h"

//extern void disable_irq(void);
//extern void enable_irq(void);
void syscalls_entry_point(void);

/**
*   Syscalls Init
*
*   Initializes the system calls interface
*
*/
void syscalls_init(void){
    //Starts SVCs (registers SVC callback)
    hal_cpu_svc_start( syscalls_entry_point );
}

/**
*   Syscalls Entry Point
*
*   This is the execution entry point of a system call. That is, where execution
*   goes on SVC call.
*
*   Important: We assume the processor is always in user mode when executing SVC.
*              (Not sure what happens if a syscall is triggered from kernel mode )
*
*
*/

uint32_t arg0, arg1, arg2, arg3, syscall_num;

void syscalls_entry_point(void){
    // -- Get arguments and Return address in R7--
    //   It has to be placed right here, before r0-r3 get messed up
    asm volatile (  "mov %[arg0], x0 \n"
                    "mov %[arg1], x1 \n"
                    "mov %[arg2], x2 \n"
                    "mov %[arg3], x3 \n"
                    "mov %[syscall_num], x8 \n"
                    : // output
                      [arg0] "=r" (arg0),
                      [arg1] "=r" (arg1),
                      [arg2] "=r" (arg2),
                      [arg3] "=r" (arg3),
                      [syscall_num] "=r" (syscall_num)
                    : // no input
                    : "x0","x1","x2","x3","x8" // Clobber list
                );

    kprintf_debug( " == \n\n Attending syscall num %d \n\n ===", syscall_num );
    kprintf_debug( " == \n\n param0=%d, param1=%d, param2=%d, param3=%d,  \n\n ===", arg0, arg1, arg2, arg3 );

    //attend syscall
    switch(syscall_num){
        case SVCDummy:
          *((uint32_t*)arg0) = arg1 + arg2 + arg3;
           break;

        //Error
        default:
            break;
    }
}
