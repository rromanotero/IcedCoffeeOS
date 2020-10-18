#include <stdint.h>

void syscall( uint32_t syscall_num, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3   ){

    //Call the kernel
    asm volatile(
      "mov x0, %[param0]\n"
      "mov x1, %[param1]\n"
      "mov x2, %[param2]\n"
      "mov x3, %[param3]\n"
      "mov x8, %[syscall_num]\n"
      "svc 0"
      :  //output
      : [param0] "r" (param0),  //input
        [param1] "r" (param1),
        [param2] "r" (param2),
        [param3] "r" (param3),
        [syscall_num] "r" (syscall_num)
      : "x30"  //Clobber registers
    );

}
