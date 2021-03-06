/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/

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
