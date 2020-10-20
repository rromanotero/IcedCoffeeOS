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
#include "lib.h"

uint32_t dummy( uint32_t a, uint32_t b, uint32_t c ){
    uint32_t ret_val;

    asm volatile(
      "mov x3, %[input] \n"
      "mov x8, 11\n"  //Syscall nuimber
      "svc 0"
      : //output
      : [input] "r" (&ret_val) //input
      : "x30"  //Clobber registers
    );

    return ret_val;
}
