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
#include <stddef.h>
#include "loader.h"
#include "kprintf.h"

#define MEM_ALIGN 0x1000 // IDK why it has to be aligned to this (foud it experimentally)
                         //for this to work
                         //Looking all I was able to fins is that's the size of a page
                         //in the PI's CPU, but not sure how that relates to this.

static uint8_t* process_address_space = 0x80000+0x4000000; //Right at the end of the OS address space


uint32_t loader_load_bin(uint8_t* buffer) {
    /*
      Unimplemented.... here, just load to mem

    */

    return 0;
}
