/**
*   This file is part of os_labs
*   (https://github.com/rromanotero/os_labs).
*
*   Copyright (c) 2019 Rafael Roman Otero.
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

#include <stdbool.h>
#include <stdint.h>
#include "syscall/syscall.h"
#include "video.h"

/*
*       Put pixel
*/
void putx(  VideoArea* area, uint32_t colour ){
    //Syscall here
    syscall(  SyscallPutPixel, (uint32_t)area, (uint32_t)0, (uint32_t)0, (uint32_t)0 );
}
