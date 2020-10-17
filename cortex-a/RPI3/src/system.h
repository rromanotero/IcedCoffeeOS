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

#ifndef SYSTEM_H
#define SYSTEM_H

#define main(...) kernel_main ()

void system_init(void);


//
// OS Configuration here
// (edit manually)
//

//Screen
#define SYSTEM_SCREEN_BACKGROUND_COLOR       0x232424
#define SYSTEM_SCREEN_TEXT_COLOR             VIDEO_COLOR_WHITE
#define SYSTEM_SCREEN_TEXT_SIZE              1
#define SYSTEM_SCREEN_WIDTH                  1280
#define SYSTEM_SCREEN_HEIGHT                 768
#define SYSTEM_SCREEN_DEPTH                  32

//Debugging

//IMPORTANT:
//           T H I S     I S    B R O K E N
// (if you say FALSE, it will fail on the PI. Works on QEMU though.)
#define SYSTEM_DEBUG_ON               true    //change to true/false. By default debug info goes to SerialA
// ================================================


//File System
#define SYSTEM_MAX_FAT_TABLE_SIZE_IN_SECTORS    700 //larger starts to take to long to load, shorter and files are not found
#define SYSTEM_MAX_DIRECTORY_ENTRIES            500



#endif
