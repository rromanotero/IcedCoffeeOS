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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system.h"
#include "hal.h"
#include "fat.h"
#include "proc.h"
#include "kprintf.h"

void print_n_chars(uint8_t*, uint32_t);
void display_image(void);
void display_alice(void);

uint8_t buffer[240000]; //107KB (Recall there's 3 pixels per byte + headers)

extern uint32_t user_program_main(uint32_t, uint8_t**);

void main(){

    system_init();

    //wait for any key to begin
    // This is needed because when running on the PI, attempting to read the SD CArd
    // right away upon start causes read to fail. (A small delay would do the trick
    // as well
    //kprintf( "Press anything to continue...\n" );
    //kprintf( "\n" );
    //hal_io_serial_getc(SerialA);

    // Test running the shell
    MainFunc proc_main;
    if(proc_load_bin("APP", &proc_main) == PROC_SUCCESS){
        //Run
        uint8_t** argv = NULL;
        uint32_t argc = 0;
        uint32_t proc_ret = proc_main(argc, argv);

        kprintf("\n\nAPP EXITED WITH VALUE %d\n", proc_ret);
    } else {
        hal_video_puts("\nNOT FOUND\n", 2, VIDEO_COLOR_RED);
    }

    hal_video_puts("\nEND OF KERNEL MAIN\n", 2, VIDEO_COLOR_GREEN);

    while(1);
}
