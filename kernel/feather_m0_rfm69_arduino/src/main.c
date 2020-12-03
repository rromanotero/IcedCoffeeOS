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
#include "hal.h"

extern tPioPin led_pin;         //Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

void sample_kthread(void){

  while(!hal_io_serial_is_ready(&serial_usb)); /// <<--- WAIT FOR USER

  uint8_t raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t request_num;
  tSyscallInput input;
  tSyscallOutput output;

  while(true){
    request_num = 17;

    input.arg0 = 1;
    input.arg1 = 2;
    input.arg2 = 3;
    input.arg3 = 4;

    kprintf_debug("BEFORE input.arg2=%d",input.arg2);

    raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET] = request_num;
    raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+0] = ((uint32_t)(&input)>>8*0) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+1] = ((uint32_t)(&input)>>8*1) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+2] = ((uint32_t)(&input)>>8*2) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+3] = ((uint32_t)(&input)>>8*3) & 0xff;

    raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET] = request_num;
    raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+0] = ((uint32_t)(&output)>>8*0) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+1] = ((uint32_t)(&output)>>8*1) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+2] = ((uint32_t)(&output)>>8*2) & 0xff;
    raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+3] = ((uint32_t)(&output)>>8*3) & 0xff;


    kprintf_debug("AFTER input.arg2=%d",input.arg2);
    kprintf_debug("RAW REQUEST:");
    for(int i=0; i<9; i++)
      kprintf_debug("%x ", raw_request[i]);
    kprintf_debug("\n\r");

    kprintf_debug("Making syscall. Request num: %d, input addr: %x, output addr: %x \n\r",
                  request_num,
                  &input,
                  &output
                );

    kprintf_debug("Reconstructed addresses. input addr: %x, output addr: %x \n\r",
                  (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+3]<< 8*3),
                  (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+3]<< 8*3)
    );

    tSyscallInput* in = &input;//(tSyscallInput*)((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_INPUT_OFFSET];
    tSyscallOutput* out = &output;//(tSyscallOutput*)((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET];

    kprintf_debug("Reconstructed PARAMS. arg0=%d, arg1=%d, arg2=%d, arg3=%d, \n\r\n\r",
                  in->arg0, in->arg1, in->arg2, in->arg3
    );


    icedq_publish("system.syscalls", raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

    //blink away...
    //hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<4800;i++);
  }
}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( sample_kthread, "sample_kthread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
