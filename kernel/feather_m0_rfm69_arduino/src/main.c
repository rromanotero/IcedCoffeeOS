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

tPioPin led_left_r;
tPioPin led_left_g;
tPioPin led_left_b;

void sample_kthread(void){

  //
  //Each pin has a 2.2K resistor
  //
  hal_io_pio_create_pin(&led_left_r, PioA, 18, PioOutput);
  hal_io_pio_create_pin(&led_left_g, PioA, 16, PioOutput);
  hal_io_pio_create_pin(&led_left_b, PioA, 19, PioOutput);

  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    hal_io_pio_write(&led_left_r, true);
    hal_io_pio_write(&led_left_g, false);
    hal_io_pio_write(&led_left_b, false);
    hal_cpu_delay(1000);
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    hal_io_pio_write(&led_left_r, true);
    hal_io_pio_write(&led_left_g, true);
    hal_io_pio_write(&led_left_b, true);
    hal_cpu_delay(1000);
  }

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

    syscall_utils_raw_request_populate(raw_request, request_num, &input, &output);
    icedq_publish("system.syscalls", raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

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
