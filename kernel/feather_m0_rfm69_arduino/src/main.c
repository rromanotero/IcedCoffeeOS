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

void thread_led(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<480000;i++);
  }
}

void main_user_thread(void){

  scheduler_thread_create( thread_led, "thread_led", 1024 );

  while(true){
    hal_io_serial_puts(&serial_usb, "main user thread\n\r");
    for(volatile int i=0; i<480000;i++);
  }
}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  while(!Serial);
  delay(1000);

  scheduler_thread_create( main_user_thread, "main_user_thread", 1024 );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
