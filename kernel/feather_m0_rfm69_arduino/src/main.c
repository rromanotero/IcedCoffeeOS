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

extern uint32_t tick_count;

void task_handler2(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(int i=0; i<24000; i++);
  }
}

void task_handler1(void){
  while(true){
    hal_io_serial_puts(&serial_usb, "hey\n");
    for(int i=0; i<24000; i++);
  }
}

void ARDUINO_MAIN() {
  system_init();

  while(!hal_io_serial_is_ready(&serial_usb));
	hal_io_serial_puts(&serial_usb, "Here we go..");
	hal_cpu_delay(1000);


	scheduler_init();

  char buf[20];


  while(true){
    sprintf(buf, "tick count=%d\n\r", tick_count);
    hal_io_serial_puts(&serial_usb, buf);
    hal_cpu_delay(1000);
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
