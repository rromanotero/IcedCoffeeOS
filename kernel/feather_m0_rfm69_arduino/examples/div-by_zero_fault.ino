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

extern tPioPin led_pin;				 //Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

void tick_callback(){
  hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
  hal_io_serial_puts(&serial_usb, "tick\n\r");
}

void ARDUINO_KERNEL_MAIN() {

  system_init();

  hal_cpu_systimer_start(1000, tick_callback);

  volatile uint32_t a = 7;
  volatile uint32_t b = 0;
  volatile uint32_t c = 0;
  while(true){
    hal_io_serial_puts(&serial_usb, "Div by Zero! 7\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 6\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 5\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 4\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 3\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 2\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 1\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 0\n\r");
    hal_cpu_delay(2000);
    c = c + a/b;
    c = c/0; ///<<----- hardfault
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
