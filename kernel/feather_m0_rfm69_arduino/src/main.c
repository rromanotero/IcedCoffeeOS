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

tPioPin led_pin;
tSerialPort serial_usb;

void tick_callback(){
  hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
  hal_io_serial_puts(&serial_usb, "tick\n\r");
}

void ARDUINO_MAIN() {

  hal_io_pio_create_pin(&led_pin, PioA, 8, PioOutput);
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);
  hal_cpu_systimer_start(1000, tick_callback);

  while(true){
    hal_io_serial_puts(&serial_usb, "Still here\n\r");
    hal_cpu_delay(1400);
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
