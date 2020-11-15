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

void ARDUINO_KERNEL_MAIN() {

  tPioPin led_pin;
  hal_io_pio_create_pin(&led_pin, PioA, 8, PioOutput);

  tSerialPort serial_usb;
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);

  while(!hal_io_serial_is_ready(&serial_usb));

  while(true){
    uint8_t received = hal_io_serial_getc(&serial_usb);
    hal_io_serial_putc(&serial_usb, received);

    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));

    if(received=='\r')   //Putty when hitting enter
      hal_io_serial_putc(&serial_usb, '\n');
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
