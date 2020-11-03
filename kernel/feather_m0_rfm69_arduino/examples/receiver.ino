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

#include <stdbool.h>
#include <stdint.h>
#include "hal.h"

#define RECEIVER_ADDRESS  0
#define TRANSMITTER_ADDRESS  1

void ARDUINO_MAIN() {

  tSerialPort serial_usb;
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);

  while(!hal_io_serial_is_ready(&serial_usb));

  tRadioTransceiver radio;
  tRadioMessage message;
  hal_radio_create_transceiver(&radio, RadioA, RECEIVER_ADDRESS, HAL_RADIO_TX_POWER_MAX/2);

  while(true){

    if( hal_radio_available(&radio) ){

      //read
      hal_radio_read(&radio, &message);

      //Send over serial
      for(int i=0; i<message.len; i++)
        hal_io_serial_putc(&serial_usb, message.payload[i]);
      hal_io_serial_puts("\n\r");
    }
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
