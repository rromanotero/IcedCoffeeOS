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

#define RECEIVER_ADDRESS  0
#define TRANSMITTER_ADDRESS  1

tRadioMessage message;  //Must be static

void ARDUINO_KERNEL_MAIN() {

  tPioPin led_pin;
  tRadioTransceiver radio;

  hal_io_pio_create_pin(&led_pin, PioA, 8, PioOutput);
  hal_radio_create_transceiver(&radio, RadioA, TRANSMITTER_ADDRESS, HAL_RADIO_TX_POWER_MAX/2);

  message.address = RECEIVER_ADDRESS;
  message.len = 10;
  message.payload[0] = 'h';
  message.payload[1] = 'e';
  message.payload[2] = 'y';
  message.payload[3] = ' ';
  message.payload[4] = 't';
  message.payload[5] = 'h';
  message.payload[6] = 'e';
  message.payload[7] = 'r';
  message.payload[8] = 'e';
  message.payload[9] = '\0';

  while(true){
    hal_radio_write(&radio, &message);
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));

    hal_cpu_delay(1000);
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
