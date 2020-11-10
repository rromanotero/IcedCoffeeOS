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

void ARDUINO_MAIN() {

  tServoChannel servo_a;
  tServoChannel servo_b;
  hal_io_servo_create_channel(&servo_a, ServoA);
  hal_io_servo_create_channel(&servo_b, ServoB);

  while(true){
    for (int pos = 90; pos <=180; pos += 1) {
      hal_io_servo_write(&servo_a, pos);
      hal_io_servo_write(&servo_b, pos);
      hal_cpu_delay(15);
    }
    for (int pos = 180; pos >= 90; pos -= 1) {
      hal_io_servo_write(&servo_a, pos);
      hal_io_servo_write(&servo_b, pos);
      hal_cpu_delay(15);
    }

  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
