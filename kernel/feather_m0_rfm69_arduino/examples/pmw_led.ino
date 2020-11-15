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

  tPwmChannel pwm;
  hal_io_pwm_create_channel(&pwm, PwmA);

  while(true){
      for(uint32_t i=0; i<=100; i++){
        hal_io_pwm_write(&pwm, i);
        hal_cpu_delay(20);
      }
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
