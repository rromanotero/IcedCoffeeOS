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

extern tPioPin led_pin;
volatile int32_t light_intensity = 0;

void motor_kthread(void){

  tPioPin l298n_a;
  tPioPin l298n_b;
  tPioPin l298n_c;
  tPioPin l298n_d;

  pio_create_pin(&l298n_a, PioB, 9, PioOutput);
  pio_create_pin(&l298n_b, PioA, 4, PioOutput);
  pio_create_pin(&l298n_c, PioA, 5, PioOutput);
  pio_create_pin(&l298n_d, PioB, 2, PioOutput);

  pio_write(&l298n_a, true);
  pio_write(&l298n_b, false);
  pio_write(&l298n_c, true);
  pio_write(&l298n_d, false);


  while(true){
      kprintf_debug("Motor thread");
      hal_cpu_delay(1000);
  }

}



void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( motor_kthread, "motor_kthread", 512, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
