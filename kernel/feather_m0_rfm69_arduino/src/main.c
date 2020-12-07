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

tPioPin led_pin_onboard;
tPioPin led_pin_r;
tPioPin led_pin_g;
tPioPin led_pin_b;

void sample_kthread(void){
  //
  //Each pin has a 2.2K resistor
  //
  pio_create_pin(&led_pin_onboard, PioA, 8, PioOutput);
  pio_create_pin(&led_pin_r, PioA, 18, PioOutput);
  pio_create_pin(&led_pin_g, PioA, 16, PioOutput);
  pio_create_pin(&led_pin_b, PioA, 19, PioOutput);

  //Light sensor
  tAdcChannel light_sensor_adc;
  hal_io_adc_create_channel(&light_sensor_adc, AdcA, IoPoll);

  while(true){
    pio_write(&led_pin_onboard, !pio_read(&led_pin_onboard));
    pio_write(&led_pin_r, true);
    pio_write(&led_pin_g, false);
    pio_write(&led_pin_b, false);

    kprintf_debug("\n\rLight Sensor = %d \n\r", hal_io_adc_read(&light_sensor_adc));

    hal_cpu_delay(1000);

    pio_write(&led_pin_onboard, !pio_read(&led_pin_onboard));
    pio_write(&led_pin_r, true);
    pio_write(&led_pin_g, true);
    pio_write(&led_pin_b, true);

    kprintf_debug("\n\rLight Sensor = %d \n\r", hal_io_adc_read(&light_sensor_adc));

    hal_cpu_delay(1000);


  }

}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( sample_kthread, "sample_kthread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
