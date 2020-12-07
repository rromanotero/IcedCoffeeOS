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

tPioPin led_pin_r;
tPioPin led_pin_g;
tPioPin led_pin_b;

volatile int32_t light_intensity;
volatile int32_t inverse_light_intensity;

void light_intensity_print_kthread(void){
  //  Wiring diagram
  //
  //           Vcc
  //           |
  //    PHOTO-TRANSISTOR
  //           |
  //           +------ 10K RESISTOR ---- ADC 0
  //           |
  //      560K RESISTOR
  //           |
  //          GND

  //Light sensor
  tAdcChannel light_sensor_adc;
  adc_create_channel(&light_sensor_adc, AdcA, IoPoll);

  while(true){
    light_intensity = adc_read(&light_sensor_adc);
    inverse_light_intensity = 1024 - light_intensity;

    kprintf_debug("Light Intensity = %d \n\r", light_intensity);
    kprintf_debug("Light Intensity (inversed) = %d \n\r", inverse_light_intensity);
    hal_cpu_delay(100);
  }

}

void led_blink_kthread(void){
  //
  //Each pin has a 2.2K resistor
  //
  pio_create_pin(&led_pin_r, PioA, 18, PioOutput);
  pio_create_pin(&led_pin_g, PioA, 16, PioOutput);
  pio_create_pin(&led_pin_b, PioA, 19, PioOutput);

  while(true){
    //pio_write(&led_pin, !pio_read(&led_pin));

    if(light_intensity < 60){
      //red
      pio_write(&led_pin_r, true);
      pio_write(&led_pin_g, false);
      pio_write(&led_pin_b, false);
    }
    else if(light_intensity < 70){
      //Purple
      pio_write(&led_pin_r, true);
      pio_write(&led_pin_g, false);
      pio_write(&led_pin_b, true);
    }
    else if(light_intensity < 130){
      //White
      pio_write(&led_pin_r, true);
      pio_write(&led_pin_g, true);
      pio_write(&led_pin_b, true);
    }
    else{
      //Green
      pio_write(&led_pin_r, false);
      pio_write(&led_pin_g, true);
      pio_write(&led_pin_b, false);
    }

    hal_cpu_delay(inverse_light_intensity/2);

    pio_write(&led_pin_r, false);
    pio_write(&led_pin_g, false);
    pio_write(&led_pin_b, false);

    hal_cpu_delay(inverse_light_intensity/2 );
  }

}


void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( led_blink_kthread, "led_blink_kthread", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( light_intensity_print_kthread, "light_intensity_print_kthread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
