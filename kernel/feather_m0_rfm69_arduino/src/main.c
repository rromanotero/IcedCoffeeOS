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

void distance_kthread(void){

  DFRobotVL53L0X vl53l0_front;
  DFRobotVL53L0X vl53l0_middle_left;
  DFRobotVL53L0X vl53l0_middle_right;
  DFRobotVL53L0X vl53l0_back_left;
  DFRobotVL53L0X vl53l0_back_right;

  tPioPin d_sensor_front_pin;
  tPioPin d_sensor_middle_left_pin;
  tPioPin d_sensor_middle_right_pin;
  tPioPin d_sensor_back_left_pin;
  tPioPin d_sensor_back_right_pin;

  pio_create_pin(&d_sensor_front_pin, PioB, 9, PioOutput);
  pio_create_pin(&d_sensor_middle_left_pin, PioA, 20, PioOutput);
  pio_create_pin(&d_sensor_middle_right_pin, PioB, 2, PioOutput);
  pio_create_pin(&d_sensor_back_left_pin, PioA, 4, PioOutput);
  pio_create_pin(&d_sensor_back_right_pin, PioA, 5, PioOutput);

  //Reset all sensors
  pio_write(&d_sensor_front_pin, false);
  pio_write(&d_sensor_middle_left_pin, false);
  pio_write(&d_sensor_middle_right_pin, false);
  pio_write(&d_sensor_back_left_pin, false);
  pio_write(&d_sensor_back_right_pin, false);

  hal_cpu_delay(10);

  Wire.begin();

  // -- Bring them ON with a new adddress one by one --
  pio_write(&d_sensor_front_pin, true); //enable
  vl53l0_front.begin(0x10);
  vl53l0_front.setMode(Continuous,Low);
  vl53l0_front.start();

  pio_write(&d_sensor_middle_left_pin, true); //enable
  vl53l0_middle_left.begin(0x11);
  vl53l0_middle_left.setMode(Continuous,Low);
  vl53l0_middle_left.start();

  pio_write(&d_sensor_middle_right_pin, true); //enable
  vl53l0_middle_right.begin(0x12);
  vl53l0_middle_right.setMode(Continuous,Low);
  vl53l0_middle_right.start();

  pio_write(&d_sensor_back_left_pin, true); //enable
  vl53l0_back_left.begin(0x13);
  vl53l0_back_left.setMode(Continuous,Low);
  vl53l0_back_left.start();

  pio_write(&d_sensor_back_right_pin, true); //enable
  vl53l0_back_right.begin(0x14);
  vl53l0_back_right.setMode(Continuous,Low);
  vl53l0_back_right.start();

  hal_cpu_delay(100);

  while(true){
    //Get distances
    kprintf_debug("Front: %d \n\r", (uint32_t)(vl53l0_front.getDistance()));
    kprintf_debug("Middle left: %d \n\r", (uint32_t)(vl53l0_middle_left.getDistance()));
    kprintf_debug("Middle right: %d \n\r", (uint32_t)(vl53l0_middle_right.getDistance()));
    kprintf_debug("Back left: %d \n\r", (uint32_t)(vl53l0_back_left.getDistance()));
    kprintf_debug("Back right: %d \n\r", (uint32_t)(vl53l0_back_right.getDistance()));

    light_intensity =  (uint32_t)(vl53l0_back_right.getAmbientCount());

    hal_cpu_delay(50);
  }

}


void led_blink_kthread(void){

  tPioPin led_pin_r;
  tPioPin led_pin_g;
  tPioPin led_pin_b;

  //wait for light reads to begin
  while(light_intensity==0);

  //
  //Each pin has a 2.2K resistor
  //
  pio_create_pin(&led_pin_r, PioA, 18, PioOutput);
  pio_create_pin(&led_pin_g, PioA, 16, PioOutput);
  pio_create_pin(&led_pin_b, PioA, 19, PioOutput);

  while(true){

    if(light_intensity > 1000){
      //red
      pio_write(&led_pin_r, true);
      pio_write(&led_pin_g, false);
      pio_write(&led_pin_b, false);
    }
    else if(light_intensity > 50){
      //Purple
      pio_write(&led_pin_r, true);
      pio_write(&led_pin_g, false);
      pio_write(&led_pin_b, true);
    }
    else if(light_intensity > 5){
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

    hal_cpu_delay(100+light_intensity);

    pio_write(&led_pin_r, false);
    pio_write(&led_pin_g, false);
    pio_write(&led_pin_b, false);

    hal_cpu_delay(100+light_intensity);

  }

}


void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( led_blink_kthread, "led_blink_kthread", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( distance_kthread, "distance_kthread", 4096, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
