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


extern tPioPin led_pin;         //Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

extern uint32_t tick_count_a;

void task_handler2(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(int i=0; i<24000; i++);
  }
}

void task_handler1(void){
  while(true){
    Serial.println("Hey");
    Serial.println(tick_count_a);

    //hal_io_serial_puts(&serial_usb, "hey\n");
    for(int i=0; i<24000; i++);
  }
}

void ARDUINO_MAIN() {
  system_init();
  //scheduler_thread_create( task_handler1, "task_handler1", 512 );

  bool status;

  /* Initialize task stacks: */
	static uint32_t stack1[512];
	static uint32_t stack2[512];
	static uint32_t stack3[512];


  /* Setup task parameters: */
	uint32_t p1 = 200000;
	uint32_t p2 = p1/2;
	uint32_t p3 = p1/4;

	os_init();

	//status = os_task_init(&task_handler1, stack1, sizeof(stack1));
	//status = os_task_init(&task_handler2, stack2, sizeof(stack2));

	/* Context switch every second: */
  //Serial.println("Begin scheduler");
	//status = os_start(3);

  scheduler_thread_create( task_handler1, "task_handler1a", 1024 );
  scheduler_thread_create( task_handler1, "task_handler1b", 1024 );

  while(true){

  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
