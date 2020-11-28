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

void main_user_thread(void){

  scheduler_thread_create( thread_a, "thread_a", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( thread_b, "thread_b", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( thread_led, "thread_led", 1024, ProcQueueReadyRealTime );

  while(true){
    //hal_io_serial_puts(&serial_usb, "Main Thread (LED is in its own thread)\n\r");
    for(volatile int i=0; i<480000*5;i++);
  }
}

void thread_a(void){
  tIcedQTopic topic;
  topic.name = "system";
  topic.encoding = IcedEncodingString;

  uint8_t* raw_message = (uint8_t*)"hey there!\n";


  while(true){
    for(volatile int i=0; i<480000*5;i++);

    //publish
    hal_io_serial_puts(&serial_usb, "Publishing to IcedQ\n\r");
    icedq_publish(&topic, "", raw_message, 11);
  }
}

uint8_t buffer[100];

void thread_b(void){
  tIcedQTopic topic;
  topic.name = "system";
  topic.encoding = IcedEncodingString;

  tIcedQQueue in_queue;
  in_queue.queue = buffer;
  in_queue.head = 0;
  in_queue.tail = 0;
  in_queue.capacity = 100;

  icedq_subscribe(&topic, "", &in_queue);

  while(true){
    //wait for data
    if( in_queue.tail - in_queue.head > 0){
        for(volatile int i=0; i<480000*2;i++);

        //consume messages
        uint8_t item =  in_queue.queue[in_queue.head];
        in_queue.head = (in_queue.head + 1) % in_queue.capacity;

        Serial.print("\n\r Consumed:");
        //Serial.print(item);
        Serial.print("\n\r");
        hal_io_serial_putc(&serial_usb, item);
    }
  }
}

void thread_led(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<480000;i++);
  }
}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  while(!hal_io_serial_is_ready(&serial_usb));

  scheduler_thread_create( main_user_thread, "main_user_thread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
