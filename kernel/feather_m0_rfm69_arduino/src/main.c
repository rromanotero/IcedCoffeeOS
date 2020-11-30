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
  scheduler_thread_create( thread_b, "thread_b", 2048, ProcQueueReadyRealTime );

  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<480000*2;i++);
  }
}

void thread_a(void){

  //Let consumer go up
  for(volatile int i=0; i<480000*11;i++);

  uint32_t counter = 0;

  while(true){

    //Create message
    //hey0, hey1, hey2, ...
    uint8_t raw_message[] = {'h','e','y','X'};
    raw_message[3] = counter + '0';
    counter = (counter+1)%10;

    //Publish it
    icedq_publish("dummy_topic", raw_message, 4);

    for(volatile int i=0; i<4800;i++);
  }
}

uint8_t buffer[100];
char items[100];

void thread_b(void){

  //Init queue where Producer will publish to
  tIcedQQueue in_queue;
  in_queue.queue = buffer;
  in_queue.head = 0;
  in_queue.tail = 0;
  in_queue.capacity = 100;

  //Subscribe to topic
  icedq_subscribe("dummy_topic", &in_queue);

  while(true){

    volatile uint32_t head = in_queue.head;
    volatile uint32_t tail = in_queue.tail;

    uint32_t bytes_to_read;
    if(head > tail){
      //tail went around
      bytes_to_read = (in_queue.capacity - head) + tail;
    }
    else{
      bytes_to_read = (tail - head);
    }

    if(bytes_to_read > 0){

      Serial.println("CONSUMER: Found this many elements in queue:");
      Serial.println(bytes_to_read);

      Serial.println("CONSUMER: tail:");
      Serial.println(tail);

      Serial.println("CONSUMER: head");
      Serial.println(head);

      for(int i=0; i< bytes_to_read; i++){
          //consume messages
          items[i] = in_queue.queue[in_queue.head];
          in_queue.head = (in_queue.head + 1) % in_queue.capacity;
      }

      Serial.println("CONSUMER: New value of head");
      Serial.println(in_queue.head);

      Serial.println("CONSUMER: consumed the items: ");
      for(int j=0; j<(bytes_to_read); j++){
        Serial.write(items[j]);
      }
      Serial.println("");
    }
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
