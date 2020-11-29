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

  uint8_t* raw_message = (uint8_t*)"hey there!";

  for(volatile int i=0; i<480000*10;i++);

  while(true){
    for(volatile int i=0; i<4800;i++);

    //publish
    //hal_io_serial_puts(&serial_usb, "Publishing to IcedQ\n\r");
    icedq_publish(&topic, "", raw_message, 10);
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


  uint32_t counter = 0;
  while(true){

  //  if(counter++ % 1000 == 0){
  //    Serial.println("CONSUMER: Waiting to consume");
  //  }

    volatile int32_t head = in_queue.head;
    volatile int32_t tail = in_queue.tail;
    volatile int32_t fixed_tail = tail % in_queue.capacity;
    volatile int32_t fixed_head = head % in_queue.capacity;
    char items[100];

    if(tail - head > 0){

      Serial.println("CONSUMER: Found this many elements in queue:");
      Serial.println(tail-head);

      Serial.println("CONSUMER: fixed tail:");
      Serial.println(fixed_tail);

      Serial.println("CONSUMER: fixed head");
      Serial.println(fixed_head);

      Serial.println("CONSUMER: real tail:");
      Serial.println(tail);

      Serial.println("CONSUMER: real head");
      Serial.println(head);

      //wait for data
      for(int i=0; i< tail-head; i++){
          //consume messages
          items[i] =  in_queue.queue[fixed_head+i];
      }

      //Update head atomically
      in_queue.head = in_queue.head + (tail-head);


      Serial.println("CONSUMER: consumed the items: ");
      for(int j=0; j<(tail-head); j++){
        Serial.write(items[j]);
      }
      Serial.println("");
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
