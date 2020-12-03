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

  //scheduler_thread_create( producer, "producer", 1024, ProcQueueReadyRealTime );
  //scheduler_thread_create( consumer, "consumer", 2048, ProcQueueReadyRealTime );

  uint8_t raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t request_num;
  tSyscallInput input;
  tSyscallOutput output;

  while(true){
    request_num = 3;

    input.arg0 = 7;
    input.arg1 = 7;
    input.arg2 = 7;
    input.arg3 = 7;

    raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET] = request_num;
    ((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_INPUT_OFFSET] = (uint32_t)(&input);
    ((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET] = (uint32_t)(&output);


    kprintf_debug("\n\rMaking syscall. Request num: %d, input addr: %x, output addr: %x \n\r",
                  request_num,
                  &input,
                  &output
                );

    kprintf_debug("Reconstructed addresses. input addr: %x, output addr: %x \n\r",
                  ((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_INPUT_OFFSET],
                  ((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET]
    );

    tSyscallInput* in = &input;//(tSyscallInput*)((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_INPUT_OFFSET];
    tSyscallOutput* out = &output;//(tSyscallOutput*)((uint32_t*)raw_request)[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET];

    kprintf_debug("Reconstructed PARAMS. arg0=%d, arg1=%d, arg2=%d, arg3=%d, \n\r\n\r",
                  in->arg0, in->arg1, in->arg2, in->arg3
    );


    icedq_publish("system.syscalls", raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

    //blink away...
    //hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<480000*10;i++);
  }
}

void producer(void){
  uint32_t counter = 0;

  while(true){
    //Create message
    //hey0, hey1, hey2, ...
    uint8_t raw_message[] = {'h','e','y','X','\0','\0'};
    raw_message[3] = counter + '0';
    counter = (counter+1)%10;

    //Publish it
    icedq_publish("dummy_topic", raw_message, 4);
    kprintf_debug("Produced the items: %s \n\r", raw_message); //<<-- Print inside lock just so when printed
                                                               //     we can see a "produced" followe by a "consume"
    for(volatile int i=0; i<480;i++);
  }
}

uint8_t queue_buffer[100];
uint8_t consumed_buffer[25];

void consumer(void){

  //Init queue where Producer
  //will publish to
  tIcedQQueue queue;
  queue.queue = queue_buffer;
  queue.head = 0;
  queue.tail = 0;
  queue.capacity = 100;

  //Subscribe to topic
  icedq_subscribe("dummy_topic", &queue);

  while(true){

    uint8_t received = icedq_utils_queue_to_buffer(&queue, consumed_buffer, 12);

    if(received > 0){
      kprintf_debug("Consumed the items: \n\r");
      for(int i=0; i<received; i++){
        kprintf_debug( "%c", consumed_buffer[i] );
      }
      kprintf_debug("\n\r");
    }

  }//end while
}


void ARDUINO_KERNEL_MAIN() {
  system_init();

  while(!hal_io_serial_is_ready(&serial_usb));

  syscalls_init(); //<<--- CAN'T BE PLACED BEFORE while(!hal_io_serial_is_ready(&serial_usb));
                   //      OR IT FAILS. I don't know why.

  scheduler_thread_create( main_user_thread, "main_user_thread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}
