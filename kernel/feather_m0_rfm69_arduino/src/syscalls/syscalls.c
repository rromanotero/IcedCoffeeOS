/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   and adapted from MiniOS:
*   (https://github.com/rromanotero/minios).
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

tIcedQQueue syscalls_queue;
uint8_t syscalls_queue_buffer[SYSCALLS_QUEUE_SIZE];

/**
*   Syscalls Init
*/
void syscalls_init(void){
  //Init syscalls queue
  syscalls_queue.queue = syscalls_queue_buffer;
  syscalls_queue.head = 0;
  syscalls_queue.tail = 0;
  syscalls_queue.capacity = SYSCALLS_QUEUE_SIZE;

  //Subscribe to topic
  //TODO : Once routing keys are enabled, change this for
  //       topic=system, routing_key=syscalls
  icedq_subscribe("system.syscalls", &syscalls_queue);
}

/**
*   Syscalls Kernel Thread
*/
void syscalls_kthread(void){

    uint32_t arg0, arg1, arg2, arg3, syscall_num;

    while(true){
      attend_syscall(&syscalls_queue, syscall_num, arg0, arg1, arg2, arg3);
    }

}

void attend_syscall( tIcedQQueue* syscalls_queue, uint32_t syscall_num, uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3){
    kprintf_debug( " == \n\n Attending syscall num %d \n\n ===", syscall_num );
    kprintf_debug( " == \n\n param0=%d, param1=%d, param2=%d, param3=%d,  \n\n ===", arg0, arg1, arg2, arg3 );

    //attend syscall
    switch(syscall_num){
        case SVCDummy:

           break;

        //Error
        default:
            break;
    }
}
