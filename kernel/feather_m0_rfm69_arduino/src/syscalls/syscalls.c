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
  //Init queue
  syscalls_queue.queue = syscalls_queue_buffer;
  syscalls_queue.head = 0;
  syscalls_queue.tail = 0;
  syscalls_queue.capacity = SYSCALLS_QUEUE_SIZE;

  //Begin syscall KThread
  scheduler_thread_create( syscalls_kthread, "syscalls_kthread", 1024, ProcQueueReadyRealTime );
}

/**
*   Syscalls KThread
*/
void syscalls_kthread(void){

    //Subscribe to topic
    //TODO : Once routing keys are enabled, change this for
    //       topic=system, routing_key=syscalls
    icedq_subscribe("system.syscalls", &syscalls_queue);

    uint8_t raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
    uint8_t request_num;
    tSyscallInput* input;
    tSyscallOutput* output;

    while(true){
      uint8_t items = icedq_utils_queue_to_buffer(&syscalls_queue, raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

      if(items > 0){

        if(items != SYSCALLS_REQUEST_SIZE_IN_BYTES){
            faults_kernel_panic("Syscalls: malformed request");
        }

        request_num = raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET];
        input = (tSyscallInput*)((raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+3]<< 8*3));
        output =(tSyscallOutput*)((raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+3]<< 8*3));

        attend_syscall(request_num, input, output);
      }
    }

}

void attend_syscall( uint32_t request_num, tSyscallInput* input, tSyscallOutput* output){
    kprintf_debug( " \n\r == Attending syscall num %d ===", request_num );
    kprintf_debug( " == param0=%d, param1=%d, param2=%d, param3=%d === \n\r\n\r", input->arg0, input->arg1, input->arg2, input->arg3 );

    //attend syscall
    /*switch(syscall_num){
        case SVCDummy:

           break;

        //Error
        default:
            break;
    }*/
}
