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
uint8_t raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];

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
*   (handles syscalls)
*/
void syscalls_kthread(void){

    //Subscribe to topic
    icedq_subscribe(SYSCALLS_TOPIC, &syscalls_queue);

    while(true){
      uint8_t bytes_read = icedq_utils_queue_to_buffer(&syscalls_queue, raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

      if(bytes_read > 0){
        //A request was placed

        if(bytes_read != SYSCALLS_REQUEST_SIZE_IN_BYTES){
            faults_kernel_panic("Syscalls: malformed request");
        }

        uint8_t request_num = syscall_utils_raw_request_parse_request_num(raw_request);
        tSyscallInput* input = syscall_utils_raw_request_parse_input(raw_request);
        tSyscallOutput* output = syscall_utils_raw_request_parse_output(raw_request);

        attend_syscall(request_num, input, output);
      }
    }//end while
}

/*
*   Utils so this code is not repeated over and over
*
*   Populates a raw request, given the params of a request.
*   (request number, a pointer to the req's input, a pointer to the req's output)
*/
void syscall_utils_raw_request_populate(uint8_t* raw_request, uint32_t request_num, tSyscallInput* input, tSyscallOutput* output ){

  raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET] = request_num;
  raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+0] = ((uint32_t)(input)>>8*0) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+1] = ((uint32_t)(input)>>8*1) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+2] = ((uint32_t)(input)>>8*2) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+3] = ((uint32_t)(input)>>8*3) & 0xff;

  raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+0] = ((uint32_t)(output)>>8*0) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+1] = ((uint32_t)(output)>>8*1) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+2] = ((uint32_t)(output)>>8*2) & 0xff;
  raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+3] = ((uint32_t)(output)>>8*3) & 0xff;

}

/*
*   Parses request number from a raw request
*/
inline uint32_t syscall_utils_raw_request_parse_request_num(uint8_t* raw_request){
  return raw_request[SYSCALLS_RAW_REQUEST_NUM_OFFSET];
}

/*
*   Parse input from a raw request
*/
inline tSyscallInput* syscall_utils_raw_request_parse_input(uint8_t* raw_request){
  return (tSyscallInput*)((raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_INPUT_OFFSET+3]<< 8*3));
}

/*
*   Parse output from a raw request
*/
inline tSyscallOutput* syscall_utils_raw_request_parse_output(uint8_t* raw_request){
  return (tSyscallOutput*)((raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+0]<< 8*0) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+1]<< 8*1) + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+2]<< 8*2)  + (raw_request[SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET+3]<< 8*3));
}

void attend_syscall( uint32_t request_num, tSyscallInput* input, tSyscallOutput* output){
    kprintf_debug( " == Attending syscall num %d ===", request_num );
    kprintf_debug( " == param0=%d, param1=%d, param2=%d, param3=%d === \n\r", input->arg0, input->arg1, input->arg2, input->arg3 );

    //attend syscall
    /*switch(syscall_num){
        case SVCDummy:

           break;

        //Error
        default:
            break;
    }*/
}
