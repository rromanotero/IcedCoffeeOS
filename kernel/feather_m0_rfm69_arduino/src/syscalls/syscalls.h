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

#ifndef SYSCALLS_H_
#define SYSCALLS_H_

#define SYSCALLS_TOPIC    "system.syscalls"   //TODO : Once routing keys are enabled, change this for
                                              //       topic=system, routing_key=syscalls

/**
*   System call numbers
*/
enum{
    SyscallSleep                    = 0,
    SyscallPioCreatePin             = 1,
    SyscallPioWrite                 = 2,
    SyscallPioRead                  = 3
};

#define SYSCALLS_QUEUE_SIZE             1000
#define SYSCALLS_REQUEST_SIZE_IN_BYTES  9 //syscall_num(1) + input (4) + output (4)

#define SYSCALLS_RAW_REQUEST_NUM_OFFSET         0
#define SYSCALLS_RAW_REQUEST_INPUT_OFFSET       1
#define SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET      5

typedef struct{
  volatile uint32_t arg0;
  volatile uint32_t arg1;
  volatile uint32_t arg2;
  volatile uint32_t arg3;
}tSyscallInput;

typedef struct{
  volatile uint32_t ret_val;
  volatile bool output_ready;
}tSyscallOutput;


#endif /* SYSCALLS_H_ */
