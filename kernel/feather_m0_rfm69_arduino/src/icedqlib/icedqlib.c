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

uint32_t pio_create_pin(tPioPin* pio_pin, tPioPort pio_port, uint32_t pin_number, tPioDir dir){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallPioCreatePin);
  iql_input.arg0 = (uint32_t)(pio_pin);
  iql_input.arg1 = (uint32_t)(pio_port);
  iql_input.arg2 = (uint32_t)(pin_number);
  iql_input.arg3 = (uint32_t)(dir);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //Call and wait for output to be ready
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);
  while(!iql_output.output_ready);

  return iql_output.ret_val;
}

void pio_write(tPioPin* pio_pin, bool state){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallPioWrite);
  iql_input.arg0 = (uint32_t)(pio_pin);
  iql_input.arg1 = (uint32_t)(state);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //Call and wait for output to be ready
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);
  while(!iql_output.output_ready);
}

bool pio_read(tPioPin* pio_pin){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallPioRead);
  iql_input.arg0 = (uint32_t)(pio_pin);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //Call and wait for output to be ready
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);
  while(!iql_output.output_ready);

  return iql_output.ret_val;
}
