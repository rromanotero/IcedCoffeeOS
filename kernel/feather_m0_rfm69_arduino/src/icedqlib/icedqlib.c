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
void serial_putc(tSerialPort* serial_port, char* str){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallSerialPutc);
  iql_input.arg0 = (uint32_t)(serial_port);
  iql_input.arg1 = (uint32_t)(str);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();
}

uint32_t adc_create_channel(tAdcChannel* adc, tAdcId id, tIoType io_type){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallAdcCreateChannel);
  iql_input.arg0 = (uint32_t)(adc);
  iql_input.arg1 = (uint32_t)(id);
  iql_input.arg2 = (uint32_t)(io_type);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();

  return iql_output.ret_val;
}

uint32_t adc_read(tAdcChannel* adc){
  uint8_t iql_raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t iql_request_num;
  tSyscallInput iql_input;
  tSyscallOutput iql_output;

  //Setup request
  iql_request_num = (uint32_t)(SyscallAdcRead);
  iql_input.arg0 = (uint32_t)(adc);
  syscall_utils_raw_request_populate(iql_raw_request, iql_request_num, &iql_input, &iql_output);

  //mark output as not ready
  iql_output.output_ready = false;

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();

  return iql_output.ret_val;
}


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

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();


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

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();
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

  //make syscall
  icedq_publish("system.syscalls", iql_raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

  //wait for output to be ready
  while(!iql_output.output_ready)
    icedqlib_yield();

  return iql_output.ret_val;
}

void icedqlib_yield(void){
  context_switcher_trigger(); //yield()
}
