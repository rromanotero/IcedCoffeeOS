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

void (*systick_callback)(void);

/**
*	SystemTimer Start
*
*	Starts the System Timer
*
*	@param tick_freq_in_ms the tick frequency in milliseconds
*	@param callback function to be called when a tick occurs
*/
static uint32_t ms_count = 0;  //milliseconds count
static uint32_t ms_goal = 0;   //milliseconds goal
void hal_cpu_systimer_start(uint32_t tick_freq_in_ms, void(*callback)(void)){
	systick_callback = callback;
  ms_goal = tick_freq_in_ms;  //Arduino's systimer is set in millisecond steps
                              //No conversion needed
  //Nothing to start. Arduino has it running already
}





/**
*	Fault Exception Register Callback
*
*	Registers a generic callback function for CPU Fault Exceptions
*
*	@param callback the function that gets called on fault_type exception
*/
void hal_cpu_fault_register_callback( tFaultOrigin faultOrigin, void(*callback)(void)  ){

}

/**
*	SVC Start
*
*	Starts SVC calls and registers a callback function. The callback
*	execution of an SVC instruction
*
*	@param callback the function that gets called on supervisor calls
*/
void hal_cpu_svc_start( void(*callback)(void) ){

}

/**
*	HAL CPU Speed in Hz
*
*	Returns the CPU clock speed in Hz.
*
*	@return CPU clock speed in Hz
*/
uint32_t hal_cpu_speed_in_hz(void){

}

/**
*	CPU Delays
*
*	Busy-waiting delay
*
*	@param delay_in_ms delay in milliseconds
*/
void hal_cpu_delay(uint32_t delay_in_ms){
  delay(delay_in_ms);
}

/**
*	CPU Sleep
*
*/
void hal_cpu_sleep(uint32_t delay_in_ms){

}


extern "C" {
//C++ code cannot override weak aliases defined in C

int sysTickHook(void){
  if( systick_callback ){ //We don't want Arduion to trigger this
                            //before it's defined
    if( ms_count++ >= ms_goal ){
        (*systick_callback)();
        ms_count = 0;
    }
  }
  return 0; //Arduino expects a 0 when things go well
}

}
