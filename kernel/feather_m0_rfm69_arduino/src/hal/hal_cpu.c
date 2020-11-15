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

#define USER_MODE_EXEC_VALUE 0xFFFFFFFD

//SysTick-related definitions
void (*systick_callback)(void);

//SVC-related defintions
void (*svc_callback)(void);

//PendSV-related definitions
void (*pendsv_callback)(void);

//Faults-related definitions
void (*fault_app_callback)(void);
void (*fault_system_callback)(void);

/**
*	HAL CPU Init
*
*	Initializes the CPU. This function must be called before
*	HAL IO Init. That is: hal_cpu_init(); hal_io_init();
*/
void hal_cpu_init(void){

	//Required for the CONTEXT SWITCHING to function
	// - Low Pty interrupt must have the lowest priority possible
	// - System timer the Highest highest
	NVIC_SetPriority(PendSV_IRQn, 0xFF); // Lowest
	NVIC_SetPriority(SysTick_IRQn, 0x00); // Highest
}

/**
*	Low Priority Software Interrupt Trigger
*
*	Triggers a PendsSV Exception
*/
void hal_cpu_lowpty_softint_trigger(void){
	SCB->ICSR |= (1<<28);
}

/**
*	Low Priority Software Interrupt Start
*
*	Registers a callback function for the PendSV Exception
*
*	@param callback the function that gets called on PendSV exception
*/
void hal_cpu_lowpty_softint_start( void(*callback)(void) ){
	pendsv_callback = callback;


	//Required for the CONTEXT SWITCHING to function
	// - Low Pty interrupt must have the lowest priority
	//   possible. System timer the Highest highest
	NVIC_SetPriority(PendSV_IRQn, 0xFF); // Lowest
}

/**
*	SystemTimer Start
*
*	Starts the System Timer
*
*	@param tick_freq_in_ms the tick frequency in milliseconds
*	@param callback function to be called when a tick occurs
*/
uint32_t ms_count = 0;  //milliseconds count
uint32_t ms_goal = 0;   //milliseconds goal
void hal_cpu_systimer_start(uint32_t tick_freq_in_ms, void(*callback)(void)){

	systick_callback = callback;
  ms_goal = tick_freq_in_ms;  //Arduino's systimer is set in millisecond steps
                              //No conversion needed

	//Required for the CONTEXT SWITCHING to function
	// - Low Pty interrupt must have the lowest priority
	//   possible. System timer the Highest highest
	NVIC_SetPriority(SysTick_IRQn, 0x00); // Highest

  //Nothing to start. Arduino has it running already
}


/**
*	Fault Start
*
*	Registers a generic callback function for CPU Fault Exceptions
*
*	@param callback the function that gets called on fault_type exception
*/
void hal_cpu_fault_start( tFaultOrigin fault_origin, void(*callback)(void)  ){
  switch(fault_origin){
			case FaultApp:		fault_app_callback = callback;		break;
			case FaultSystem:	fault_system_callback = callback;	break;
			default:			/* Error */							break;
		}
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
