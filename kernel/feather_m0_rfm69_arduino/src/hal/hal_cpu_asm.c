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


#define USER_MODE_EXEC_VALUE, 0xFFFFFFFD

/**
*
*	Sleep
*
*	Executes the instruction that puts the CPU
*	to sleep
*/
__attribute__((naked)) void hal_cpu_sleep(void){
  asm volatile( "wfi" );
}

/**
*	Return from exception in user mode
*
*	Returns from an exception, specifying an EXEC_VALUE
*	of 0xFFFFFFFD i.e. specifyin user mode and PSP as active SP
*/
__attribute__((naked)) void hal_cpu_return_exception_user_mode(){
  //TO DO FIX IT
	//asm volatile("ldr pc, =USER_MODE_EXEC_VALUE");
}


/**
*	void hal_cpu_save_context(void)
*
*	CPU Save Context
*
*	Pushes onto the stack the registers
*	R4-R11 that ought to be saved "in software"
*	during context switch
*
*/
__attribute__((naked)) void hal_cpu_save_context(void){
  //TO DO FIX IT
	//asm volatile("mrs r0, psp\nstmfd r0!, {r4-r11}\nmsr psp, r0\nbx lr");
}

/**
*	void hal_cpu_restore_context(void)
*
*	CPU Restore Context
*
*	Pops from the stack the registers
*	R4-R11 that ought to be restored "in software"
*	during context switch
*
*/
__attribute__((naked)) void hal_cpu_restore_context(void){
  //TO DO FIX IT
	//asm volatile("mrs r0, psp\nldmfd r0!, {r4-r11}\nmsr psp, r0\nbx lr");
}

/**
*	uint32_t hal_cpu_get_psp(void)
*
*	Gets the PSP
*
*	Returns the process stack pointer
*/
__attribute__((naked)) uint32_t hal_cpu_get_psp(void){
  //TO DO FIX IT
	//asm volatile("mrs	r0, psp\nbx lr");
}

/**
*	void hal_cpu_set_unprivileged(void)
*
*	CPU Set unprivileged
*
*	Set the CPU as unprivileged (when in thread mode)s
*/
__attribute__((naked)) void hal_cpu_set_unprivileged(void){
  //TO DO FIX IT
	//asm volatile("mrs r3, control\norr	r3, r3, #1\nmsr control, r3\nisb\nbx lr");
}


/**
*	void hal_cpu_set_psp_active(void)
*
*	CPU Set PSP active
*
*	Sets the Process Stack Pointer as active (when in thread mode)
*/
__attribute__((naked)) void hal_cpu_set_psp_active(void){
  //TO DO FIX IT
	//asm volatile("mrs r3, control\norr	r3, r3, #2\nmsr control, r3\nisb\nbx lr");
}

/**
*	void hal_cpu_set_psp(uint32_t)
*
*	Sets the PSP
*
*	Sets the Process Stack Pointer value
*/
__attribute__((naked)) void hal_cpu_set_psp(uint32_t){
  //TO DO FIX IT
	//asm volatile("msr psp, r0\nbx lr");
}



/////////////////////////////////////////////////////////////////////
//              Handler/Hooks overriding Area                     ///
/////////////////////////////////////////////////////////////////////


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

__attribute__((naked)) void PendSV_Handler(void){
  	(*pendsv_callback)();
}

__attribute__((naked)) void HardFault_Handler(){
	asm volatile("b faults_goto_right_callback");
}

__attribute__((naked)) void NMI_Handler(){
	asm volatile("b faults_goto_right_callback");
	//EIC->NMIFLAG.bit.NMI = 1; // Clear interrupt
}

__attribute__((naked)) void faults_goto_right_callback(){
	// Bit 2 in EXC_RETURN (placed in the LR on exception entry) tells
	// which stack to use on exception return
	register uint32_t exc_return;
  __asm volatile ("mov %0, lr\n" : "=r" (exc_return) );

	if ( exc_return & 0b0100 )
		(*fault_app_callback)();
	else
	 	(*fault_system_callback)();
}

void svcHook(void){

}



} //end extern C