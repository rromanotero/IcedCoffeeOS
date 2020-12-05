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
#include "main.h"

extern tPioPin led_pin;         //Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

tPioPin led_left_r;
tPioPin led_left_g;
tPioPin led_left_b;

void sample_kthread(void){

  hal_io_pio_create_pin(&led_left_r, PioA, 18, PioOutput);
  hal_io_pio_create_pin(&led_left_g, PioA, 16, PioOutput);
  hal_io_pio_create_pin(&led_left_b, PioA, 19, PioOutput);

  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    hal_io_pio_write(&led_left_r, true);
    hal_io_pio_write(&led_left_g, false);
    hal_io_pio_write(&led_left_b, false);
    hal_cpu_delay(1000);
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    hal_io_pio_write(&led_left_r, true);
    hal_io_pio_write(&led_left_g, true);
    hal_io_pio_write(&led_left_b, true);
    hal_cpu_delay(1000);
  }

  while(!hal_io_serial_is_ready(&serial_usb)); /// <<--- WAIT FOR USER

  uint8_t raw_request[SYSCALLS_REQUEST_SIZE_IN_BYTES];
  uint8_t request_num;
  tSyscallInput input;
  tSyscallOutput output;

  while(true){
    request_num = 17;
    input.arg0 = 1;
    input.arg1 = 2;
    input.arg2 = 3;
    input.arg3 = 4;

    syscall_utils_raw_request_populate(raw_request, request_num, &input, &output);
    icedq_publish("system.syscalls", raw_request, SYSCALLS_REQUEST_SIZE_IN_BYTES);

    for(volatile int i=0; i<4800;i++);
  }
}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  scheduler_thread_create( sample_kthread, "sample_kthread", 1024, ProcQueueReadyRealTime );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
}



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

/**
*	System Init
*
*	Initializes everything. Must be called before any other call
*/
void system_init(void){
	hal_cpu_init();
	hal_io_init();
	hal_radio_init();
	faults_init();
	scheduler_init();
	icedq_init();
	syscalls_init(); 
}



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

/*  There's now way I was getting that PendSV Handler code right
 *  by myself. Thanks to Adam Heinrich. So here's the link and License.
 *    - Rafael
 *
 * https://github.com/adamheinrich/os.h/blob/master/src/os.c
 *
 * This file is part of os.h.
 *
 * Copyright (C) 2016 Adam Heinrich <adam@adamh.cz>
 *
 * Os.h is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Os.h is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with os.h.  If not, see <http://www.gnu.org/licenses/>.
 */

tMiniProcess* active_proc;		//The active process

/*
*   Context Switcher Init
*/
static uint32_t tick_frequency;

void context_switcher_init(uint32_t tick_freq){
  tick_frequency = tick_freq;

  //Active process is the null process
  //A null process (used to mark the lack of an active process)
  static tMiniProcess null_proc;
  null_proc.name = "null",
  null_proc.state = ProcessStateNull;

	active_proc = &null_proc;


  //Inits Low Pty Int
	hal_cpu_lowpty_softint_start( low_pty_callback );
}


/*
* Triggers a context switch
*/
void context_switcher_trigger(){
    //trigger low pty software interrupt
    hal_cpu_lowpty_softint_trigger();
}

/*
*   Begin context switching
*
*  Here's where the System Timer begins ticking
*/
void context_switcher_begin_ticking(){
    hal_cpu_systimer_start( tick_frequency, tick_callback );
}

/*
*	The tick callback
*
*/
void tick_callback(void){

	//Context switch happens in the low pty interrupt
	//
	//This is to prevent the issue of a tick interrupting
	//an interrupt handler and then giving it back
	//to a thread (instead of the handler that had it).
	hal_cpu_lowpty_softint_trigger();
}


/*
*	Low Pty interrupt callback
*
*  Context switch takes place here.
*/
__attribute__((naked)) void low_pty_callback(void){
	// This is adapted from both MiniOS's and Heinrich's os.h
	// Cortex M0 context switcher.
	//
	// https://github.com/adamheinrich/os.h/blob/master/src/os.c

	__asm volatile(
    "cpsid	i        \n" \
      :::
    );

	//save software context
  __asm volatile(
    "mrs	r0, psp      \n"  \
    "sub	r0, #16      \n"  \
    "stmia	r0!,{r4-r7}\n"  \
    "mov	r4, r8       \n"  \
    "mov	r5, r9       \n"  \
    "mov	r6, r10      \n"  \
    "mov	r7, r11      \n"  \
    "sub	r0, #32      \n"  \
    "stmia	r0!,{r4-r7}\n"  \
    "sub	r0, #16      \n" \
		"msr psp, r0"			//<<--- Needed so SP points to the top of the stack
      :::							// New SP (FIgure 12.2 MiniOS Book)
    );

  //Not the null process?
	//(this'll skiip hal_cpu_get_psp() on the very first tick)
	if( active_proc->state != ProcessStateNull ){
		//save SP
		active_proc->sp = (uint32_t*)hal_cpu_get_psp();
	}

	//get next active process
	active_proc = scheduler_proc_next(active_proc);

	//restore SP
	hal_cpu_set_psp( (uint32_t)active_proc->sp );

	//restore software context
  __asm volatile(
      "ldmia	r0!,{r4-r7} \n"  \
      "mov	r8, r4        \n"  \
      "mov	r9, r5        \n"  \
      "mov	r10, r6       \n"  \
      "mov	r11, r7       \n"  \
      "ldmia	r0!,{r4-r7} \n"  \
      "msr	psp, r0       "   //<<--- Needed so SP points to the top of the stack
      :::											// New SP (FIgure 12.2 MiniOS Book)
    );

		//give CPU to active process
    __asm volatile(
      "ldr r0, =0xFFFFFFFD\n" \
      "cpsie i       			\n" \
      "bx r0"
        :::
      );	//0xFFFFFFFD is USER_MODE_EXEC_VALUE. It tells
					//the CPU to return the exception in User (thread) mode
					//and using the PSP.
}



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

/**
*	Fault Init
*
*	Enables CPU Faults exceptions, and sets their respective
*	callbacks
*
*	@param fault_type the fault type
*	@param callback the function that gets called on fault_type exception
*/
void faults_init(void){
	hal_cpu_fault_start( FaultApp, faults_app_entry_point );
	hal_cpu_fault_start( FaultSystem, faults_system_entry_point );
}

/*
*	App Faults entry point
*
*   Execution comes here when a fault originated in an app.
*
*/
void faults_app_entry_point(void){
  //Terminate offending app
  while(1); //we don't have apps yet
}

/*
*	Kernel Faults entry point
*
*   Execution comes here when a fault originated in the OS itself.
*
*/
void faults_system_entry_point(void){
  //We did something wrong. Panic.
  faults_kernel_panic( "Caught HardFault or NMI Fault. Cannot recover." );
}

/**
*	Kernel Panic Function
*
*	@param manic_msg the panic message
*/
extern tPioPin led_pin;				//Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

void faults_kernel_panic( char* panic_msg ){

	hal_io_serial_puts(&serial_usb, "\n\rSomething went wrong =(\n\r");
	hal_io_serial_puts(&serial_usb, panic_msg);

	//Show panic LED sequence.
	while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));

		//Can't use HAL Delay function, since
		//it uses the System Timer to count.
		//Hard Fault has a hgher priority, so
		//System Timer won't tick
		//while we're here.
    for(volatile int i=0; i<SYS_PANIC_LED_BLINKING_WAIT; i++);
  }
}



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




/**
*	uint32_t hal_cpu_get_psp(void)
*
*	Gets the PSP
*
*	Returns the process stack pointer
*/
__attribute__((naked)) uint32_t hal_cpu_get_psp(void){
  __asm volatile(
    "mrs	r0, psp        \n" \
    "bx lr"
      :::
    );
}


/**
*	void hal_cpu_set_psp_active(void)
*
*	CPU Set PSP active
*
*	Sets the Process Stack Pointer as active (when in thread mode)
*/
__attribute__((naked)) void hal_cpu_set_psp_active(void){
  __asm volatile(
    "mrs r3, control    \n" \
    "mov r2, #2         \n" \
    "orr	r3, r3, r2    \n" \
    "msr control, r3    \n" \
    "isb                \n" \
    "bx lr"
      :::
    );
}

/**
*	void hal_cpu_set_psp(uint32_t)
*
*	Sets the PSP
*
*	Sets the Process Stack Pointer value
*/
__attribute__((naked)) void hal_cpu_set_psp(uint32_t){
  __asm volatile(
    "msr psp, r0  \n" \
    "bx lr"
      :::
    );
}


/////////////////////////////////////////////////////////////////////
//              Handler/Hooks overriding Area                     ///
/////////////////////////////////////////////////////////////////////


extern uint32_t ms_goal;

extern "C" {
//C++ code cannot override weak aliases defined in C

int sysTickHook(void){
  if( systick_callback ){ //We don't want Arduion to trigger this
                            //before it's defined
    if( ms_count++ >= ms_goal ){
        ms_count = 0;
        (*systick_callback)();
    }
  }
  return 0; //Arduino expects a 0 when things go well
}

__attribute__((naked)) void PendSV_Handler(void){
	//This works because GCC will use a scratch register
	//to hold 'address'. Scratch rgisters are stacked already.
	__asm volatile(
		"blx %[address]\n" \
			:
			: [address] "r" (pendsv_callback)
			:
		);
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

} //end extern C



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

/**
*	HAL IO Init
*
*	Initializes the board and IO pins. This function must be called before
*	any other call to an IO device. Example: hal_io_init(); hal_mtimer_Start();...
*
*/
tPioPin led_pin;
tSerialPort serial_usb;

void hal_io_init(void){
  //LED Pin begins off
  hal_io_pio_create_pin(&led_pin, PioA, 8, PioOutput);
  hal_io_pio_write(&led_pin, false);

  //Serial USB begins on
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);
}

/**
*	PIO Create
*
*  Creates a PIO pin
*
*	 Given a tPioPin* p, it populates p->internal_pin so when
*  pio_write or pio_read  are called, they know which pin to pass
*  to the drivers.
*/
uint32_t hal_io_pio_create_pin(tPioPin* pio_pin, tPioPort pio_port, uint32_t pin_number, tPioDir dir){

  uint32_t status = HAL_SUCCESS;

	pio_pin->pin_number = pin_number;
	pio_pin->pio_port = pio_port;

	switch (pio_port)
	{
		case PioA:
      if(pin_number == 2) pio_pin->internal_pin = 14;
      else if(pin_number == 4) pio_pin->internal_pin = 17;
      else if(pin_number == 5) pio_pin->internal_pin = 18;
      else if(pin_number == 8) pio_pin->internal_pin = 13;
      else if(pin_number == 10) pio_pin->internal_pin = 1;
      else if(pin_number == 11) pio_pin->internal_pin = 0;
      else if(pin_number == 12) pio_pin->internal_pin = 22;
      else if(pin_number == 15) pio_pin->internal_pin = 5;
      else if(pin_number == 16) pio_pin->internal_pin = 11;
      else if(pin_number == 17) pio_pin->internal_pin = 13;
      else if(pin_number == 18) pio_pin->internal_pin = 10;
      else if(pin_number == 19) pio_pin->internal_pin = 12;
      else if(pin_number == 20) pio_pin->internal_pin = 6;
      else if(pin_number == 22) pio_pin->internal_pin = 20;
      else if(pin_number == 23) pio_pin->internal_pin = 21;
      else status = HAL_IO_PIO_PIN_NOT_FOUND;

			break;
		case PioB:
      if(pin_number == 2) pio_pin->internal_pin = 19;
      else if(pin_number == 8) pio_pin->internal_pin = 15;
      else if(pin_number == 9) pio_pin->internal_pin = 16;
      else status = HAL_IO_PIO_PIN_NOT_FOUND;

      break;
		default :
			status = HAL_IO_PIO_PORT_NOT_FOUND;
	}

  //set pin direction
  switch(dir){
		case PioOutput:	  pinMode(pio_pin->internal_pin, OUTPUT); break;
		case PioInput:	  pinMode(pio_pin->internal_pin, INPUT);  break;
	}

  return status;
}


/**
*	PIO Write
*
*	Writes a state/level to a PIO pin
*/
void hal_io_pio_write(tPioPin* pio_pin, bool state){
  digitalWrite( pio_pin->internal_pin, state? HIGH : LOW );
}

/**
*	PIO Read
*
*	Reads a PIO pin state/level
*/
bool hal_io_pio_read(tPioPin* pio_pin){
	return digitalRead(pio_pin->internal_pin);
}

/**
*	ADC Create
*
*	Stops ADC conversion on given ADC channel
*/
uint32_t hal_io_adc_create_channel(tAdcChannel* adc, tAdcId id, tIoType io_type){

    if( io_type == IoPoll ){
      switch( id ){
        case AdcA:
          adc->id = id;
          adc->io_type = IoPoll;
          adc->internal_adc = A0;
          break;
    		case AdcB:
          adc->id = id;
          adc->io_type = IoPoll;
          adc->internal_adc = A7; //VBattery
          break;
    		default:
          return HAL_IO_ADC_CHANNEL_NOT_FOUND;
    	}
    }
    else{
      return HAL_IO_TYPE_NOT_FOUND;
    }

    return HAL_SUCCESS;
}


/**
*	ADC Read
*
*/
uint32_t hal_io_adc_read(tAdcChannel* adc){
  return analogRead(adc->internal_adc);
}


/**
*  Creates a Serial Port of type 'io_type' (poll or interrupt)
*  that maps to the physical serial port with ID 'id'.
*
*   The serial port is init at 'baudrate' bps
*/
uint32_t hal_io_serial_create_port( tSerialPort* serial_port, tSerialId id, tIoType io_type, uint32_t baudrate ){

  if( io_type == IoPoll ){

    switch( id ){
  		case SerialA:
        Serial.begin(baudrate);

        serial_port->id = SerialA;
        serial_port->baudrate = baudrate;
        serial_port->io_type = IoPoll;
        serial_port->internal_driver_a = &Serial;  //Serial
        break;

      case SerialB:
          Serial1.begin(baudrate);

          serial_port->id = SerialB;
          serial_port->baudrate = baudrate;
          serial_port->io_type = IoPoll;
          serial_port->internal_driver_b = &Serial1;  //Serial 1
          break;

  		default:
        return HAL_IO_SERIAL_PORT_NOT_FOUND;
  	}

  }
  else{
    return HAL_IO_TYPE_NOT_FOUND;
  }

  return HAL_SUCCESS;
}


/**
*	Serial is ready
*
*/
bool hal_io_serial_is_ready( tSerialPort* serial_port ){
  switch( serial_port->id ){
    case SerialA: return *(serial_port->internal_driver_a);
    case SerialB: return *(serial_port->internal_driver_b);
    default:
      //Can't happen since it was us who init  serial_port->id
      break;
    }
}

/**
*	Serial putc
*
*	Writes a string to the specified Serial port
*
*/
void hal_io_serial_puts( tSerialPort* serial_port, char* str ){
  while(*str)
		hal_io_serial_putc( serial_port, *str++);
}

/**
*	Serial putc
*
*	Writes a character to the specified Serial port
*
*/
void hal_io_serial_putc( tSerialPort* serial_port, uint8_t c ){
  switch( serial_port->id ){
    case SerialA: serial_port->internal_driver_a->write(c); break;
    case SerialB: serial_port->internal_driver_b->write(c); break;
    default:
      //Can't happen since it was us who init  serial_port->id
      break;
  }
}

/**
*	Serial getc
*
*	Reads a character from the specified Serial port
*/
uint8_t hal_io_serial_getc( tSerialPort* serial_port ){
  switch( serial_port->id ){
    case SerialA:
      while(!serial_port->internal_driver_a->available()); //wait for a character
      return serial_port->internal_driver_a->read();
    case SerialB:
      while(!serial_port->internal_driver_b->available()); //wait for a character
      return serial_port->internal_driver_b->read();
    default:
      //Can't happen since it was us who init  serial_port->id
      break;
  }
}


uint32_t hal_io_pwm_create_channel(tPwmChannel* pwm, tPwmId id){
  //
  //The following pins can be configured for PWM without any signal conflicts
  //as long as the SPI, I2C, and UART pins keep their protocol functions:
  //
  // Digital pins 5, 6, 9, 10, 11, 12, and 13
  // Analog pins A3 and A4
  //
  //From https://learn.adafruit.com/adafruit-feather-m0-basic-proto/adapting-sketches-to-m0

    switch( id ){
      case PwmA:
        pinMode(A3, OUTPUT);   //Output mode indicates PWM (as opposed to DAC)
        analogWrite(A3, 0);   //Begin off
        pwm->id = id;
        pwm->internal_pin = A3;
        break;
    	case PwmB:
        pinMode(A4, OUTPUT);   //Output mode indicates PWM (as opposed to DAC)
        analogWrite(A4, 0);    //Begin off
        pwm->id = id;
        pwm->internal_pin = A4;
        break;
    	default:
        return HAL_IO_PWM_CHANNEL_NOT_FOUND;
    }

    return HAL_SUCCESS;
}


void hal_io_pwm_write(tPwmChannel* pwm, uint32_t duty_cycle){
  uint32_t max_val = 256;
  analogWrite(pwm->internal_pin, (duty_cycle*max_val)/100);
}



uint32_t hal_io_servo_create_channel(tServoChannel* servo, tServoId id){

    static Servo myservo_a;
    static Servo myservo_b;

    switch( id ){
      case ServoA:
        myservo_a.attach(12);

        servo->id = id;
        servo->internal_driver = &myservo_a;
        break;
    	case ServoB:
        myservo_b.attach(13);

        servo->id = id;
        servo->internal_driver = &myservo_b;
        break;
    	default:       return HAL_IO_PWM_CHANNEL_NOT_FOUND;
    }



    return HAL_SUCCESS;
}


void hal_io_servo_write(tServoChannel* servo, uint32_t orientation_in_degrees){
  servo->internal_driver->write(orientation_in_degrees);
}



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

/**
 * @file	hal_memreg.c
 * @author
 * @version
 *
 * @brief Memory Regions part of the Hardware Abstraction Layer.
 *
 */

//System Memory Region
#define MEM_REGION_SYS_BASEPTR		(uint8_t*)0x20000000
#define MEM_REGION_SYS_SIZE			SYS_SYSTEM_MEM_AVAILABLE

//App Memory Region
//( Starts right after System + 4 bytes (for SP ))
#define MEM_REGION_APP_BASEPTR		(uint8_t*)( MEM_REGION_SYS_BASEPTR + MEM_REGION_SYS_SIZE + 4 )
#define MEM_REGION_APP_SIZE			SYS_APP_MAX_SIZE-4

//Stack Memory Region
#define MEM_REGION_STACK_BASEPTR	( MEM_REGION_APP_BASEPTR + MEM_REGION_APP_SIZE )
#define MEM_REGION_STACK_SIZE		SYS_USER_STACK_MAX_SIZE

//Linker script variables for System Stack
extern uint32_t __StackTop;				//end of stack

/**
*	HAL Memory Regions Init
*
*	Initializes Memory. This function must be called after
*	HAL CPU Init. That is: hal_cpu_init(); hal_mem_init();...
*
*/
void hal_memreg_init(void){
	//nothing to initialize...
	//for consistency...
	//and compatibility if future versions require initialization
}

/**
*	HAL Memory Region Read
*
*	Reads information related to the specified memory region.
*
*	@param memid	the specified memory region
*	@param memreg	a pointer to the tMemRegion variable to be populated
*/
void hal_memreg_read( tMemRegionId memid, tMemRegion* memreg ){
	if( memreg == 0 ) return; //Error (null ptr)

	memreg->id = memid;

	switch( memid ){
		case MemRegSystem:
			memreg->base = (uint8_t*)MEM_REGION_SYS_BASEPTR;
			memreg->size  = MEM_REGION_SYS_SIZE;
			break;
		case MemRegApp:
			memreg->base = (uint8_t*)MEM_REGION_APP_BASEPTR;
			memreg->size  = MEM_REGION_APP_SIZE;
			break;
		case MemRegSystemStack:
			memreg->base = (uint8_t*)&__StackTop;				//base = stack's end address
                      //THIS IS WHERE ARDUINO SETS THE STACK ACTUALLY....
                      //LATER THIS NEEDS TO BE CHANGED TO SOMEWHER ELSE.
			memreg->size =  4096; //FIXED FOR NOWW, since we have no way to set &__stack_size__;
													  //in the linker script... JUST MAKE SURE IT'S 8-BYTE ALIGNED
			break;
		case MemRegUserStack:
			memreg->base = (uint8_t*)MEM_REGION_STACK_BASEPTR;				//base = stack's end address
			memreg->size = MEM_REGION_STACK_SIZE;
			break;
		default:
			//Error
			break;
	}

}



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

/**
*	HAL Radio Init
*
*/
void hal_radio_init(void){
	//For compatibility
}

uint32_t hal_radio_write(tRadioTransceiver* transceiver, tRadioMessage* message ){
  if (!transceiver->internal_manager->sendtoWait(message->payload, sizeof(message->payload), message->address))
        return HAL_RADIO_SEND_FAILED;

  return HAL_SUCCESS;
}

uint32_t hal_radio_read(tRadioTransceiver* transceiver, tRadioMessage* message ){
  uint8_t len = sizeof(message->payload);
  uint8_t from;

  if( transceiver->internal_manager->recvfromAck( message->payload, &len, &from ) ){
    message->len = strlen( (const char*)message->payload );
    message->address = from;
    message->rssi = abs( transceiver->internal_driver->lastRssi() );
    return HAL_SUCCESS;
  }else{
    return HAL_RADIO_READ_FAILED;
  }
}

uint32_t hal_radio_available(tRadioTransceiver* transceiver){
  return transceiver->internal_manager->available();
}

uint32_t hal_radio_create_transceiver(tRadioTransceiver* transceiver, tRadioId id, uint32_t address, uint32_t tx_power ){
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4

  // Make sure these two are STATIC!
  static RH_RF69 rf69(RFM69_CS, RFM69_INT);
  static RHReliableDatagram rf69_manager(rf69, address);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init())
    return HAL_RADIO_INIT_FAILED;

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(915.0))
    return HAL_RADIO_INIT_FAILED;

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(HAL_RADIO_TX_POWER_MIN, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  transceiver->id = id;
  transceiver->io_type = IoPoll;
  transceiver->internal_driver = &rf69;
  transceiver->internal_manager = &rf69_manager;

  return HAL_SUCCESS;
}



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

#define ICEDQ_SUSCRIPTION_POOL_SIZE  ICEDQ_MAX_NUM_SUSCRIPTIONS

typedef struct{
	tIcedQSuscription list[ICEDQ_SUSCRIPTION_POOL_SIZE];
	uint32_t count;
}tSuscriptionPool;

tSuscriptionPool suscription_pool;
tIcedQSuscription* active_subscriptions[ICEDQ_SUSCRIPTION_POOL_SIZE];
uint32_t num_of_active_suscriptions;


/*
*   IcedQ Init
**/
void icedq_init(){
	//init suscription pool
	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++)
	  suscription_pool.list[i].free = true;

	//Init active subscriptions
	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++)
		active_subscriptions[i] = NULL;
}


/*
*   IcedQ Publish to topic
*
**/
void icedq_publish(const char* topic, uint8_t* raw_message_bytes, uint32_t message_len_in_bytes){
	//TODO:
	//	- Replace linear looking for topic for a table lookup
	//	- Add Routing Key
	//
  for(int i=0; i<num_of_active_suscriptions; i++){

			//Look for suscriptors
      if( active_subscriptions[i] != NULL && strcmp(topic, active_subscriptions[i]->topic) == 0 ){

					//Found one. Pubished to its queue.
					tIcedQQueue* q = active_subscriptions[i]->registered_queue;

					spin_lock_acquire();	//<<-- Some strange bugs if consuming from queue
					 										//			and publishing to it are not mut exclusive

					volatile uint32_t tail = q->tail;
					volatile uint32_t head = q->head;

					uint32_t spaced_used;
			    if(head > tail){
			      //tail went around
			      spaced_used = ((q->capacity) - head) + tail;
			    }
			    else{
			      spaced_used = (tail - head);
			    }

					if( spaced_used + message_len_in_bytes <= q->capacity ){

						//if there's space,
						//copy over raw bytes
						for(int j=0; j<message_len_in_bytes; j++){
								q->queue[q->tail] = raw_message_bytes[j];
								q->tail = (q->tail + 1) % q->capacity;
						}

					}else{
						//Queue full. Silently skip it.
					}
					spin_lock_release();

      }//end if suscriptor matching
  }//end for

}

/*
*   IcedQ Subscribe
*
**/
uint32_t icedq_subscribe(const char* topic, tIcedQQueue* queue){

    tIcedQSuscription* suscription = suscriptions_pool_get_one();

    if(suscription ==  NULL)
        return ICEDQ_NO_MORE_SUSCRIPTIONS_AVAILABLE;

    suscription->topic = topic;
    suscription->registered_queue = queue;

    active_subscriptions[num_of_active_suscriptions++] = suscription;

    return ICEDQ_SUCCESS;
}

/*
* 	Utils copy from Queue to buffer
*		(just so this code is not repeatded everywhere)
*
*		Populates buffer with as many element are available in queue or
*   'max_items', whichever happens first.
*
*		Returns number of elements read.
*/
uint32_t icedq_utils_queue_to_buffer(tIcedQQueue* queue, uint8_t* buffer, uint32_t max_items){

		spin_lock_acquire(); //<<-- Some strange bugs if consuming from queue
		 										//			and publishing to it are not mut exclusive

		volatile uint32_t head = queue->head;
		volatile uint32_t tail = queue->tail;

		uint32_t bytes_to_read;
		if(head > tail){
			//tail went around
			bytes_to_read = (queue->capacity - head) + tail;
		}
		else{
			bytes_to_read = (tail - head);
		}

		//adjust for max_items
		bytes_to_read = min(bytes_to_read, max_items);

		if(bytes_to_read > 0){

			for(int i=0; i< bytes_to_read; i++){
					//copy messages from queue to items
					buffer[i] = queue->queue[queue->head];
					queue->head = (queue->head + 1) % queue->capacity;
			}
		}

		spin_lock_release();

		return bytes_to_read;
}

tIcedQSuscription* suscriptions_pool_get_one(void){
  	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++){
  		if( suscription_pool.list[i].free ){
          suscription_pool.list[i].free = false;  //No longer free
    			return &(suscription_pool.list[i]);
      }
  	}
  	return NULL; //We're out of threads
}




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

/* This is from here:
*   https://github.com/mrvn/RaspberryPi-baremetal/tree/master/005-the-fine-printf 
*
* Copyright (C) 2007-2015 Goswin von Brederlow <goswin-v-b@web.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

void local_putc( uint8_t c ){
    //No video yet
    //hal_video_putc( c, SYSTEM_SCREEN_TEXT_SIZE, SYSTEM_SCREEN_TEXT_COLOR );
}

void debug_putc( uint8_t c ){
    hal_io_serial_putc( &serial_usb, c );
}

_Bool isdigit(unsigned char c) {
    return ((unsigned char)(c - '0') < 10);
}

void kprintf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vcprintf((vcprintf_callback_t)local_putc, NULL, format, args);
    va_end(args);
}

void kprintf_debug(const char *format, ...) {

    if( SYSTEM_DEBUG_ON ){
        spin_lock_acquire();

        va_list args;
        va_start(args, format);
        vcprintf((vcprintf_callback_t)debug_putc, NULL, format, args);
        va_end(args);

        spin_lock_release();
    }
}


int snprintf(char *buf, size_t size, const char *format, ...) {
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buf, size, format, args);
    va_end(args);

    return len;
}


/* atoi - convert string to int
 * @ptr: pointer to string
 *
 * Returns converted int and leaves ptr pointing to the first character after
 * the number.
 */
int atoi(const char** ptr) {
    const char* s = *ptr;
    int i = 0;
    while(isdigit(*s)) {
	i = i * 10 + (*s++ - '0');
    }
    *ptr = s;
    return i;
}

/* cprint_int - Convert integer to string
 * @callback:	callback function to add char
 * @num:	number to convert
 * @base:	must be 10 or 16
 * @size:	number of bytes to fill
 * @precision:	number of digits for floats
 * @flags:	output flags
 *
 * Returns nothing.
 */
void cprint_int(vcprintf_callback_t callback, void *state, uint64_t num,
		int base, int width, int precision, Flags flags) {
    const char LOWER[] = "0123456789abcdef";
    const char UPPER[] = "0123456789ABCDEF";
    const char *digits = (flags.upper) ? UPPER : LOWER;
    char tmp[20];

    // Sanity check base
    if (base != 8 && base != 10 && base != 16) return;

    // Check for sign
    _Bool negative = false;
    if (flags.sign) {
	int64_t t = num;
	if (t < 0) {
	    num = -t;
	    negative = true;
	}
    }

    // convert number in reverse order
    int len = 0;
    if (num == 0) { // special case
	tmp[len++] = '0';
    }
    while(num > 0) {
	tmp[len++] = digits[num % base];
	num /= base;
    }
    // Correct presision if number too large
    if (precision < len) precision = len;

    // Account for sign and alternate form
    if (negative || flags.plus) {
	--width;
    }
    if (flags.alternate) {
	width -= 2;
    }

    // Put sign if any
    if (negative) {
	callback('-', state);
    } else if (flags.plus) {
	callback(flags.space ? ' ' : '+', state);
    }

    // Put 0x prefix
    if (flags.alternate) {
	callback('0', state);
	callback('x', state);
    }

    // Pad with ' ' if not left aligned
    if (!flags.left) {
	while(precision < width--) callback(flags.zeropad ? '0' : ' ', state);
    }

    // Pad with ' ' or '0' to precision
    while(len < precision--) {
	callback(flags.zeropad ? '0' : ' ', state);
	--width;
    }

    // Put number
    while(len > 0) {
	callback(tmp[--len], state);
	--width;
    }

    // fill remaining space (flags.left was set)
    while(width-- > 0) callback(' ', state);
}

static void buffer_add(char c, BufferState *state) {
    if (state->size > 0) {
	*state->pos = c;
	--state->size;
    }
    ++state->pos;
}

/* vcprintf - Format a string and call callback for each char
 * @callback:	callback function to add char
 * @format:	Format string for output
 * @args:	Arguments for format string
 *
 * Returns nothing.
 */
void vcprintf(vcprintf_callback_t callback, void *state, const char* format,
	     va_list args) {
    while(*format != 0) {
	// Copy normal chars 1:1
	if (*format++ != '%') {
	    callback(format[-1], state); // format has already advanced
	    continue;
	}

	// Placeholder: %[flags][width][.precision][length]type
	/* Flags:
	 * '+': Always include a '+' or '-' sign for numeric types
	 * '-': Left align output
	 * '#': Alternate form, '0x' prefix for p and x
	 * ' ': Include ' ' for postive numbers
	 * '0': Pad with '0'
	 */
	Flags flags = {false, false, false, false, false, false, false};
    repeat:
	switch(*format++) {
	case '+': flags.plus = true; goto repeat;
	case '-': flags.left = true;  goto repeat;
	case '#': flags.alternate = true; goto repeat;
	case ' ': flags.space = true; goto repeat;
	case '0': flags.zeropad = true; goto repeat;
	default: --format; // undo ++
	}
	/* Width:
	 * '[0-9]'+: use at least this many characters
	 * '*'     : use int from 'args' as width
	 */
	int width = 0;
	if (*format == '*') {
	    ++format;
	    width = va_arg(args, int);
	    if (width < 0) width = 0;
	} else if (isdigit(*format)) {
	    width = atoi(&format);
	}
	/* Precision:
	 * '[0-9]'+: use max this many characters for a string
	 * '*'     : use int from 'args' as precision
	 */
	int precision = -1;
	if (*format == '.') {
	    ++format;
	    if (*format == '*') {
		++format;
		precision = va_arg(args, int);
		if (precision < 0) precision = 0;
	    } else {
		precision = atoi(&format);
	    }
	}
	/* Length:
	 * 'hh': [u]int8_t
	 * 'h' : [u]int16_t
	 * 'l' : [u]int32_t
	 * 'll': [u]int64_t
	 * 'z' : [s]size_t
	 * 't' : ptrdiff_t
	 */
	int length = 4;
	switch(*format++) {
	case 'h':
	    if (*format == 'h') {
		++format; length = 1;
	    } else {
		length = sizeof(short);
	    }
	    break;
	case 'l':
	    if (*format == 'l') {
		++format; length = sizeof(long long);
	    } else {
		length = sizeof(long);
	    }
	    break;
	case 'z':
	    length = sizeof(size_t);
	    break;
	case 't':
	    length = sizeof(intptr_t);
	    break;
	default: --format; // undo ++
	}
	/* Type:
	 * 'd', 'i': signed decimal
	 * 'u'     : unsigned decimal
	 * 'x', 'X': unsigned hexadecimal (UPPER case)
	 * 'p'     : signed hexadecimal of a pointer
	 * 'c'     : character
	 * 's'     : string
	 * '%'     : literal '%'
	 */
	int base = 10;
	uint64_t num = 0;
	switch(*format++) {
	case 'd':
	case 'i':
	    switch(length) {
	    case 1: num = (int8_t) va_arg(args, int); break;
	    case 2: num = (int16_t)va_arg(args, int); break;
	    case 4: num = (int32_t)va_arg(args, int); break;
	    case 8: num = (int64_t)va_arg(args, int64_t); break;
	    }
	    flags.sign = true;
	    if (precision == -1) precision = 0;
	    cprint_int(callback, state, num, base, width, precision, flags);
	    break;
	case 'p':
	    flags.alternate = true;
	    if (precision == -1) precision = 2 * sizeof(void*);
	case 'X': flags.upper = true;
	case 'x': base = 16; flags.space = false; flags.zeropad = true;
	case 'u':
	    switch(length) {
	    case 1: num = (uint8_t) va_arg(args, int); break;
	    case 2: num = (uint16_t)va_arg(args, int); break;
	    case 4: num = (uint32_t)va_arg(args, int); break;
	    case 8: num = (uint64_t)va_arg(args, uint64_t); break;
	    }
	    if (precision == -1) precision = 0;
	    cprint_int(callback, state, num, base, width, precision, flags);
	    break;
	case 'c':
	    callback((char)va_arg(args, int), state);
	    break;
	case 's': {
	    char* s = va_arg(args, char*);
	    if (precision == -1) {
		while(*s != 0) {
		    callback(*s++, state);
		}
	    } else {
		while(precision > 0 && *s != 0) {
		    --precision;
		    callback(*s++, state);
		}
	    }
	    break;
	}
	case '%':
	    callback('%', state);
	    break;
	default: // Unknown placeholder, rewind and copy '%' verbatim
	    while(*format != '%') --format;
	    callback(*format++, state);
	}
    }
}

/* vsnprintf - Format a string and place it in a buffer
 * @buf:    Buffer for result
 * @size:   Size of buffer including trailing '\0'
 * @format: Format string for output
 * @args:   Arguments for format string
 *
 * Returns the number of characters which would be generated for the given
 * input, excluding the trailing '\0', as per ISO C99. If the result is
 * greater than or equal to @size, the resulting string is truncated.
 */
int vsnprintf(char* buf, size_t size, const char* format, va_list args) {
    BufferState state = (BufferState){buf, size};
    vcprintf((vcprintf_callback_t)buffer_add, (void*)&state, format, args);
    // terminate string if there is space in the buffer
    buffer_add('\0', &state);
    // always terminate string even if there was no space
    buf[size - 1] = '\0';
    return state.pos - buf - 1;
}

void cprintf(vcprintf_callback_t callback, void *state, const char* format,
	     ...) {
    va_list args;
    va_start(args, format);
    vcprintf(callback, state, format, args);
    va_end(args);
}



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


typedef struct{
	tMiniProcess* list[SCHEDULER_MAX_NUM_PROCESSES];
	uint32_t capacity;
	uint32_t head;
	uint32_t tail;
}tQueue;

//The different queues
static tQueue q_ready_real_time;
static tQueue q_blocked_real_time;
static tQueue q_ready_system;
static tQueue q_blocked_system;
static tQueue q_ready_user;
static tQueue q_blocked_user;
static tQueue q_ready_batch;
static tQueue q_blocked_batch;
static tQueue q_ready_idle;

static tQueue* queue_map[PROC_QUEUE_TOTAL_NUM_OF_QUEUES];

void proc_queue_init(){

  queue_map[ProcQueueReadyRealTime] = &q_ready_real_time;
  queue_map[ProcQueueReadySystem] = &q_ready_system;
  queue_map[ProcQueueReadyUser] = &q_ready_user;
  queue_map[ProcQueueReadyBatch] = &q_ready_batch;
	queue_map[ProcQueueReadyIdle] = &q_ready_idle;
  queue_map[ProcQueueBlockedRealTime] = &q_blocked_real_time;
  queue_map[ProcQueueBlockedSystem] = &q_blocked_system;
  queue_map[ProcQueueBlockedUser] = &q_blocked_user;
  queue_map[ProcQueueBlockedBatch] = &q_blocked_batch;

	for(uint32_t i=0; i<PROC_QUEUE_TOTAL_NUM_OF_QUEUES; i++){
		queue_map[i]->head = 0;
		queue_map[i]->tail = 0;
		queue_map[i]->capacity = SCHEDULER_MAX_NUM_PROCESSES;
	}

}

void proc_queue_enqueue(tProcQueueId queue_id, tMiniProcess* proc){
  tQueue* q = queue_map[queue_id];

  q->list[q->tail] = proc;
  q->tail = (q->tail + 1) % q->capacity;
}

tMiniProcess* proc_queue_dequeue(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];
  tMiniProcess* proc;

  proc =  q->list[q->head];
  q->head = (q->head + 1) % q->capacity;

  return proc;
}

tMiniProcess* proc_queue_peek(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];

  return q->list[q->head];
}

uint32_t proc_queue_size(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];

  return q->head - q->tail;
}



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

#define TICK_FREQ		SYS_SCHED_CONTEXT_SWITCH_FREQ

extern void process_thread_delete(void);
extern void idle_process_thread(void);

//This is where thread/processes data strutures
//are allocated memory. All thread pointers passe around
//ultimately point to this.
static tThreadPool thread_pool;

static uint32_t proc_started_count = 0;	//How many processes have been started
																				//since the beginning of time
/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){
	//Init context switcher
	context_switcher_init(TICK_FREQ);

	//Init thread pool
	for(uint32_t i=0; i<SCHEDULER_THREAD_POOL_SIZE; i++){
		tMiniProcess proc;
		proc.state = ProcessStateFree;
		thread_pool.list[i] = proc;
		thread_pool.count++;
	}


	//Init ready and blocked queues
	proc_queue_init();

	//Init process stack
	static tMemRegion stack_memreg;
	hal_memreg_read( MemRegUserStack, &stack_memreg );
	stack_init( (uint32_t*)stack_memreg.base );	//stack_init( epstack )

	//No processes
	proc_started_count = 0;

	//Create idle process/thread
	//(we need one!)
	scheduler_thread_create( idle_process_thread, "idle process thread", 256, ProcQueueReadyIdle );
}

/*
*	Scheduler Process Create
*
*	Creates a process from a binary in nvmem. Ticking here begins!
*/
uint32_t scheduler_process_create( uint8_t* binary_file_name, const char* name, uint32_t* loader_rval, tProcQueueId destination_queue_id ){
	//tMemRegion proc_memregion;

	//Load app binary
	//uint32_t rval = loader_load_app( binary_file_name, &proc_memregion, &stack_sz );
	//	if(  rval != LOADER_LOAD_SUCCESS ){
	//		*loader_rval = rval;				//populate loader error
	//		return SCHEDULER_PROCESS_CREATE_FAILED;
	//	}

	//LOAD BIN ... THEN CALL scheduler_thread_create


	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

/*
*	Scheduler Thread Create
*
*	Creates a Thread
*
*   It's really a process, but processes are so simple, that there's
*   no  difference between a process and a thread.
*
*/
uint32_t scheduler_thread_create( void(*thread_code)(void), const char* name, uint32_t stack_sz, tProcQueueId destination_queue_id ){
	tMiniProcess* proc = thread_pool_get_one();

	if(proc == NULL)
			faults_kernel_panic("Failed to schedule. Thread pool exhausted.");

	//Set process info
	proc->name = name;
	proc->state = ProcessStateReady;
	proc->queue_id = destination_queue_id;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc->sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc->sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc->sp[OFFSET_PC] =     ((uint32_t) (thread_code +1));
	proc->sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_started_count++;

	//Send to ready queue
	proc_queue_enqueue(destination_queue_id, proc);

	//Start ticking on first process
	// Proc 1 = Idle thread
	// Proc 2 = this thread (FIRST actual process)
  if( proc_started_count == 2 ){
		tMiniProcess* idle_thread = proc_queue_peek(ProcQueueReadyIdle);
		hal_cpu_set_psp( (uint32_t)idle_thread->sp );	//or else the first tick fails

		context_switcher_begin_ticking();
  }

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

/*
*		 Scheduler Next Process
*
*    Slect next process to run from all the ready queues
*		 as per some priority
*/
static tProcQueueId queue_ids[] = {
	ProcQueueReadyRealTime,	//priority 1
	ProcQueueReadySystem,		//priority 2
	ProcQueueReadyUser,		  //priority 3
	ProcQueueReadyBatch,	  //...
	ProcQueueReadyIdle
};

tMiniProcess* scheduler_proc_next(tMiniProcess* active_proc){

	if(active_proc->state != ProcessStateNull){
		//Queue active process back to where it was
		proc_queue_enqueue(active_proc->queue_id, active_proc);	//enqueue(active)
	}

	//Find next
	tMiniProcess* next;

	for(uint32_t i=0; i<PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES; i++){
			if( proc_queue_size(queue_ids[i]) > 0 ){
				next = proc_queue_dequeue(queue_ids[i]);	//next = dequeue()
				break;
			}
	}

	if(next == NULL)
			faults_kernel_panic("Failed to schedule. No process found in ready queues.");

	return next;
}


void scheduler_process_current_stop(void){
	//changes thread state to dead
	//(so it's not scheduled anymore)
	//active_proc.state =  ProcessStateDead;

	//context switches
	context_switcher_trigger();
}


tMiniProcess* thread_pool_get_one(){
	for(uint32_t i=0; i<SCHEDULER_THREAD_POOL_SIZE; i++){
		if( thread_pool.list[i].state == ProcessStateFree )
			return &(thread_pool.list[i]);
	}

	return NULL; //We're out of threads
}



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

/*	Puts the processor to sleep. executes in user mode  */
__attribute__((naked)) void idle_process_thread(){
     __asm volatile(
        "wfi\n" \
        "b =idle_process_thread"
      );
}

void process_thread_delete(){
  /*	Deletes the current process/thread */
  //Unimplemented
  while(1);

}



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


void spin_lock_acquire(){
  //I need an actual spin lock here....
  //disabling interrupts is outrageous =P
  //
  //LAter can aslo add a MUTEX so the thread goes to sleep
  //instead of waiting.... pros and cons i guess...
  __asm volatile ("cpsid  i");
}

void spin_lock_release()
{
  __asm volatile ("cpsie  i");
}



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

/**
 * @file	stack.c
 * @author
 * @version
 *
 * @brief Stack module. This is a generic module to handle allocation of memory in the stack.
 *		  It assumes a full-descendant stack, and ensures 8-byte alignment on allocation.
 *		  This module must be ported when porting MiniOS
 *
 *		  Careful with mixing integer arithmetic with pointer arithmetic.
 */

static void alloc(uint32_t);
static void align_to_eight_byte_boundary(void);

static uint32_t* stack;
static bool initialized = false;

void stack_init(uint32_t* address){
	stack = address;
	initialized = true;
}

uint32_t* stack_top(void){
	if( !initialized )
		faults_kernel_panic( "Reading uninit stack" );

	return stack;
}

void stack_alloc(uint32_t elements){
	if( !initialized )
		faults_kernel_panic( "Allocation on uninit stack" );

	alloc(elements);
	align_to_eight_byte_boundary();
}

static void alloc(uint32_t elements){
	stack = stack - elements;
}


static void align_to_eight_byte_boundary(void){
	uint32_t address = (uint32_t)stack;

	while( address % 8 != 0 ){
		address--;
	}

	stack = (uint32_t*)address;
}



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



