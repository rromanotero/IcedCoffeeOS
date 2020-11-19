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

void main_user_thread(void){

  scheduler_thread_create( thread_a, "thread_a", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( thread_b, "thread_b", 1024, ProcQueueReadyRealTime );
  scheduler_thread_create( thread_led, "thread_led", 1024, ProcQueueReadyRealTime );

  while(true){
    hal_io_serial_puts(&serial_usb, "Main Thread (LED is in its own thread)\n\r");
    for(volatile int i=0; i<480000*5;i++);
  }
}

void thread_a(void){
  while(true){
    hal_io_serial_puts(&serial_usb, "Thread A\n\r");
    for(volatile int i=0; i<480000*2;i++);
  }
}

void thread_b(void){
  while(true){
    hal_io_serial_puts(&serial_usb, "Thread B\n\r");
    for(volatile int i=0; i<480000*3;i++);
  }
}

void thread_led(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    for(volatile int i=0; i<480000;i++);
  }
}

void ARDUINO_KERNEL_MAIN() {
  system_init();

  while(!hal_io_serial_is_ready(&serial_usb));

  scheduler_thread_create( main_user_thread, "main_user_thread", 1024, ProcQueueReadyRealTime );

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
 *  by myself. Thanks to Adam Heinrich. So here's the License.
 *    - Rafael
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

#define CONTEXT_SIZE    16
#define INITIAL_APSR    (1 << 24) //Bit 24 is the Thumb bit
#define OFFSET_LR       13
#define OFFSET_PC       14
#define OFFSET_APSR     15

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
      if(pin_number == 2) pio_pin->internal_pin = 4;
      else if(pin_number == 4) pio_pin->internal_pin = 9;
      else if(pin_number == 5) pio_pin->internal_pin = 10;
      else if(pin_number == 8) pio_pin->internal_pin = 13;
      else if(pin_number == 10) pio_pin->internal_pin = 15;
      else if(pin_number == 11) pio_pin->internal_pin = 16;
      else if(pin_number == 12) pio_pin->internal_pin = 21;
      else if(pin_number == 16) pio_pin->internal_pin = 25;
      else if(pin_number == 17) pio_pin->internal_pin = 26;
      else if(pin_number == 18) pio_pin->internal_pin = 27;
      else if(pin_number == 19) pio_pin->internal_pin = 28;
      else if(pin_number == 20) pio_pin->internal_pin = 29;
      else if(pin_number == 22) pio_pin->internal_pin = 31;
      else if(pin_number == 23) pio_pin->internal_pin = 32;
      else status = HAL_IO_PIO_PIN_NOT_FOUND;

			break;
		case PioB:
      if(pin_number == 2) pio_pin->internal_pin = 47;
      else if(pin_number == 8) pio_pin->internal_pin = 7;
      else if(pin_number == 9) pio_pin->internal_pin = 8;
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
//is allocated memory. All thread pointers passe around
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

	return NULL;
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



