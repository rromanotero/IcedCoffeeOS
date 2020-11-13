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

void my_thread(void){
  while(true){
    hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
    hal_cpu_delay(1000);
  }
}

void ARDUINO_MAIN() {
  system_init();
  scheduler_thread_create( my_thread, "my thread", 512 );

  while(true);

  //Exit so we don't
  //loop over and over
  exit(0);
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


/**
*	System Init
*
*	Initializes everything. Must be called before any other call
*/
void system_init(void){
	hal_cpu_init();
	hal_io_init();
	hal_radio_init();
	hal_timer_init();
	faults_init();
	scheduler_init();
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
  while(1); //we don't have appas yet
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
	//For compatibility
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
*	Low Priority Software Interrupt Register Callback
*
*	Registers a callback function for the PendSV Exception
*
*	@param callback the function that gets called on PendSV exception
*/
void hal_cpu_lowpty_softint_register_callback( void(*callback)(void) ){
	pendsv_callback = callback;
}

/**
*	SystemTimer Stop
*
*	Stops the system timer
*
*/
void hal_cpu_systimer_stop(void){
	SysTick->VAL   = 0;								/* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |	/* Disable SysTick IRQ and SysTick Timer */
	SysTick_CTRL_TICKINT_Msk   |
	(0UL << SysTick_CTRL_ENABLE_Pos);
}

/**
*	SystemTimer reestart
*
*	Once started, this function can be used to re-estart the system timer
*	with the same configuration.
*
*/
void hal_cpu_systimer_reestart(void){
	SysTick->VAL   = 0;								// Load the SysTick Counter Value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |	// Enable SysTick IRQ and SysTick Timer
	SysTick_CTRL_TICKINT_Msk   |
	SysTick_CTRL_ENABLE_Msk;
}

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
*	SVC Start
*
*	Starts SVC calls and registers a callback function. The callback
*	execution of an SVC instruction
*
*	@param callback the function that gets called on supervisor calls
*/
void hal_cpu_svc_start( void(*callback)(void) ){
  svc_callback = callback; //SVC Handler definition is in hal_cpu_asm.s
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
    case SerialA: return serial_port->internal_driver_a;
    case SerialB: return serial_port->internal_driver_b;
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
			memreg->size =  4098; //FIXED FOR NOWW....  // &__stack_size__;
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

/**
*	HAL Timer Init
*/
void hal_timer_init(void){
	//For compatibility
}



#define TICK_FREQ		    3
#define CONTEXT_SIZE    16
#define INITIAL_APSR    (1 << 24) //Bit 24 is the Thumb bit
#define OFFSET_LR       13
#define OFFSET_PC       14
#define OFFSET_APSR     15

__attribute__((naked)) static void tick_callback(void);
__attribute__((naked)) static void low_pty_callback(void);

extern void process_thread_delete(void);
extern void idle_process_thread(void);

static tProcessList proc_list;			// process list
static tMiniProcess* wait_list[10];		//waiting list
static tMiniProcess* active_proc;		//The active process
static uint32_t proc_count = 0;


/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){
  //A null process (used to mark the lack of an active process)
  static tMiniProcess null_proc;
  null_proc.name = "null";
  null_proc.state = ProcessStateNull;

	//Init process stack
	static tMemRegion stack_memreg;
	hal_memreg_read( MemRegUserStack, &stack_memreg );
	stack_init( (uint32_t*)stack_memreg.base );	//stack_init( epstack )

	//Inits Low Pty Int
	hal_cpu_lowpty_softint_register_callback( low_pty_callback );

	//Active process is the null process
	active_proc = &null_proc;

	//No processes
	proc_count = 0;

	//Create idle process/thread
	//(we need one!)
	scheduler_thread_create( idle_process_thread, "idle process thread", 256 );
}

/*
*	The infamous tick callback
*
*	Context switch takes place here.
*/
__attribute__((naked)) static void tick_callback(void){

	//save software context
	hal_cpu_save_context();

	//Not the null process?
	//(this'll skiip hal_cpu_get_psp() on the very first tick)
	if( active_proc->state != ProcessStateNull ){
		//save SP
		active_proc->sp = (uint32_t*)hal_cpu_get_psp();
	}

	//get next active process
	active_proc = scheduling_policy_next( active_proc, &proc_list );

	//restore SP
	hal_cpu_set_psp( (uint32_t)(active_proc->sp) );

	//restore software context
	hal_cpu_restore_context();

	//give CPU to active process
	hal_cpu_return_exception_user_mode();
}

/*
*	Low Priority Callback
*	(This callback is interruptible by the system timer)
*
*	Context switch takes place here.
*/
__attribute__((naked)) static void low_pty_callback(void){

	//save software context
	hal_cpu_save_context();

	//stop systick (else race conditions)
	hal_cpu_systimer_stop();

	//Not the null process?
	//(this'll skiip hal_cpu_get_psp() on the very first tick)
	if( active_proc->state != ProcessStateNull ){
		//save SP
		active_proc->sp = (uint32_t*)hal_cpu_get_psp();
	}

	//get next active process
	active_proc = scheduling_policy_next( active_proc, &proc_list );

	//restore SP
	hal_cpu_set_psp( (uint32_t)(active_proc->sp) );

	//restart system timer (from 0)
	hal_cpu_systimer_reestart();

	//restore software context
	hal_cpu_restore_context();

	//give CPU to active process
	hal_cpu_return_exception_user_mode();
}

/*
*	Scheduler Process Create
*
*	Creates a process from a binary in nvmem. Ticking here begins!
*/
uint32_t scheduler_process_create( uint8_t* binary_file_name, const char* name, uint32_t* loader_rval ){
	tMemRegion proc_memregion;
	uint32_t stack_sz;

	//Load app binary
	//uint32_t rval = loader_load_app( binary_file_name, &proc_memregion, &stack_sz );
	//if(  rval != LOADER_LOAD_SUCCESS ){
	//	*loader_rval = rval;				//populate loader error
	//	return SCHEDULER_PROCESS_CREATE_FAILED;
	//}

	//Set process info
	proc_list.list[proc_list.count].name = name;
	proc_list.list[proc_list.count].state = ProcessStateReady;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc_list.list[proc_list.count].sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc_list.list[proc_list.count].sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc_list.list[proc_list.count].sp[OFFSET_PC] =     ((uint32_t) (proc_memregion.base +1));
	proc_list.list[proc_list.count].sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_list.count++;

	//Start ticking on first process (idle thread is process/thread 1)
	if( proc_list.count == 2 ){
		hal_cpu_set_psp( (uint32_t)(proc_list.list[0].sp) );						//or else the first tick fails
		hal_cpu_systimer_start( TICK_FREQ, tick_callback );
	}

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
uint32_t scheduler_thread_create( void(*thread_code)(void) , const char* name, uint32_t stack_sz ){

	//Set process info
	proc_list.list[proc_list.count].name = name;
	proc_list.list[proc_list.count].state = ProcessStateReady;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc_list.list[proc_list.count].sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc_list.list[proc_list.count].sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc_list.list[proc_list.count].sp[OFFSET_PC] =     ((uint32_t) (thread_code +1));
	proc_list.list[proc_list.count].sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_list.count++;

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

void scheduler_process_current_stop(void){
	//changes thread state to dead
	//(so it's not scheduled anymore)
	active_proc->state =  ProcessStateDead;

	//context switches
	hal_cpu_lowpty_softint_trigger();
}




/*	Puts the processor to sleep. executes in user mode  */
void idle_process_thread(){
   while(true){
     hal_cpu_sleep();
   }
}

void process_thread_delete(){
/*	Deletes the current process/thread. this function will be
 accessed in user mode when a thread "returns" hence the syscall  */
  // svc 29 /* thread_stop system call see syscalls.c */
   //b .		/* execution won't get here */
}



uint32_t process_mark = 0;

/*
* NOTE: This needs a better name. Scheduling Policy sounds so
*       INSURANCEsy
*
*       I believe this is round robin! It jsut looks different thatn what
*		it appears on textsbooks.
*/
tMiniProcess* scheduling_policy_next( tMiniProcess* active_proc, tProcessList* proc_list  ){

	//Increment process mark
	//(skip dead processes)
	uint32_t count=0;
	do{
		count++;
		process_mark = (process_mark + 1) % proc_list->count;
	}while( proc_list->list[process_mark].state == ProcessStateDead );


	//Return process at process mark
	return &(proc_list->list[process_mark]);
}



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



