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
    for(volatile int i=0; i<KERNEL_PANIC_LED_BLINKING_WAIT; i++);
  }
}
