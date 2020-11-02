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

#include <stdbool.h>
#include <stdint.h>
#include "hal.h"

void ARDUINO_MAIN() {

  tPioPin led_pin;
  hal_io_pio_create_pin(&led_pin, PioA, 8, PioOutput);

  tSerialPort serial_usb;
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);

  while(!hal_io_serial_is_ready(&serial_usb));

  while(true){
    hal_io_pio_write(&led_pin, true);
    hal_cpu_delay(1000);

    hal_io_pio_write(&led_pin, false);
    hal_cpu_delay(1000);

    hal_io_serial_puts(&serial_usb, "Feather M0\n\r");
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}

//============================================================
//////////////////////////////////////////////////////////////
//////////           HAL PARALLEL IO                    //////
//////////////////////////////////////////////////////////////
//============================================================


/**
*	PIO Create
*
*  Creates a PIO pin
*
*	 Given a tPioPin* p, it populates p->internal_rep so when
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
      if(pin_number == 2) pio_pin->internal_rep = 4;
      else if(pin_number == 4) pio_pin->internal_rep = 9;
      else if(pin_number == 5) pio_pin->internal_rep = 10;
      else if(pin_number == 8) pio_pin->internal_rep = 13;
      else if(pin_number == 10) pio_pin->internal_rep = 15;
      else if(pin_number == 11) pio_pin->internal_rep = 16;
      else if(pin_number == 12) pio_pin->internal_rep = 21;
      else if(pin_number == 16) pio_pin->internal_rep = 25;
      else if(pin_number == 17) pio_pin->internal_rep = 26;
      else if(pin_number == 18) pio_pin->internal_rep = 27;
      else if(pin_number == 19) pio_pin->internal_rep = 28;
      else if(pin_number == 20) pio_pin->internal_rep = 29;
      else if(pin_number == 22) pio_pin->internal_rep = 31;
      else if(pin_number == 23) pio_pin->internal_rep = 32;
      else status = HAL_IO_PIO_PIN_NOT_FOUND;

			break;
		case PioB:
      if(pin_number == 2) pio_pin->internal_rep = 47;
      else if(pin_number == 8) pio_pin->internal_rep = 7;
      else if(pin_number == 9) pio_pin->internal_rep = 8;
      else status = HAL_IO_PIO_PIN_NOT_FOUND;

      break;
		default :
			status = HAL_IO_PIO_PORT_NOT_FOUND;
	}

  //set pin direction
  switch(dir){
		case PioOutput:	  pinMode(pio_pin->internal_rep, OUTPUT); break;
		case PioInput:	  pinMode(pio_pin->internal_rep, INPUT);  break;
	}

  return status;
}


/**
*	PIO Write
*
*	Writes a state/level to a PIO pin
*/
void hal_io_pio_write(tPioPin* pio_pin, bool state){
  digitalWrite( pio_pin->internal_rep, state? HIGH : LOW );
}

/**
*	PIO Read
*
*	Reads a PIO pin state/level
*/
bool hal_io_pio_read(tPioPin* pio_pin){
	//bool state = ioport_get_pin_level(pio_pin->internal_rep);
	//return state;
}

//============================================================
//////////////////////////////////////////////////////////////
//////////                 HAL ADC                      //////
//////////////////////////////////////////////////////////////
//============================================================

/**
*
*	ADM Start
*
*	Starts the ADC, but does NOT stipulate any specific ADC channels to enable.
*
*/
void hal_io_adc_channel_start(){

}

/**
*	ADC Channel Enable
*
*	Begins ADC conversion on given ADC channel
*/
void hal_io_adc_channel_enable(tAdcChannel* adc_chan){

}

/**
*	ADC Channel Disable
*
*	Stops ADC conversion on given ADC channel
*/
void hal_io_adc_channel_disable(tAdcChannel* adc_chan){

}

/**
*	ADC Channel Status
*
*	Returns 1 if channel is enabled, 0 otherwise
*/
uint32_t hal_io_adc_channel_status(tAdcChannel* adc_chan){

}

/**
*	ADC Channel value
*
*	Returns a value between 0 - 4096
*/
uint32_t hal_io_adc_channel_read(tAdcChannel* adc_chan){

}

//============================================================
//////////////////////////////////////////////////////////////
//////////          HAL IO SERIAL                       //////
//////////////////////////////////////////////////////////////
//============================================================



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
        serial_port->internal_rep = 1;  //Serial 1
        break;
  		default:
        return HAL_IO_SERIAL_PORT_NOT_FOUND;
  	}

    serial_port->id = SerialA;
    serial_port->baudrate = baudrate;
    serial_port->io_type = IoPoll;

    //Begin Serial Port
    Serial.begin(baudrate);
  }
  else{
    return HAL_IO_TYPE_NOT_FOUND;
  }

  return HAL_SUCCESS;
}

/**
*	Serial putc
*
*	Writes a string to the specified Serial port
*
*/
bool hal_io_serial_is_ready( tSerialPort* serial_port ){
  switch(serial_port->internal_rep){
    case 1:
        return Serial; //Serial1
    default: break;
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
  switch(serial_port->internal_rep){
    case 1:  Serial.write(c); break; //Serial1
    default: break;
  }
}

/**
*	Serial getc
*
*	Reads a character from the specified Serial port
*/
uint8_t hal_io_serial_getc( tSerialPort* serial_port ){

}


/*
*  HAL IO Serial Hex Dump (512 BYtes)
*
*  Hex Dumps 512 bytes starting from ptr_to_first_byte
*/
void hal_io_serial_hex_dump_512_bytes( tSerialPort* serial_port, uint8_t* ptr_to_first_byte){
	//uart0_dump(ptr_to_first_byte);
}


/*
*  HAL IO Put Hex Dump 64-bits
*
*  Writes a 64-bit hex value to id
*/
void hal_io_serial_puthex_64_bits( tSerialPort* serial_port, uint64_t value){
	//uart0_puthex_64_bits(value);
}


//============================================================
//////////////////////////////////////////////////////////////
//////////             HAL IO PMW                       //////
//////////////////////////////////////////////////////////////
//============================================================
/**
*
*	PWM Start
*
*	Starts the PWM Peripheral
*/
void hal_io_pwm_start( void ){


}

/**
*
*	PWM Channel Start
*
*	Starts the PWM Channel associated on a given PIO pin. If there's no PWM
*	associated with the given pin, nothing happens.
*
*   Period is given in steps of 0.1ms (with a maximum of X millisecond), while duty cycle is given
*   percentage. Example for creating a square wave of a 1ms period:
*
*	channel.period = 10;
*   channel.duty_dycle = 50;
*   hal_io_pwm_start( &channel );
*
*/
void hal_io_pwm_channel_start( tPwmChannel* channel ){

}

/**
*	PWM Stop
*
*	Stops a PWM
*/
void hal_io_pwm_channel_stop( tPwmChannel* channel ){

}

/**
*	PWM Write
*
*	Updates period and duty cycle for a PWM
*/
void hal_io_pwm_channel_write( tPwmChannel* channel ){

}


//============================================================
//////////////////////////////////////////////////////////////
//////////              HAL CPU                        //////
//////////////////////////////////////////////////////////////
//============================================================
/**
*	HAL CPU Init
*
*	Initializes the CPU. This function must be called before
*	HAL IO Init. That is: hal_cpu_init(); hal_io_init();
*/
void hal_cpu_init(void){

}


/**
*	Low Priority Software Interrupt Trigger
*
*	Triggers a PendsSV Exception
*/
void hal_cpu_lowpty_softint_trigger(void){

}

/**
*	Low Priority Software Interrupt Register Callback
*
*	Registers a callback function for the PendSV Exception
*
*	@param callback the function that gets called on PendSV exception
*/
void hal_cpu_lowpty_softint_register_callback( void(*callback)(void) ){

}

/**
*	SystemTimer Start
*
*	Starts the System Timer
*
*	@param tick_freq_in_ms the tick frequency in milliseconds
*	@param callback function to be called when a tick occurs
*/
void hal_cpu_systimer_start(uint32_t tick_freq_in_ms, void(*callback)(void)){

}

/**
*	SystemTimer Stop
*
*	Stops the system timer
*
*/
void hal_cpu_systimer_stop(void){

}

/**
*	SystemTimer reestart
*
*	Once started, this function can be used to re-estart the system timer
*	with the same configuration.
*
*/
void hal_cpu_systimer_reestart(void){

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
