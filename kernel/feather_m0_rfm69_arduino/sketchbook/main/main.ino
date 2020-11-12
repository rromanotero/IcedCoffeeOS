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

extern tPioPin led_pin;				 //Defined as part of the HAL (in HAL IO)
extern tSerialPort serial_usb;

void tick_callback(){
  hal_io_pio_write(&led_pin, !hal_io_pio_read(&led_pin));
  hal_io_serial_puts(&serial_usb, "tick\n\r");
}

void ARDUINO_MAIN() {

  system_init();

  hal_cpu_systimer_start(1000, tick_callback);

  volatile uint32_t a = 7;
  volatile uint32_t b = 0;
  volatile uint32_t c = 0;
  while(true){
    hal_io_serial_puts(&serial_usb, "Div by Zero! 7\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 6\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 5\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 4\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 3\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 2\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 1\n\r");
    hal_cpu_delay(2000);
    hal_io_serial_puts(&serial_usb, "Div by Zero! 0\n\r");
    hal_cpu_delay(2000);
    c = c + a/b;
    c = c/0; ///<<----- hardfault
  }

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
    for(volatile int i=0; i<KERNEL_PANIC_LED_BLINKING_WAIT; i++);
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
*	CPU Sleep
*
*/
void hal_cpu_sleep(void){
  asm volatile( "wfi" );
}

////////////////////////////////////////////////////////////////////////////
/////                      C Area                                     //////
///////////////////////////////////////////////////////////////////////////

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

void pendSVHook(void){

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



