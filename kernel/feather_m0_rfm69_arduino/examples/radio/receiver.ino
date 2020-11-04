#include "hal.h"

#define RECEIVER_ADDRESS  0
#define TRANSMITTER_ADDRESS  1

void ARDUINO_MAIN() {

  tSerialPort serial_usb;
  hal_io_serial_create_port(&serial_usb, SerialA, IoPoll, 115200);

  while(!hal_io_serial_is_ready(&serial_usb));

  tRadioTransceiver radio;
  tRadioMessage message;
  hal_radio_create_transceiver(&radio, RadioA, RECEIVER_ADDRESS, HAL_RADIO_TX_POWER_MAX/2);

  while(true){

    if( hal_radio_available(&radio) ){

      //read
      hal_radio_read(&radio, &message);

      //Send over serial
      for(int i=0; i<message.len; i++)
        hal_io_serial_putc(&serial_usb, message.payload[i]);

      hal_io_serial_puts(&serial_usb, "\n\r");
    }
  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
