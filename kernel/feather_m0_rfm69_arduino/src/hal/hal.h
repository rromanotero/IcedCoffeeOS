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
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Servo.h>

#define ARDUINO_KERNEL_MAIN  loop
void setup() {} /* Don't need it*/

#define HAL_SUCCESS  									1
#define HAL_IO_TYPE_NOT_FOUND						101
#define HAL_IO_PIO_PORT_NOT_FOUND     	102
#define HAL_IO_PIO_PIN_NOT_FOUND      	102
#define HAL_IO_SERIAL_PORT_NOT_FOUND		103
#define HAL_IO_ADC_CHANNEL_NOT_FOUND    104
#define HAL_IO_PWM_CHANNEL_NOT_FOUND    105
#define HAL_RADIO_INIT_FAILED						106
#define HAL_RADIO_TRANSCEIVER_NOT_FOUND	107
#define HAL_RADIO_READ_FAILED						108
#define HAL_RADIO_SEND_FAILED           109

#define HAL_IO_SERIAL_CHAR_NOT_READY       0
#define HAL_IO_SERIAL_CHAR_READY           1

#define HAL_RADIO_TX_POWER_MIN        14
#define HAL_RADIO_TX_POWER_MAX        20
#define HAL_RADIO_MAX_MESSAGE_LEN     RH_RF69_MAX_MESSAGE_LEN
#define HAL_RADIO_ADDRESS_MIN  				0
#define HAL_RADIO_ADDRESS_MAX  				255
#define HAL_RADIO_DEFAULT_ADDRESS    	0    //Default address of this node (my address)

typedef uint32_t tPwmType;		/**< PWM Pin Type */
typedef uint32_t tSensorId;		/**< Sensor ID type */
typedef enum tIoType			 { IoPoll = 0, IoInterrupt  };
typedef enum tPioPort			 { PioA = 0, PioB, PioC, PioD };
typedef enum tPioDir		   { PioOutput = 0, PioInput } ;
typedef enum tSerialId     { SerialA = 0, SerialB, SerialC, SerialD  };
typedef enum tAdcId        { AdcA = 0, AdcB  };
typedef enum tPwmId        { PwmA = 0, PwmB  };
typedef enum tServoId      { ServoA = 0, ServoB  };
typedef enum tRadioId      { RadioA = 0, RadioB };
typedef enum tTimerId      { TimerSysTick = 0, TimerMicroseconds = 1, };
typedef enum tFaultOrigin	 { FaultApp = 0, FaultSystem };
typedef enum tMemRegionId	{ MemRegSystem = 0, MemRegApp, MemRegSystemStack, MemRegUserStack };

/**
* Memory regions
*/
typedef struct{
	tMemRegionId	id;
	uint8_t*		base;		/**< a pointer to the beginning of the region */
	uint32_t		size;		/**< size in bytes */
}tMemRegion;

typedef struct{
    uint8_t payload[HAL_RADIO_MAX_MESSAGE_LEN];
    uint32_t address;
		uint32_t len;
    uint32_t rssi;
}tRadioMessage;

typedef struct{
	tRadioId		id;
	tIoType			io_type;
  RH_RF69*    internal_driver;
}tRadioTransceiver;

typedef struct{
	uint32_t		pin_number;
	tPioPort		pio_port;
	uint32_t		internal_pin; /* How the pin is represented internally (this is hardware specific) */
}tPioPin;

typedef struct{
	tSerialId		id;
	tIoType			io_type;
	uint32_t		baudrate;
	Serial_*	  internal_driver_a;
  Uart*	      internal_driver_b;  //Serialx in Arduino are not the same tpye for some reason
}tSerialPort;


typedef struct{
  tAdcId		id;
	tIoType		io_type;
  uint32_t  internal_adc;
}tAdcChannel;

typedef struct{
  tPwmId		id;
  uint32_t  internal_pin;
}tPwmChannel;

typedef struct{
  tServoId		id;
  Servo*  internal_driver;
}tServoChannel;

typedef struct{
	uint32_t seconds; /**< seconds */
	uint32_t minutes; /**< minutes */
	uint32_t hours;   /**< hours */
	uint32_t day;     /**< day */
	uint32_t month;   /**< month */
	uint32_t year;    /**< year */
}tTime;

typedef struct{
    uint8_t* name;
    uint32_t speed_mhz;
    uint32_t num_cores;
}CPUInfo;

#endif
