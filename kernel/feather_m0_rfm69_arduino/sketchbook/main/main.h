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

#ifndef SYSTEM_H_
#define SYSTEM_H_


#define SYSTEM_DEBUG_ON             1   //1=Turn debug output ON, 0=Turn debug output OFF


#define SYS_VERSION					       "3"
#define SYS_NAME					         "IcedCoffeeOS"
#define SYS_USERNAME_MAX_LENGTH		  10
#define SYS_CONF_SERIAL_A_BAUDRATE	115200
#define SYS_CONF_SERIAL_B_BAUDRATE	115200

#define SYS_SCHED_MAX_NUM_OF_PROCESSES	10	//Max number of processes supported
#define SYS_SCHED_CONTEXT_SWITCH_FREQ   5   //How often in milliseconds should
                                            //context switch occur

#define SYS_PANIC_MSG_MAX_LENGTH	50
#define SYS_PANIC_LED_BLINKING_WAIT 240000   //The smaller, the faster the Kenel Panic LED Blinks

#define SYS_SYSTEM_MEM_AVAILABLE	16384							//Total SYSTEM mem available is 16Kb of 32KB Total
#define SYS_USER_MEM_AVAILABLE		16384							//Total USER mem available is 16Kb of 32KB Total

#define SYS_USER_STACK_MAX_SIZE		   (SYS_USER_MEM_AVAILABLE/2)		//HALF of Total User Mem Available
#define SYS_APP_MAX_SIZE			       (SYS_USER_MEM_AVAILABLE/4)		//QUARTER of Total User Mem Available
#define SYS_LOADER_BUFFER_MAX_SIZE	 (SYS_USER_MEM_AVAILABLE/4)		//QUARTER of Total User Mem Available

//#define SYS_APP_DEFAULT_NAME		"App.bin"
#define SYS_CONSOLE_NEWLINE			"\n\r"		//Some serial console like putty use \n\r as newlines,
												                  //others like Atmel's Terminal use only \n. Choose accordingly.

#endif /* SYSTEM_H_ */



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


#ifndef CONTEXT_SWTICHER_H
#define CONTEXT_SWTICHER_H

#define CONTEXT_SIZE    16
#define INITIAL_APSR    (1 << 24) //Bit 24 is the Thumb bit
#define OFFSET_LR       13
#define OFFSET_PC       14
#define OFFSET_APSR     15
#define OFFSET_PC       14
#define OFFSET_R0       8

#endif



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


#ifndef FAULTS_H_
#define FAULTS_H_


#endif /* FAULTS_H_ */



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
  RHReliableDatagram* internal_manager;
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

#ifndef ICED_Q_H_
#define ICED_Q_H_

#define ICEDQ_MAX_BYTESTRING_LEN              32
#define ICEDQ_MAX_NUM_SUSCRIPTIONS            100

#define ICEDQ_SUCCESS                         1
#define ICEDQ_NO_MORE_SUSCRIPTIONS_AVAILABLE  2

typedef enum tIcedEncoding {
  IcedEncodingInteger = 0,
  IcedEncodingString = 0
};

typedef struct{
  const char* name;
  tIcedEncoding encoding;
}tIcedQTopic;

typedef struct{
	uint8_t* queue;
	uint32_t capacity;
	volatile uint32_t head;  //These 2 are how processes synchronize, so make them visible to
	volatile uint32_t tail;  //each other with volatile i.e. tell the compiler to not optimize them
}tIcedQQueue;

typedef struct{
  const char* topic;
  tIcedQQueue* registered_queue;
  bool free;
}tIcedQSuscription;


#endif /* ICED_Q_H_ */



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

#ifndef ICEDQLIB_H_
#define ICEDQLIB_H_



#endif /* ICEDQLIB_H_ */



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

/* Copyright (C) 2007-2015 Goswin von Brederlow <goswin-v-b@web.de>
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

#ifndef KERNEL_KPRINTF_H
#define KERNEL_KPRINTF_H


typedef void (*vcprintf_callback_t)(char c, void *state);

typedef struct {
    char *pos;
    size_t size;
} BufferState;

typedef struct Flags {
    _Bool plus:1;	// Always include a '+' or '-' sign
    _Bool left:1;	// left justified
    _Bool alternate:1;	// 0x prefix
    _Bool space:1;	// space if plus
    _Bool zeropad:1;	// pad with zero
    _Bool sign:1;	// unsigned/signed number
    _Bool upper:1;	// use UPPER case
} Flags;


#endif // #ifndef PRINTF_H



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


#ifndef PROC_QUEUE_H_
#define PROC_QUEUE_H_

#define PROC_QUEUE_TOTAL_NUM_OF_QUEUES          PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES + PROC_QUEUE_TOTAL_NUM_OF_BLOCKED_QUEUES
#define PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES    5
#define PROC_QUEUE_TOTAL_NUM_OF_BLOCKED_QUEUES  4

typedef enum tProcQueueId {
    ProcQueueReadyRealTime = 0,
    ProcQueueReadySystem,
    ProcQueueReadyUser,
    ProcQueueReadyBatch,
    ProcQueueReadyIdle,
    ProcQueueBlockedRealTime,
    ProcQueueBlockedSystem,
    ProcQueueBlockedUser,
    ProcQueueBlockedBatch
  };



#endif /* PROC_QUEUE_H_ */



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

#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#define SCHEDULER_PROCESS_CREATE_FAILED		0
#define SCHEDULER_PROCESS_CREATE_SUCCESS	1
#define SCHEDULER_MAX_NUM_PROCESSES			  SYS_SCHED_MAX_NUM_OF_PROCESSES
#define SCHEDULER_THREAD_POOL_SIZE				SCHEDULER_MAX_NUM_PROCESSES

typedef enum tProcessState {
		ProcessStateReady = 0,
		ProcessStateRunning,
		ProcessStateFree,
		ProcessStateNull
	};

typedef struct{
	const char* name;
	uint32_t* sp;
	tProcessState state;
  tProcQueueId queue_id;
}tMiniProcess;


typedef struct{
	tMiniProcess list[SCHEDULER_MAX_NUM_PROCESSES];
	uint32_t count;
}tThreadPool;

#endif /* SCHEDULER_H_ */



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

#ifndef SPIN_LOCK_H_
#define SPIN_LOCK_H_



#endif



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

#ifndef STACK_H_
#define STACK_H_



#endif /* STACK_H_ */



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

#ifndef SYSCALLS_H_
#define SYSCALLS_H_

#define SYSCALLS_TOPIC    "system.syscalls"   //TODO : Once routing keys are enabled, change this for
                                              //       topic=system, routing_key=syscalls

/**
*   System call numbers
*/
enum{
    SyscallPioCreatePin             = 0,
    SyscallPioWrite                 = 1,
    SyscallPioRead                  = 2
};

#define SYSCALLS_QUEUE_SIZE             1000
#define SYSCALLS_REQUEST_SIZE_IN_BYTES  9 //syscall_num(1) + input (4) + output (4)

#define SYSCALLS_RAW_REQUEST_NUM_OFFSET         0
#define SYSCALLS_RAW_REQUEST_INPUT_OFFSET       1
#define SYSCALLS_RAW_REQUEST_OUTPUT_OFFSET      5

typedef struct{
  volatile uint32_t arg0;
  volatile uint32_t arg1;
  volatile uint32_t arg2;
  volatile uint32_t arg3;
}tSyscallInput;

typedef struct{
  volatile uint32_t ret_val;
  volatile bool output_ready;
}tSyscallOutput;


#endif /* SYSCALLS_H_ */



