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

#ifndef SYSTEM_H_
#define SYSTEM_H_

#define SYS_VERSION					       "3"
#define SYS_NAME					         "IcedCoffeeOS"
#define SYS_USERNAME_MAX_LENGTH		  10
#define SYS_CONF_SERIAL_A_BAUDRATE	115200
#define SYS_CONF_SERIAL_B_BAUDRATE	115200

#define SYS_SYSTEM_MEM_AVAILABLE	16384							//Total SYSTEM mem available is 16Kb of 32KB Total
#define SYS_USER_MEM_AVAILABLE		16384							//Total USER mem available is 16Kb of 32KB Total

#define SYS_USER_STACK_MAX_SIZE		   (SYS_USER_MEM_AVAILABLE/2)		//HALF of Total User Mem Available
#define SYS_APP_MAX_SIZE			       (SYS_USER_MEM_AVAILABLE/4)		//QUARTER of Total User Mem Available
#define SYS_LOADER_BUFFER_MAX_SIZE	 (SYS_USER_MEM_AVAILABLE/4)		//QUARTER of Total User Mem Available

//#define SYS_APP_DEFAULT_NAME		"App.bin"
#define SYS_CONSOLE_NEWLINE			"\n\r"		//Some serial console like putty use \n\r as newlines,
												//others like Atmel's Terminal use only \n. Choose accordingly.

#define SYS_MAX_NUM_OF_PROCESSES	10								//Max number of processes supported


#define SYS_PANIC_MSG_MAX_LENGTH	50
#define SYS_PANIC_LED_BLINKING_WAIT 240000   //The smaller, the faster the Kenel Panic LED Blinks



#endif /* SYSTEM_H_ */
