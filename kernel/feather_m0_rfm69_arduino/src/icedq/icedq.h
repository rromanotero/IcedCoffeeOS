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
