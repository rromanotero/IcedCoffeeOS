/**
 * \file
 *
 * \brief Sensors with analog output driver
 *
 * Copyright (c) 2012-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ADC_SENSOR1_H_INCLUDED
#define ADC_SENSOR1_H_INCLUDED

#include <stdio.h>
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEMP_INFO(...) // do{printf(__VA_ARGS__);}while(0)

/* Structure to hold internal temperature sensor calibration data */
COMPILER_PACK_SET(8)
typedef union {
	struct {
		uint8_t  ROOM_TEMP_VAL_INT : 8;
		uint8_t  ROOM_TEMP_VAL_DEC : 4;
		uint8_t  HOT_TEMP_VAL_INT : 8;
		uint8_t  HOT_TEMP_VAL_DEC : 4;
		int8_t   ROOM_INT1V_VAL : 8;
		int8_t   HOT_INT1V_VAL : 8;
		uint16_t ROOM_ADC_VAL : 12;
		uint16_t HOT_ADC_VAL : 12;
	} log_row_struct;
	uint64_t raw;
} temperature_log_row;
COMPILER_PACK_RESET()

void  configure_adc(void);
float get_internal_temperature(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* ADC_SENSOR_H_INCLUDED */
