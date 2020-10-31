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

#include "adc_sensor.h"
#include "atmel_start.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Location of internal temperature sensor calibration data */
#define SOFTWARE_TEMPERATURE_LOG_ROW_ADDRESS 0x00806030

/*! \brief Initialize ADC module to use TEMP sensor
 *
 */
void configure_adc(void)
{
	/* Enable temperature sensor */
	hri_sysctrl_set_VREF_reg(SYSCTRL, SYSCTRL_VREF_TSEN);
	/* Load in the fixed device ADC calibration constants */
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL((*(uint32_t *)ADC_FUSES_BIASCAL_ADDR >> ADC_FUSES_BIASCAL_Pos))
	                 | ADC_CALIB_LINEARITY_CAL((*(uint64_t *)ADC_FUSES_LINEARITY_0_ADDR >> ADC_FUSES_LINEARITY_0_Pos));
	/* Enable ADC*/
	adc_sync_enable_channel(&ADC_TEMPERATURE, 0);
}

/*! \brief Get calibrated internal temperature.
 * \return Returns the temperature in units of C as a float value.
 */
float get_internal_temperature(void)
{
	uint16_t ADC_m;
	uint8_t  ADC_temp[2];

	/* Calibration routine from SAM D21 data sheet */
	static temperature_log_row log_row;
	log_row = *(temperature_log_row *)SOFTWARE_TEMPERATURE_LOG_ROW_ADDRESS;
	float temp_R
	    = log_row.log_row_struct.ROOM_TEMP_VAL_INT + ((float)(log_row.log_row_struct.ROOM_TEMP_VAL_DEC) / 10.0f);
	float temp_H = log_row.log_row_struct.HOT_TEMP_VAL_INT + ((float)(log_row.log_row_struct.HOT_TEMP_VAL_DEC) / 10.0f);
	uint32_t ADC_R   = log_row.log_row_struct.ROOM_ADC_VAL;
	uint32_t ADC_H   = log_row.log_row_struct.HOT_ADC_VAL;
	float    INT1V_R = 1 - (float)log_row.log_row_struct.ROOM_INT1V_VAL / 1000.0f;
	float    INT1V_H = 1 - (float)log_row.log_row_struct.HOT_INT1V_VAL / 1000.0f;

	/* Sync Read ADC result */
	adc_sync_read_channel(&ADC_TEMPERATURE, 0, ADC_temp, 2);
	ADC_m = (ADC_temp[1] << 8) | ADC_temp[0];
	TEMP_INFO("Tsens: %X\r\n", ADC_m);

	/* Get coarse temperature value, note: no need to do the divisions by
	 * (2^12-1), each term in both the divisor and the dividend has these
	 *  factors, so they cancel nicely*/
	float temp_c
	    = temp_R + ((((ADC_m * 1) - (ADC_R * INT1V_R)) * (temp_H - temp_R)) / ((ADC_H * INT1V_H) - (ADC_R * INT1V_R)));
	/* Calculate the approximate INT1V value from the first approximation above */
	float INT1V_m = INT1V_R + ((INT1V_H - INT1V_R) * (temp_c - temp_R)) / (temp_H - temp_R);

	/* Get fine temperature value */
	float temp_f
	    = temp_R
	      + ((((ADC_m * INT1V_m) - (ADC_R * INT1V_R)) * (temp_H - temp_R)) / ((ADC_H * INT1V_H) - (ADC_R * INT1V_R)));
	return temp_f;
}

#ifdef __cplusplus
}
#endif
