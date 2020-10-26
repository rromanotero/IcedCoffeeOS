/**
 * \file
 *
 * \brief LED driver
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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

//#include <port.h>
//#include <board.h>
#include "led.h"
//#include "node_plug_jd.h"
#include "atmel_start_pins.h"
#include <hal_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

static led_mode_t led_ctr[WIFI_LED_CNT] = {LED_OFF, LED_OFF, LED_OFF};

/**
 * \brief Configures LED working mode.
 */
void set_wifi_led_mode(led_mode_t led_mode, led_index_t led_pin_num)
{
	if (led_pin_num < WIFI_LED_CNT) {
		// avoid LED flicker
		if (led_ctr[led_pin_num] != led_mode) {
			led_ctr[WIFI_LED_Y]  = LED_OFF;
			led_ctr[WIFI_LED_R]  = LED_OFF;
			led_ctr[WIFI_LED_G]  = LED_OFF;
			led_ctr[led_pin_num] = led_mode;
			exec_wifi_led_mode();
		}
	}
}

void exec_wifi_led_mode(void)
{
	// if (get_plug_info()->plug_status.c_status[0].status_list[0].s_code == SYS_NO_ERROR) {
	// Yellow LED
	if (LED_TOGGLE == led_ctr[WIFI_LED_Y]) {
		gpio_toggle_pin_level(LED_WY_PIN);
	} else if (LED_ON == led_ctr[WIFI_LED_Y]) {
		gpio_set_pin_level(LED_WY_PIN, LED_ACTIVE);
	} else {
		gpio_set_pin_level(LED_WY_PIN, LED_INACTIVE);
	}
	// Red LED
	if (LED_TOGGLE == led_ctr[WIFI_LED_R]) {
		gpio_toggle_pin_level(LED_WR_PIN);
	} else if (LED_ON == led_ctr[WIFI_LED_R]) {
		gpio_set_pin_level(LED_WR_PIN, LED_ACTIVE);
	} else {
		gpio_set_pin_level(LED_WR_PIN, LED_INACTIVE);
	}
	// Green LED
	if (LED_TOGGLE == led_ctr[WIFI_LED_G]) {
		gpio_toggle_pin_level(LED_WG_PIN);
	} else if (LED_ON == led_ctr[WIFI_LED_G]) {
		gpio_set_pin_level(LED_WG_PIN, LED_ACTIVE);
	} else {
		gpio_set_pin_level(LED_WG_PIN, LED_INACTIVE);
	}
	//}
}

#ifdef __cplusplus
}
#endif
