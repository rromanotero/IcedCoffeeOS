/**
 *
 * \file
 *
 * \brief MAIN file.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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

/** \mainpage
 * \section intro Introduction
 * This reference design demonstrates Atmel SAMD21 MCU and WINC1500 Wi-Fi module
 * usage in a typical IoT application - Smart Plug.<br>
 * The hardware is designed to be like a real plug with Wi-Fi and touch function.
 * The plug can be controlled via smart phone App remotely. By including Atmel
 * metering chip ATM90E26, it also provides output energy measurement feature.
 *
 * \section files Main Files
 * - main_samd21.c : Initialization and main loop.
 *
 * \section usage Usage
 * -# Install plug in a power socket.
 * -# Open phone App and scan new plug.Follow the steps in phone App to add plug.
 * -# After plug is successfully added, it can be viewed and controlled via phone
 * App.
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

//#include <string.h>
#include "atmel_start.h"
//#include "status_codes.h"
//#include "clock_feature.h"
//#include "driver/include/m2m_wifi.h"
//#include "driver/source/nmasic.h"
//#include "winc1500_jd.h"
//#include "node_plug_jd.h"
//#include "led.h"
//#include "button.h"
//#include "m90e26_jd.h"
#include "adc_sensor.h"
//#include "config/touch_config_samd.h"
//#include "touch_api_SAMD.h"
//#include "gclk.h"
//#include "otau.h"
//#include "sw_timer.h"
//#include "eeprom.h"
//#include "stdio_start.h"


/**
 * \brief Main application function.
 *
 * \return program return value.
 */
int main(void)
{
	/* Initialize the board. */
	system_init();

	//MAIN_INFO(STRING_HEADER);

	/* Initialize EEPROM service. */
	//configure_eeprom();

	/* Initialize ADC module */
	//configure_adc();

	/* Initialize timer for providing timing info to QTouch library. */
	//timer_init_config();

	/* get config data saved in nvm */
	//read_nvm_data();
	//read_nvm_appinfo();

	/* Initialize QTouch library and configure touch sensors. */
	//touch_sensors_init();
	/* register button handler. */
	//register_button_handler(button_handler);
	/* Initialize m90e26 sensor. */
	//configure_m90e26();
	/* Initialize sw timer service. */
	//configure_sw_timer();
	/* Initialize http client service. */
	//otau_http_init();
	/* Initialize node plug */
	//node_plug_init();
	/* Initialize ecc service. */
	//eccContexInit();

	/* check if smart config mode enabled */
	//smart_config_mode_check();

	while (1) {
		/* smart config */
		//smart_config_task();
		/* wifi handling */
		//winc1500_task();
		/* local task */
		//local_task();
		/* otau task */
		//otau_task();
		delay_ms(2000);
		gpio_toggle_pin_level(LED0);
	}

	return 0;
}
