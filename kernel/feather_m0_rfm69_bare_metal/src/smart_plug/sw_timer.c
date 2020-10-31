/**
 * \file
 *
 * \brief SW Timer component for the IoT(Internet of things) service.
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
#include "utils_assert.h"
#include "sw_timer.h"

/*use system tick as reference */

/** Tick count of timer. */
static uint32_t sw_timer_tick = 0;

/**
 * \brief SysTick handler used to measure precise delay.
 */
void SysTick_Handler(void)
{
	sw_timer_tick++;
}

/**
 * \brief TCC callback of SW timer.
 *
 * This function performs to the increasing the tick count.
 *
 * \param[in] module Instance of the TCC.
 */

void sw_timer_get_config_defaults(struct sw_timer_config *const config)
{
	ASSERT(config);

	config->accuracy             = 100;
	config->tcc_dev              = 0;
	config->tcc_callback_channel = 0;
}

void sw_timer_init(struct sw_timer_module *const module_inst, struct sw_timer_config *const config)
{
	ASSERT(module_inst);
	ASSERT(config);
	module_inst->accuracy = config->accuracy;
	/* Enable SysTick interrupt for non busy wait delay. */
	if (SysTick_Config(48000000UL / (1000 / config->accuracy))) {
		while (1)
			;
	}
}

int sw_timer_register_callback(struct sw_timer_module *const module_inst, sw_timer_callback_t callback, void *context,
                               uint32_t period)
{
	int                     index;
	struct sw_timer_handle *handler;

	ASSERT(module_inst);

	for (index = 0; index < CONF_SW_TIMER_COUNT; index++) {
		if (module_inst->handler[index].used == 0) {
			handler                  = &module_inst->handler[index];
			handler->callback        = callback;
			handler->callback_enable = 0;
			handler->context         = context;
			handler->period          = period / module_inst->accuracy;
			handler->used            = 1;
			return index;
		}
	}
	return -1;
}

void sw_timer_unregister_callback(struct sw_timer_module *const module_inst, int timer_id)
{
	struct sw_timer_handle *handler;

	ASSERT(module_inst);
	ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT));

	handler = &module_inst->handler[timer_id];

	handler->used = 0;
}

void sw_timer_enable_callback(struct sw_timer_module *const module_inst, int timer_id, uint32_t delay)
{
	struct sw_timer_handle *handler;

	ASSERT(module_inst);
	ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT));

	handler = &module_inst->handler[timer_id];

	handler->callback_enable = 1;
	handler->expire_time     = sw_timer_tick + (delay / module_inst->accuracy);
}

void sw_timer_disable_callback(struct sw_timer_module *const module_inst, int timer_id)
{
	struct sw_timer_handle *handler;

	ASSERT(module_inst);
	ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT));

	handler = &module_inst->handler[timer_id];

	handler->callback_enable = 0;
}

void sw_timer_task(struct sw_timer_module *const module_inst)
{
	int                     index;
	struct sw_timer_handle *handler;

	ASSERT(module_inst);

	for (index = 0; index < CONF_SW_TIMER_COUNT; index++) {
		if (module_inst->handler[index].used && module_inst->handler[index].callback_enable) {
			handler = &module_inst->handler[index];
			if ((int)(handler->expire_time - sw_timer_tick) < 0 && handler->busy == 0) {
				/* Enter critical section. */
				handler->busy = 1;
				/* Timer was expired. */
				if (handler->period > 0) {
					handler->expire_time = sw_timer_tick + handler->period;
				} else {
					/* One shot. */
					handler->callback_enable = 0;
				}
				/* Call callback function. */
				handler->callback(module_inst, index, handler->context, handler->period);
				/* Leave critical section. */
				handler->busy = 0;
			}
		}
	}
}
