/**
 * \file
 *
 * \brief Energy Management cluster.
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

#ifndef ENERGY_MANAGE_H_INCLUDED
#define ENERGY_MANAGE_H_INCLUDED

#include "atmel_start.h"
#include "conf_clusterlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \name Energy management clusters ID definition.
 * @{
 */
#define ENERGY_MEASURE_CID 0x0600
#define PLUGSOCKET_TYPE_CID 0x0601
/*@}*/

/** \name Energy measurement cluster length definition. */
/* \{ */
#define ENERGY_C_ATTR1_LEN 1 /**< Length of energy measure cluster attribute1. */
#define ENERGY_C_ATTR2_LEN 1 /**< Length of energy measure cluster attribute2. */
#define ENERGY_C_ATTR3_LEN 2 /**< Length of energy measure cluster attribute3. */
#define ENERGY_C_ATTR4_LEN 2 /**< Length of energy measure cluster attribute4. */
#define ENERGY_C_ATTR5_LEN 2 /**< Length of energy measure cluster attribute5. */
#define ENERGY_CLUSTER_LEN                                                                                             \
	(ENERGY_C_ATTR1_LEN + ENERGY_C_ATTR2_LEN + ENERGY_C_ATTR3_LEN + ENERGY_C_ATTR4_LEN                                 \
	 + ENERGY_C_ATTR5_LEN) /**< Length of energy measure cluster. */
#define ENERGY_C_ATTR1_OFFSET 0
#define ENERGY_C_ATTR2_OFFSET (ENERGY_C_ATTR1_LEN)
#define ENERGY_C_ATTR3_OFFSET (ENERGY_C_ATTR1_LEN + ENERGY_C_ATTR2_LEN)
#define ENERGY_C_ATTR4_OFFSET (ENERGY_C_ATTR1_LEN + ENERGY_C_ATTR2_LEN + ENERGY_C_ATTR3_LEN)
#define ENERGY_C_ATTR5_OFFSET (ENERGY_C_ATTR1_LEN + ENERGY_C_ATTR2_LEN + ENERGY_C_ATTR3_LEN + ENERGY_C_ATTR4_LEN)
/*@}*/

/** \name Plug socket standard cluster length definition. */
/* \{ */
#define PLUG_TYPE_C_ATTR1_LEN 1                     /**< Length of plug socket type cluster attribute1. */
#define PLUG_TYPE_CLUSTER_LEN PLUG_TYPE_C_ATTR1_LEN /**< Length of plug socket type cluster. */
#define PLUG_TYPE_C_ATTR1_OFFSET 0
/*@}*/

/** Plug socket standard cluster */
typedef enum plug_standard {
	STANDARD_NA,
	US_STANDARD,
	EU_STANDARD,
	CN_STANDARD,
} plug_standard_t;

/** Plug output type definition */
typedef enum { NOT_SPECIFIED, AC_OUTPUT, DC_OUTPUT } output_type_t;

/** Energy measurement cluster */
typedef struct cluster_energy_measure {
	output_type_t output_type;
	uint8_t       output_voltage;
	uint16_t      output_current;
	uint16_t      output_power;
	uint16_t      active_energy;
} cluster_energy_measure_t;

/** Plug socket type cluster */
typedef struct cluster_plug_type {
	plug_standard_t type;
} cluster_plug_type_t;

#ifdef __cplusplus
}
#endif

#endif /* ENERGY_MANAGE_H_INCLUDED */
