/**
 * \file
 *
 * \brief Generic cluster.
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

#ifndef GENERIC_H_INCLUDED
#define GENERIC_H_INCLUDED

#include "atmel_start.h"
#include "conf_clusterlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \name Generic clusters ID definition
 * @{
 */
#define ONOFF_CID 0x0001
#define LEVEL_CTR_CID 0x0002
#define DEVICE_TEMP_CID 0x0003
#define TIME_CALENDAR_CID 0x0004
#define SCHEDULE_CID 0x0005
#define ALARM_CID 0x0006
#define NAME_CID 0x0007
#define CLOUD_INFO_CID 0x0008
#define NODE_CLOUD_INFO_CID 0x0009
#define HISTORIC_LOG_CID 0x000A
#define SYS_STATUS_CID 0x000B

#define MAC_ADDR_CID 0x00FD
#define PRODUCT_INFO_CID 0x00FE
/*@}*/

/** \name Onoff cluster length definition. */
/* \{ */
#define ONOFF_C_ATTR1_LEN 1                 /**< Length of onoff cluster attribute1. */
#define ONOFF_CLUSTER_LEN ONOFF_C_ATTR1_LEN /**< Length of onoff cluster. */
#define ONOFF_C_ATTR1_OFFSET 0
/*@}*/

/** \name Device temperature cluster length definition. */
/* \{ */
#define DEV_TEMP_C_ATTR1_LEN 2 /**< Length of device temp cluster attribute1. */
#define DEV_TEMP_C_ATTR2_LEN 2 /**< Length of device temp cluster attribute2. */
#define DEV_TEMP_C_ATTR3_LEN 2 /**< Length of device temp cluster attribute3. */
#define DEV_TEMP_CLUSTER_LEN                                                                                           \
	(DEV_TEMP_C_ATTR1_LEN + DEV_TEMP_C_ATTR2_LEN + DEV_TEMP_C_ATTR3_LEN) /**< Length of device temp cluster. */
#define DEV_TEMP_C_ATTR1_OFFSET 0
#define DEV_TEMP_C_ATTR2_OFFSET (DEV_TEMP_C_ATTR1_LEN)
#define DEV_TEMP_C_ATTR3_OFFSET (DEV_TEMP_C_ATTR1_LEN + DEV_TEMP_C_ATTR2_LEN)
/*@}*/

/** \name Time calendar cluster length definition. */
/* \{ */
#define TIME_CAL_C_ATTR1_LEN 2 /**< Length of time calendar cluster attribute1. */
#define TIME_CAL_C_ATTR2_LEN 1 /**< Length of time calendar cluster attribute2. */
#define TIME_CAL_C_ATTR3_LEN 1 /**< Length of time calendar cluster attribute3. */
#define TIME_CAL_C_ATTR4_LEN 4 /**< Length of time calendar cluster attribute4. */
#define TIME_CAL_C_ATTR5_LEN 4 /**< Length of time calendar cluster attribute5. */
#define TIME_CAL_CLUSTER_LEN                                                                                           \
	(TIME_CAL_C_ATTR1_LEN + TIME_CAL_C_ATTR2_LEN + TIME_CAL_C_ATTR3_LEN + TIME_CAL_C_ATTR4_LEN                         \
	 + TIME_CAL_C_ATTR5_LEN) /**< Length of device temp cluster. */
#define TIME_CAL_C_ATTR1_OFFSET 0
#define TIME_CAL_C_ATTR2_OFFSET (TIME_CAL_C_ATTR1_LEN)
#define TIME_CAL_C_ATTR3_OFFSET (TIME_CAL_C_ATTR1_LEN + TIME_CAL_C_ATTR2_LEN)
#define TIME_CAL_C_ATTR4_OFFSET (TIME_CAL_C_ATTR1_LEN + TIME_CAL_C_ATTR2_LEN + TIME_CAL_C_ATTR3_LEN)
#define TIME_CAL_C_ATTR5_OFFSET                                                                                        \
	(TIME_CAL_C_ATTR1_LEN + TIME_CAL_C_ATTR2_LEN + TIME_CAL_C_ATTR3_LEN + TIME_CAL_C_ATTR4_LEN)
/*@}*/

/** \name Schedule cluster length definition. */
/* \{ */
#define SCHEDULE_C_ATTR1_LEN 2 /**< Length of schedule cluster attribute1. */
#define SCHEDULE_C_ATTR2_LEN 1 /**< Length of schedule cluster attribute2. */
#define SCHEDULE_C_ATTR3_LEN 1 /**< Length of schedule cluster attribute3. */
#define SCHEDULE_C_ATTR4_LEN 1 /**< Length of schedule cluster attribute4. */
#define SCHEDULE_C_ATTR5_LEN 2 /**< Length of schedule cluster attribute5. */
#define SCHEDULE_C_ATTR6_LEN 1 /**< Length of schedule cluster attribute6. */
#define SCHEDULE_CLUSTER_LEN                                                                                           \
	(SCHEDULE_C_ATTR1_LEN + SCHEDULE_C_ATTR2_LEN + SCHEDULE_C_ATTR3_LEN + SCHEDULE_C_ATTR4_LEN + SCHEDULE_C_ATTR5_LEN  \
	 + SCHEDULE_C_ATTR6_LEN) /**< Length of schedule cluster. */
#define SCHEDULE_C_ATTR1_OFFSET 0
#define SCHEDULE_C_ATTR2_OFFSET (SCHEDULE_C_ATTR1_LEN)
#define SCHEDULE_C_ATTR3_OFFSET (SCHEDULE_C_ATTR1_LEN + SCHEDULE_C_ATTR2_LEN)
#define SCHEDULE_C_ATTR4_OFFSET (SCHEDULE_C_ATTR1_LEN + SCHEDULE_C_ATTR2_LEN + SCHEDULE_C_ATTR3_LEN)
#define SCHEDULE_C_ATTR5_OFFSET                                                                                        \
	(SCHEDULE_C_ATTR1_LEN + SCHEDULE_C_ATTR2_LEN + SCHEDULE_C_ATTR3_LEN + SCHEDULE_C_ATTR4_LEN)
#define SCHEDULE_C_ATTR6_OFFSET                                                                                        \
	(SCHEDULE_C_ATTR1_LEN + SCHEDULE_C_ATTR2_LEN + SCHEDULE_C_ATTR3_LEN + SCHEDULE_C_ATTR4_LEN + SCHEDULE_C_ATTR5_LEN)
/*@}*/

/** \name Historic log cluster length definition. */
/* \{ */
#define HIST_LOG_C_ATTR1_LEN 2
#define HIST_LOG_C_ATTR2_LEN 1
#define HIST_LOG_C_ATTR3_LEN 1
#define HIST_LOG_C_ATTR4_LEN 2
#define HIST_LOG_C_ATTR5_LEN (sizeof(rec_t) * REC_NUM) // NULL as can't be predefined
#define LOG_CLUSTER_LEN                                                                                                \
	(HIST_LOG_C_ATTR1_LEN + HIST_LOG_C_ATTR2_LEN + HIST_LOG_C_ATTR3_LEN + HIST_LOG_C_ATTR4_LEN + HIST_LOG_C_ATTR5_LEN)
#define HIST_LOG_C_ATTR1_OFFSET 0
#define HIST_LOG_C_ATTR2_OFFSET (HIST_LOG_C_ATTR1_LEN)
#define HIST_LOG_C_ATTR3_OFFSET (HIST_LOG_C_ATTR1_LEN + HIST_LOG_C_ATTR2_LEN)
#define HIST_LOG_C_ATTR4_OFFSET (HIST_LOG_C_ATTR1_LEN + HIST_LOG_C_ATTR2_LEN + HIST_LOG_C_ATTR3_LEN)
#define HIST_LOG_C_ATTR5_OFFSET                                                                                        \
	(HIST_LOG_C_ATTR1_LEN + HIST_LOG_C_ATTR2_LEN + HIST_LOG_C_ATTR3_LEN + HIST_LOG_C_ATTR4_LEN)
/*@}*/

/** \name System Status cluster length definition. */
/* \{ */
#define SYS_STATUS_C_ATTR1_LEN 2
#define SYS_STATUS_C_ATTR2_LEN (sizeof(status_t) * SYS_STATUS_NUM) // NULL as can't be predefined
#define SYS_STATUS_CLUSTER_LEN (SYS_STATUS_C_ATTR1_LEN + SYS_STATUS_C_ATTR2_LEN)
#define SYS_STATUS_C_ATTR1_OFFSET 0
#define SYS_STATUS_C_ATTR2_OFFSET (SYS_STATUS_C_ATTR1_LEN)
/*@}*/

/** \name MAC address cluster length definition. */
/* \{ */
#define MAC_ADDR_C_ATTR1_LEN 1 /**< Length of MAC address cluster attribute1. */
#define MAC_ADDR_C_ATTR2_LEN 8 /**< Length of MAC address cluster attribute1. */
#define MAC_ADDR_CLUSTER_LEN (MAC_ADDR_C_ATTR1_LEN + MAC_ADDR_C_ATTR2_LEN) /**< Length of MAC address cluster. */
#define MAC_ADDR_C_ATTR1_OFFSET 0
#define MAC_ADDR_C_ATTR2_OFFSET (MAC_ADDR_C_ATTR1_LEN)
/*@}*/

/** \name Product information cluster length definition. */
/* \{ */
#define PROD_INFO_C_ATTR1_LEN 2              /**< Length of product information cluster attribute1. */
#define PROD_INFO_C_ATTR2_LEN 2              /**< Length of product information cluster attribute2. */
#define PROD_INFO_C_ATTR3_LEN 2              /**< Length of product information cluster attribute3. */
#define PROD_INFO_C_ATTR4_LEN 1              /**< Length of product information cluster attribute4. */
#define PROD_INFO_C_ATTR5_LEN PRODUCT_SN_LEN /**< Length of product information cluster attribute5. */
#define PROD_INFO_CLUSTER_LEN                                                                                          \
	(PROD_INFO_C_ATTR1_LEN + PROD_INFO_C_ATTR2_LEN + PROD_INFO_C_ATTR3_LEN + PROD_INFO_C_ATTR4_LEN                     \
	 + PROD_INFO_C_ATTR5_LEN) /**< Length of product information cluster. */
#define PROD_INFO_C_ATTR1_OFFSET 0
#define PROD_INFO_C_ATTR2_OFFSET (PROD_INFO_C_ATTR1_LEN)
#define PROD_INFO_C_ATTR3_OFFSET (PROD_INFO_C_ATTR1_LEN + PROD_INFO_C_ATTR2_LEN)
#define PROD_INFO_C_ATTR4_OFFSET (PROD_INFO_C_ATTR1_LEN + PROD_INFO_C_ATTR2_LEN + PROD_INFO_C_ATTR3_LEN)
#define PROD_INFO_C_ATTR5_OFFSET                                                                                       \
	(PROD_INFO_C_ATTR1_LEN + PROD_INFO_C_ATTR2_LEN + PROD_INFO_C_ATTR3_LEN + PROD_INFO_C_ATTR4_LEN)
/*@}*/

// Day index
#define DAY_MON 0x01
#define DAY_TUE 0x02
#define DAY_WED 0x03
#define DAY_THU 0x04
#define DAY_FRI 0x05
#define DAY_SAT 0x06
#define DAY_SUN 0x07
// Connection type
#define CON_WIFI 0x01
#define CON_BLUETOOTH 0x02
#define CON_ZIGBEE 0x03
#define CON_ETHERNET 0x04
#define CON_ZWAVE 0x05
#define CON_PLC 0x06

/** OnOff cluster */
typedef struct cluster_on_off {
	bool on_off;
} cluster_on_off_t;

/** Level cluster */
typedef struct cluster_level_control {
	uint16_t level;
	uint16_t trans_time;
} cluster_level_control_t;

/** Device Temperature cluster */
typedef struct cluster_device_temp {
	int16_t temperature;
	int16_t threshold_l;
	int16_t threshold_h;
} cluster_device_temp_t;

/** Time Calendar cluster */
typedef struct cluster_time_calendar {
	uint16_t start_year;
	uint8_t  weekday;
	uint8_t  calendar_mask;
	uint32_t time_calendar;
	uint32_t running_time;
} cluster_time_calendar_t;

/** Schedule cluster */
typedef struct cluster_schedule {
	uint16_t target_cluster_id;
	uint8_t  target_cluster_index;
	uint8_t  target_attr_id;
	uint8_t  day_mask;
	union {
		uint16_t time16;
		struct {
			uint8_t min;
			uint8_t hour;
		} time;
	} schedule_time;
	union {
		bool onoff;
	} action;
} cluster_schedule_t;

/** Alarm cluster */
typedef struct cluster_alarm {
	uint8_t  code;
	uint8_t  status;
	uint16_t source_cluster;
	uint8_t  source_attr;
} cluster_alarm_t;

/**
 * \name Historic Log cluster
 * @{
 */
typedef uint32_t time_stamp_t;

typedef struct _rec {
	time_stamp_t stamp;
	union {
		bool     onoff;
		int16_t  temp;
		uint16_t energy;
	} rec;
} rec_t;

#define TIME_STAMP_LEN sizeof(time_stamp_t)

typedef struct cluster_log {
	uint16_t cid;    // cluster ID
	uint8_t  cindex; // cluster index
	uint8_t  aid;    // attribute ID
	uint16_t log_num;
	rec_t *  log_list;
} cluster_log_t;
/*@}*/

/**
 * \name System Status cluster
 * @{
 */
#define SYS_NO_ERROR 0x00
#define SYS_ONOFF_ERROR 0x01

typedef struct _status {
	uint8_t s_code;
	uint8_t priority;
} status_t;

#define STATUS_CODE_LEN sizeof(uint8_t)
#define PRIORITY_LEN sizeof(uint8_t)

typedef struct cluster_status {
	uint16_t  status_num;
	status_t *status_list;
} cluster_status_t;
/*@}*/

/** MAC Address cluster */
typedef struct cluster_mac_addr {
	uint8_t connection_type;
	union {
		uint64_t addr;
		uint8_t  array[MAC_ADDR_LENGTH];
	} mac_addr;
} cluster_mac_addr_t;

/** Product Information cluster */
typedef struct cluster_product_info {
	uint16_t product_id;
	uint16_t mftr_id;
	uint16_t fw_ver;
	uint8_t  sn_len;
	uint8_t  product_sn[PRODUCT_SN_LEN];
} cluster_product_info_t;

/** Node Name cluster */
typedef struct cluster_name {
	uint8_t name[NODE_NAME_LEN];
} cluster_name_t;

/** Cloud Information cluster */
typedef struct cluster_cloud_info {
	uint8_t  cloud_vendor;
	uint8_t  server_addr[CLOUD_SERVER_ADDR_LEN];
	uint16_t port;
	uint8_t  protocol;
} cluster_cloud_info_t;

/** Node Cloud Information cluster */
typedef struct cluster_node_cloud_info {
	uint8_t product_id[PRODUCT_ID_LEN];
	uint8_t feed_id[FEED_ID_LEN];
	uint8_t access_key[ACCESS_KEY_LEN];
} cluster_node_cloud_info_t;

#ifdef __cplusplus
}
#endif

#endif /* GENERIC_H_INCLUDED */
