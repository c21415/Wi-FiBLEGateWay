
/**
 * \file
 *
 * \brief Transparent Service declarations
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


#ifndef __GATEWAY_SERVICE_H__
#define __GATEWAY_SERVICE_H__

#include "at_ble_api.h"
#include "ble_manager.h"

/**
@defgroup sdk Atmel BLE SDK

@{
*/

/* The maximum nodes supported */
#define GATEWAY_SUPPORTED_NODES_MAX		8

#define GW_SCAN_INTERVAL					(0x40)              //Scan interval 30ms in term of 625us
#define GW_SCAN_WINDOW						(0x25)              //Scan window 30ms values in term of 625ms
#define GW_SCAN_TIMEOUT						(0x0000)          //Timeout  Scan time-out, 0x0000 disables time-out
#define GW_SCAN_TYPE						(AT_BLE_SCAN_ACTIVE)

#define GW_BLE_AUTHENTICATION_LEVEL			(AT_BLE_MODE1_L2_AUTH_PAIR_ENC)

#define GW_BLE_IO_CAPABALITIES				(AT_BLE_IO_CAP_KB_DISPLAY)

extern uint8_t scan_response_count;
extern at_ble_scan_info_t scan_info[MAX_SCAN_DEVICE];


at_ble_status_t gateway_scan_data_handler(at_ble_scan_info_t *scan_info, uint8_t scan_count);

void gateway_primary_service_found_handler(at_ble_primary_service_found_t *primary_services);

void gateway_characteristic_found_handler(at_ble_characteristic_found_t * characteristics_found);

void gateway_ble_notification_handler(at_ble_notification_received_t * ble_notification);

void gateway_ble_indication_handler(at_ble_indication_received_t * ble_indication);

void gateway_discovery_complete_handler(at_ble_discovery_complete_t *discovery_complete);

at_ble_status_t gateway_scan_info_handler(at_ble_scan_info_t *scan_param);

void gateway_pair_key_request_handler (at_ble_pair_key_request_t *pair_key);

void gateway_slave_security_handler(at_ble_slave_sec_request_t* slave_sec_req);

 /** @brief Function handlers for gateway ble connected */
#if defined GENERIC_GATEWAY_SERVICE
#define	BLE_SCAN_DATA_HANDLER						gateway_scan_data_handler
#define BLE_SCAN_INFO_HANDLER						gateway_scan_info_handler

#define BLE_PRIMARY_SERVICE_FOUND_HANDLER			gateway_primary_service_found_handler
#define BLE_DISCOVERY_COMPLETE_HANDLER				gateway_discovery_complete_handler
#define BLE_CHARACTERISTIC_FOUND_HANDLER			gateway_characteristic_found_handler

#define BLE_NOTIFICATION_RECEIVED_HANDLER			gateway_ble_notification_handler
#define BLE_INDICATION_RECEIVED_HANDLER				gateway_ble_indication_handler

#define BLE_SLAVE_SEC_REQUEST						gateway_slave_security_handler
#define BLE_PAIR_KEY_REQUEST						gateway_pair_key_request_handler

#endif


 /** @}*/

#endif /* __GATEWAY_SERVICE_H__ */
