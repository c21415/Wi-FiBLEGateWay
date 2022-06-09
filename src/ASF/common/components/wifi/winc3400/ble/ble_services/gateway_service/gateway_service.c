/**
 * \file
 *
 * \brief Gateway between Wi-Fi and BLE nodes example.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

#include <asf.h>
#include <string.h>
#include "bsp/include/nm_bsp_samd21_app.h"
#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_periph.h"
#include "platform.h"
#include "at_ble_api.h"
#include "ble_manager.h"
#include "utils/ble_utils.h"

#include "gateway_service.h"

at_ble_LTK_t app_bond_info;
bool app_device_bond;
at_ble_auth_t auth_info;


/** @brief instructs device to start scanning */
at_ble_status_t gateway_node_scan(void)
{
	/* Device Scan discover started*/
	DBG_LOG("Scanning...Please wait...");
	/* make service discover counter to zero*/
	scan_response_count = 0;
	return(at_ble_scan_start(GW_SCAN_INTERVAL, GW_SCAN_WINDOW, 0, GW_SCAN_TYPE, AT_BLE_SCAN_GEN_DISCOVERY, true,true)) ;
}


/** @brief function handling scaned information */
at_ble_status_t gateway_scan_info_handler(at_ble_scan_info_t *scan_param)
{
	if(scan_param->type == AT_BLE_ADV_TYPE_SCAN_RESPONSE)
		return AT_BLE_SUCCESS;

	if(scan_response_count < MAX_SCAN_DEVICE)
	{
		// store the advertising report data into scan_info[]
		memcpy((uint8_t *)&scan_info[scan_response_count], scan_param, sizeof(at_ble_scan_info_t));
		DBG_LOG("Info:Device found address [%d]  0x%02X%02X%02X%02X%02X%02X ",
		scan_response_count,
		scan_param->dev_addr.addr[5],
		scan_param->dev_addr.addr[4],
		scan_param->dev_addr.addr[3],
		scan_param->dev_addr.addr[2],
		scan_param->dev_addr.addr[1],
		scan_param->dev_addr.addr[0]);
		scan_response_count++;
		return AT_BLE_SUCCESS;
	}
	else
	{
		DBG_LOG("Info:maximum of scan device reached");
		//Todo Stop Scanning
		return AT_BLE_FAILURE;
	}
}


void gateway_slave_security_handler(at_ble_slave_sec_request_t* slave_sec_req)
{	
	at_ble_pair_features_t features;
	uint8_t i = 0;
	
	if (app_device_bond)
	{
		app_device_bond = false;
	}
	
	if(!app_device_bond)
	{

		features.desired_auth =  GW_BLE_AUTHENTICATION_LEVEL; 
		features.bond = slave_sec_req->bond;
		features.mitm_protection = slave_sec_req->mitm_protection;
		/* Device capabilities is display only , key will be generated
		and displayed */
		features.io_capabilities = GW_BLE_IO_CAPABALITIES;

		features.oob_available = false;
			
		/* Distribution of LTK is required */
		features.initiator_keys =   AT_BLE_KEY_DIST_ENC;
		features.responder_keys =   AT_BLE_KEY_DIST_ENC;
		features.max_key_size = 16;
		features.min_key_size = 16;
		
		/* Generate LTK */
		for(i=0; i<8; i++)
		{
			app_bond_info.key[i] = rand()&0x0f;
			app_bond_info.nb[i] = rand()&0x0f;
		}
		
		for(i=8 ; i<16 ;i++)
		{
			app_bond_info.key[i] = rand()&0x0f;
		}
		
		app_bond_info.ediv = rand()&0xffff;
		app_bond_info.key_size = 16;
		/* Send pairing response */
		DBG_LOG_DEV("Sending pairing response");
		if(at_ble_authenticate(slave_sec_req->handle, &features, &app_bond_info, NULL) != AT_BLE_SUCCESS)
		{
			features.bond = false;
			features.mitm_protection = false;
			DBG_LOG(" != AT_BLE_SUCCESS ");
			at_ble_authenticate(slave_sec_req->handle, &features, NULL, NULL);
			
		}
	}
}


/** @brief function handles pair key request */
void gateway_pair_key_request_handler (at_ble_pair_key_request_t *pair_key)
{
	/* Passkey has fixed value in this example MSB */
	uint8_t passkey[6]={'1','2','3','4','5','6'};
	uint8_t idx = 0;
        uint8_t pin;
        
	at_ble_pair_key_request_t pair_key_request;
        
        memcpy((uint8_t *)&pair_key_request, pair_key, sizeof(at_ble_pair_key_request_t));
#if 0        
        if(pair_key_request.passkey_type == AT_BLE_PAIR_PASSKEY_ENTRY)
        {
          DBG_LOG("Enter the Passkey(6-Digit) in Terminal:");
            
          for(idx = 0; idx < 6; )
          {          
            pin = getchar();
            if((pin >= '0') && ( pin <= '9'))
            {
              passkey[idx++] = pin;
              DBG_LOG_CONT("%c", pin);
            }
          }
        }	
#endif	
	/* Display passkey */
	if(((pair_key_request.passkey_type == AT_BLE_PAIR_PASSKEY_DISPLAY) &&
	   (pair_key_request.type == AT_BLE_PAIR_PASSKEY)) || (pair_key_request.passkey_type == AT_BLE_PAIR_PASSKEY_ENTRY))
	{
          if(pair_key_request.passkey_type == AT_BLE_PAIR_PASSKEY_ENTRY)
          {
            DBG_LOG("Entered Pass-code:");
          }
          else
          {
            DBG_LOG("Please Enter the following Pass-code(on other Device):");
          }
          
          /* Convert passkey to ASCII format */
          for(idx=0; idx<AT_BLE_PASSKEY_LEN; idx++)
          {
                  DBG_LOG_CONT("%c",passkey[idx]);
          }		
          
          if(!(at_ble_pair_key_reply(pair_key_request.handle, pair_key_request.type, passkey)) == AT_BLE_SUCCESS)
          {
                  DBG_LOG("Pair-key reply failed");
          }
	}
	else 
	{
		if(pair_key_request.type == AT_BLE_PAIR_OOB)
		{
			DBG_LOG("OOB Feature Not supported");
		}
	}	
}





