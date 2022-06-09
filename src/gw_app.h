/**
 * \file
 *
 * \brief SAM SPI configuration
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


#ifndef GATEWAY_H_INCLUDED
#define GATEWAY_H_INCLUDED

#define GATEWAY_NAME	"MCHP-GateWay-01"

#define GATEWAY_WHITELIST_ASCII_ADDR_LEN	12

/* Gateway application node scan interval */
#define GATEWAY_NODE_SCAN_INTERVAL			4000


/* Gateway application node maximum primary services */
#define GATEWAY_NODE_PRIMARY_SERVICES_MAX		8


/* Gateway application node maximum primary services */
#define GATEWAY_NODE_ATTRIBUTES_MAX		25


#define MAIN_GATEWAY_CLIENT_NAME_MAX		32

#define GATEWAY_DISCOVERY_PORT				5001

#define GATEWAY_WHITELIST_PORT				5005

#define GATEWAY_TEST_PORT					6666

#define GATEWAY_DISCOVERY_BUFFER_LEN		100

/* Gateway application frame header size */
#define GATEWAY_DATA_FRAME_HDR_SIZE		0x03 	/* 2 bytes MFG ID and 1 byte application code*/

/* Manufacturer identifier in the beacon data	*/
#define GATEWAY_NODE_BEACON_MFG_ID		0x00CD	/* Application code in the beacon frame */

/*	The 1 byte value specifies the application code */
#define GATEWAY_NODE_BEACON_APP_CODE	0xAA

/* The node data max size */
#define GATEWAY_NODE_NAME_SIZE_MAX		0x0C

/* The node data max size */
#define GATEWAY_NODE_DATA_SIZE_MAX		0x08

/* The length of the BLE ADV packet */
#define GATEWAY_BLE_ADV_DATA_LEN		0x1E



#define GATEWAY_NODE_FRAME_HEADER		((GATEWAY_NODE_BEACON_APP_CODE << 24) | (GATEWAY_NODE_BEACON_MFG_ID << 8) | (GATEWAY_NODE_BEACON_APP_CODE) | 0xFF)

/*	The access point name for WINC3400 to connect */
#define GATEWAY_HOME_AP_NAME	"wsn"

/*	The access point passphrase  */
#define GATEWAY_HOME_AP_PASSWORD	"brucenegley"

/*	The access point security method OPEN/WPA_PSK */
#define GATEWAY_HOME_AP_SECURITY	M2M_WIFI_SEC_WPA_PSK


/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 2048 /*subscribing*/


/* Max size of the UUID of the characteristic	*/
#define MAIN_GATEWAY_UUID_ASCII_MAX 	4 + 2	/* Only SHORT UUID is stored */

/* BLE-WIFI Gateway MQTT topic. */
#define MAIN_GATEWAY_TOPIC "mchp/gw/"

/* Limitation of mqtt topic. */
#define MAIN_GATEWAY_TOPIC_MAX_SIZE	(32 + (AT_BLE_ADDR_LEN * 2))

#define CONF_BLE_PIN  {1, 2, 3, 4, 5, 6}

/* gateway configuration data */

/*
 * A MQTT broker server which was connected.
 * test.mosquitto.org is public MQTT broker.
 */
//static const char main_mqtt_broker[] = "192.168.93.121"; /*It's a local LAN MQTT server */
//static const char main_mqtt_broker[] = "192.168.1.7"; /*It's a local LAN MQTT server */

//static const char main_mqtt_broker[] = "test.mosquitto.org";
//static const char main_mqtt_broker[] = "iot.eclipse.org";
static const char main_mqtt_broker[] = "broker.hivemq.com";



extern struct sw_timer_module swt_module_inst;

#define DBG_FUNCTION_PRINT		M2M_INFO("\n\r%s", __FUNCTION__)

typedef enum 
{
	GW_APP_IDLE,
	GW_APP_INIT_DONE,
	GW_APP_WIFI_CONNECTED,
	GW_APP_MQTT_CONNECTED,
	GW_APP_BLE_CONNECTING,
	GW_APP_SCANING_STARTED,
	GW_APP_FORCE_SCANING_STARTED
}GATEWAY_APP_STATUS_t;


/**
 * \brief  Gateway demo BLE advertisement data format 
 *
 *  The format of data present in the BLE advertisment packtes from each node
 *  The user is free to choose application specific format by modifying these structures
 * */
typedef struct
{
	uint8_t				name_len;
	uint8_t				node_name[GATEWAY_NODE_NAME_SIZE_MAX];
	uint8_t				value_len;
	uint8_t				node_value[GATEWAY_NODE_DATA_SIZE_MAX];
}gateway_node_data_t;


/**
 * \brief   Gateway demo BLE node data structure, each node data is stored with these details
 *
 *	The each node data is stored locally in this format. The static array of @ref GATEWAY_NODE_DATA_SIZE_MAX nodes  
 *	is used to store each node data. 
 */
typedef struct
{
	gateway_node_data_t node_data;
	at_ble_addr_t		node_addr;
	uint16_t			node_alive;	
}gateway_node_value_t;



typedef enum
{
	GW_ATT_NODE_FREE = 0,
	GW_ATT_NODE_CONNECTING,
	GW_ATT_NODE_CONNECTED, 
	GW_ATT_PRIM_SERVICE_DISC,
	GW_ATT_CHAR_SERVICE_DISC,
	GW_ATT_DISC_SERVICE_DISC,
	GW_ATT_COMPLETED_DISC
}gw_att_discovery_states_t;



typedef struct 
{
	at_ble_handle_t start_handle;
	at_ble_handle_t end_handle;
	at_ble_uuid_t 	service_uuid;
}primary_service_data_t;

typedef struct 
{
	uint8_t 				total_services;
	uint8_t 				discovery_state;
	at_ble_handle_t 		conn_handle;
	at_ble_addr_t 			bd_addr;
	primary_service_data_t	primary_servies_list[GATEWAY_NODE_PRIMARY_SERVICES_MAX];
}client_primary_services_t;


typedef struct
{
	uint8_t			ascii_uuid[MAIN_GATEWAY_UUID_ASCII_MAX];
	at_ble_handle_t value_handle;
	uint8_t			properties;
}gw_client_attributes_t;


typedef struct
{
	gw_att_discovery_states_t		client_state;
	at_ble_addr_t					node_addr;
	at_ble_handle_t					client_conn_handle;
	uint8_t							gatt_atrib_index;
	uint8_t							mqtt_topic_len;
	uint8_t 						connParamUpdateFlag;
	uint8_t 						client_mqtt_topic[MAIN_GATEWAY_TOPIC_MAX_SIZE + MAIN_GATEWAY_UUID_ASCII_MAX];
	gw_client_attributes_t			gatt_attribs[GATEWAY_NODE_ATTRIBUTES_MAX];
	
}gw_client_details_t;

typedef struct 
{
	uint8_t 			total_connected_clients;
	gw_client_details_t client_details[GATEWAY_SUPPORTED_NODES_MAX];
}gw_client_database_t;

extern int  gateway_init(void);
extern void gateway_event_task(void);
extern void gateway_wifi_callback(uint8_t msg_type, void *pvMsg);
/**/
#endif