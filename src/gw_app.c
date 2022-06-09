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
#include "socket/include/socket.h"
#include "platform.h"
#include "at_ble_api.h"
#include "ble_manager.h"
#include "utils/ble_utils.h"

#include "iot/sw_timer.h"
#include "iot/mqtt/mqtt.h"

#include "gateway_service.h"

#include "gw_app.h"
#include "gw_nvm.h"

extern at_ble_status_t gateway_node_scan(void);

SOCKET g_gw_disc_sock_id = SOCK_ERR_INVALID;
SOCKET g_gw_wl_sock_id = SOCK_ERR_INVALID;
SOCKET g_gw_wl_client_sock_id = SOCK_ERR_INVALID;
SOCKET g_gw_test_sock_id = SOCK_ERR_INVALID;


extern volatile uint8 gu8WiFiConnectionState;

extern uint8_t scan_response_count;

uint8_t g_bcRecvBuf[GATEWAY_DISCOVERY_BUFFER_LEN];

bool g_scan_triggered = false;

uint8_t g_scan_list_count;

at_ble_scan_info_t *g_scan_list = NULL;


client_primary_services_t	g_client_primary_servies;

gw_client_database_t	g_gw_node_database;

char g_mqtt_topic[MAIN_GATEWAY_TOPIC_MAX_SIZE];

int g_scanTimerId = -1;
int g_bleTimerId = -1;

uint8_t g_gw_whiteListAddr[GATEWAY_WHITELIST_ASCII_ADDR_LEN];

uint32 gu32Jiffies1ms=0;
/** Gateway application status variable. */
uint8 gu8GwAppStatus = GW_APP_IDLE;

/** Gateway Node count	*/
static uint32_t gu32NodeAliveFlag;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of gateway. */
char mqtt_user[32] = "GateWay";

/* Instance of MQTT service. */
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

static char	g_hexToAscii[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

typedef struct 
{
	uint32_t mqttMsgLen;
	uint8_t mqttMsg[128];
}MQTT_RX_DATA_t;
MQTT_RX_DATA_t g_mqttRxData;
uint8_t g_sendHugePublish = false;
uint8_t g_tcpTestData[1024];
uint8_t g_jasonTestData[1024];

uint32_t g_scanTick;
uint8_t g_reconnectMqtt = false;


/** Wi-Fi status variable. */
static uint8 gu8IsWiFiConnected = M2M_WIFI_DISCONNECTED;

/** Wi-Fi scan variables. */
extern uint8 gu8ScanIndex;

extern uint8 ble_wifi_scan_mode;

void gateway_set_hugeDataPublishFlag(void)
{
	
}

void gateway_set_connUpdateFlag(at_ble_handle_t connHdl)
{
	g_gw_node_database.client_details[connHdl].connParamUpdateFlag = true;
}

uint8_t get_node_table_index(at_ble_addr_t *node_addr)
{
	uint8_t count;
	for(count = 0; count < GATEWAY_SUPPORTED_NODES_MAX; count++)
	{
		if(memcmp((uint8_t *)&g_gw_node_database.client_details[count].node_addr, 
					node_addr, sizeof(at_ble_addr_t)) == 0)
		{
			break;
		}
	}
	return count;
}


uint8_t get_free_node(at_ble_addr_t *node_addr)
{
	uint8_t count;
	if((count = get_node_table_index(node_addr)) == GATEWAY_SUPPORTED_NODES_MAX)
	{
		for(count = 0; count < GATEWAY_SUPPORTED_NODES_MAX; count++)
		{
			if(g_gw_node_database.client_details[count].client_state == GW_ATT_NODE_FREE)
			{
				memcpy((uint8_t *)&g_gw_node_database.client_details[count].node_addr, 
					node_addr, sizeof(at_ble_addr_t));
				
				DBG_LOG("\r\nThe BD Address = %02X:%02X:%02X:%02X:%02X:%02X", 
					g_gw_node_database.client_details[count].node_addr.addr[5],
					g_gw_node_database.client_details[count].node_addr.addr[4],
					g_gw_node_database.client_details[count].node_addr.addr[3],
					g_gw_node_database.client_details[count].node_addr.addr[2],
					g_gw_node_database.client_details[count].node_addr.addr[1],
					g_gw_node_database.client_details[count].node_addr.addr[0]);			
				break;
			}
		}
	}
	return count;
}



uint8_t hex_to_ascii(uint8_t hexValue)
{
	return g_hexToAscii[hexValue];
}

uint8_t ascii_to_hex(uint8_t asciiValue)
{
	uint8_t hexValue;
	hexValue = ((asciiValue<='9') ? (asciiValue-'0') : ((asciiValue&0x0F)+9));
	return hexValue;
}

uint8_t str_to_bdaddr(uint8_t *str, uint8_t *bdAddr)
{
	int8_t idx;
	for(idx = 0; idx < AT_BLE_ADDR_LEN; idx++)
	{
		*bdAddr = ascii_to_hex(*(str-(idx*2)-1));
		*bdAddr = (*bdAddr << 4);
		*bdAddr++ |= ascii_to_hex(*(str-(idx*2)));		
	}
	return 0;
}

uint8_t get_atrib_handl_uuid(uint8_t devIdx, uint8_t *uuid, at_ble_handle_t* attribHndl)
{
	uint8_t idx;

	for(idx = 0; idx < GATEWAY_NODE_ATTRIBUTES_MAX; idx++)
	{
		if(memcmp(g_gw_node_database.client_details[devIdx].gatt_attribs[idx].ascii_uuid, uuid, 4) == 0)
		{
			DBG_LOG("The Value handle 0x%04X", g_gw_node_database.client_details[devIdx].gatt_attribs[idx].value_handle);
			*attribHndl = (g_gw_node_database.client_details[devIdx].gatt_attribs[idx].value_handle);
			return idx;
		}
	}
	DBG_LOG("The Value handle not found!");
	return GATEWAY_NODE_ATTRIBUTES_MAX;
}


uint32_t gateway_update_ble_connection_params(at_ble_connection_params_t *connParams)
{
	uint8_t idx = GATEWAY_SUPPORTED_NODES_MAX;
	uint32_t retVal = 0;
	
	while(idx > 0)
	{
		idx--;
		if(g_gw_node_database.client_details[idx].client_state == GW_ATT_NODE_CONNECTED)
		{
			DBG_LOG("%s:i=%d", __FUNCTION__, idx);
			if(at_ble_connection_param_update(g_gw_node_database.client_details[idx].client_conn_handle, connParams) != AT_BLE_SUCCESS)
				retVal |= (1 << idx);
		}
	}
	DBG_LOG("%s:retVal = 0x%X", __FUNCTION__, retVal);
	return retVal;
}

at_ble_status_t gateway_trigger_ble_scan(void)
{

	at_ble_status_t status = AT_BLE_FAILURE;
	uint8_t cnt;	
		
	at_ble_scan_stop();
	if(g_gw_node_database.total_connected_clients < g_gw_wl_cfg_data.tot_wl_nodes)
	{
		for(cnt=0; cnt<g_gw_wl_cfg_data.tot_wl_nodes; cnt++)
		{
			if(at_ble_whitelist_remove(&g_gw_wl_cfg_data.whitelist_nodes[cnt]) == AT_BLE_SUCCESS)
			{
				if(at_ble_whitelist_add(&g_gw_wl_cfg_data.whitelist_nodes[cnt]) == AT_BLE_SUCCESS)
					DBG_LOG("Successfully added %d to WL", cnt);
			}
		}
		status = gateway_node_scan();
		if(status == AT_BLE_SUCCESS)
			gu8GwAppStatus = GW_APP_SCANING_STARTED;
		
		DBG_LOG("gateway_node_scan() ret value %d\n", status);
	}
	return status;
}


void traeger_node_read(void)
{
	static uint8_t nodeIdx = 0;
	
	if(g_gw_wl_cfg_data.tot_wl_nodes <= g_gw_node_database.total_connected_clients)
		g_sendHugePublish = true;
	else
		return;
	
	DBG_LOG("Query node %d\n", nodeIdx);
	if(at_ble_characteristic_read(g_gw_node_database.client_details[nodeIdx].client_conn_handle, 35, 0, 1) == AT_BLE_SUCCESS)
	{
		DBG_LOG_DEV("Success:Requested battery value for node %d\r\n", nodeIdx);
	}
	else
	{
		DBG_LOG("Fail:Requested battery value for node %d\r\n", nodeIdx);
	}
	nodeIdx+=1;
	if(nodeIdx >= g_gw_node_database.total_connected_clients)
		nodeIdx = 0;
	
}

void gw_scan_timer_callback(struct sw_timer_module *const module, int timer_id, void *context, int period)
{
	static uint8_t connFailCnt = 0;	
	static uint8_t scanFailCnt = 0;	
	
	g_scanTick++;

	if((g_scanTick % 200) == 0)
	{
		g_reconnectMqtt = true; 
	}
	DBG_LOG_DEV("System State %d, remaining WL %d\n", gu8GwAppStatus, (g_gw_wl_cfg_data.tot_wl_nodes - g_gw_node_database.total_connected_clients));
	switch(gu8GwAppStatus)
	{
		
		case GW_APP_MQTT_CONNECTED:
		{						
			traeger_node_read();			
			gateway_trigger_ble_scan();						
		}
		break;
#if 1		
		case GW_APP_BLE_CONNECTING:
		{			
			if(connFailCnt > 20)
			{
				uint8_t nodeIdx = get_node_table_index(&g_scan_list[g_scan_list_count-1].dev_addr);
				printf("Node idx = %d state = %d\r\n", nodeIdx, g_gw_node_database.client_details[nodeIdx].client_state);
				if(nodeIdx && (g_client_primary_servies.discovery_state != GW_ATT_COMPLETED_DISC))
				{														
					at_ble_disconnect(g_gw_node_database.client_details[nodeIdx].client_conn_handle, AT_BLE_TERMINATED_BY_USER);
					g_client_primary_servies.discovery_state = GW_ATT_NODE_FREE;
					g_gw_node_database.client_details[nodeIdx].client_state = GW_ATT_NODE_FREE;						
#ifndef TEST
				gu8GwAppStatus = GW_APP_MQTT_CONNECTED;
#else
				gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
#endif	
				}												
				connFailCnt = 0;
			}
			connFailCnt++;			
		}
		break;
#endif		
		case GW_APP_SCANING_STARTED:
		{
			if(scanFailCnt > 10)
			{
				scanFailCnt = 0;
				printf("Forceful scan stop!");
				at_ble_scan_stop();
				gu8GwAppStatus = GW_APP_MQTT_CONNECTED;			
			}
			scanFailCnt++;
		}
		break;

	}
}


int gateway_scan_timer_start(uint32_t scan_period)
{
	int timerId;
	timerId = sw_timer_register_callback(&swt_module_inst, gw_scan_timer_callback, NULL, scan_period);
	if(timerId != -1)
	{
		sw_timer_enable_callback(&swt_module_inst, timerId, 0);
		DBG_LOG("Gateway BLE Scanning Timer task started");
	}
	return timerId;
}

void gateway_scan_timer_stop(void)
{
	sw_timer_unregister_callback(&swt_module_inst, g_scanTimerId);
	
}

at_ble_status_t gateway_scan_data_handler(at_ble_scan_info_t *scan_info, uint8_t scan_count)
{

	DBG_LOG("\n\rScan Complete %d, Wait %d seconds", scan_count, GATEWAY_NODE_SCAN_INTERVAL/1000);
	g_scan_list = scan_info;
	g_scan_list_count = scan_count;
	if(gu8GwAppStatus == GW_APP_FORCE_SCANING_STARTED)
	{
		at_ble_connection_params_t connParams = {GAP_CONN_INTERVAL_MIN,
			GAP_CONN_INTERVAL_MAX, GAP_CONN_SLAVE_LATENCY, GAP_CE_LEN_MIN, GAP_CE_LEN_MAX,
		GAP_SUPERVISION_TIMEOUT};
		//gateway_update_ble_connection_params(&connParams);
	}
	at_ble_scan_stop();
	#ifndef TEST
	gu8GwAppStatus = GW_APP_MQTT_CONNECTED;
	#else
	gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
	#endif
	return AT_BLE_SUCCESS;
}

/*************************Gateway Networking Functions***************************/
SOCKET gateway_discovery_server_init( uint16 udp_port )
{
	SOCKET serversocketid = 0;
	struct sockaddr_in addr;
	if (g_gw_disc_sock_id  != SOCK_ERR_INVALID)
	{
		//g_gw_disc_sock_id  = SOCK_ERR_INVALID;
		//close(SOCK_ERR_INVALID);
		return g_gw_disc_sock_id;
	}	
	
	serversocketid = socket( AF_INET, SOCK_DGRAM, 0 );
	if(serversocketid < 0)
	{
		DBG_LOG("UDPServer socket create error");
		return serversocketid;
	}
	
	memset(&addr, 0x0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(udp_port);
	addr.sin_addr.s_addr = nmi_inet_addr("0.0.0.0");
	if( bind( serversocketid, (struct sockaddr *)&addr, sizeof(addr)) != 0 )
	{
		DBG_LOG("UDPServer socket bind error");
		close(serversocketid);
		return SOCK_ERR_INVALID;
	}		

	DBG_LOG("UDP Server socket ID %d on port %d\n", serversocketid, udp_port);
	return serversocketid;
}

SOCKET gateway_test_server_init( uint16 tcp_port )
{
	SOCKET serversocketid = 0;
	struct sockaddr_in addr;

	if(g_gw_test_sock_id != SOCK_ERR_INVALID)
	{
		//close(g_gw_test_sock_id);
		//g_gw_test_sock_id = SOCK_ERR_INVALID;
		return g_gw_test_sock_id;
		
	}
	
	serversocketid = socket( AF_INET, SOCK_STREAM, 0 );
	if(serversocketid < 0)
	{
		DBG_LOG("TCPServer socket create error");
		return serversocketid;
	}
	
	memset(&addr, 0x0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(tcp_port);
	addr.sin_addr.s_addr = nmi_inet_addr("0.0.0.0");
	if( bind( serversocketid, (struct sockaddr *)&addr, sizeof(addr)) != 0 )
	{
		DBG_LOG("TCPServer socket bind error");
		close(serversocketid);
		return SOCK_ERR_INVALID;
	}

	DBG_LOG("TCP Server socket ID %d on port %d\n", serversocketid, tcp_port);
	return serversocketid;
}

SOCKET gateway_whitelist_server_init( uint16 tcp_port )
{
	SOCKET serversocketid = 0;
	struct sockaddr_in addr;

	if(g_gw_wl_sock_id != SOCK_ERR_INVALID)
	{
		//close(g_gw_wl_sock_id);
		//g_gw_wl_sock_id = SOCK_ERR_INVALID;
		return g_gw_wl_sock_id;
		
	}
	if(g_gw_wl_client_sock_id != SOCK_ERR_INVALID)
	{
		close(g_gw_wl_client_sock_id);
		g_gw_wl_client_sock_id = SOCK_ERR_INVALID;
	}
	
	serversocketid = socket( AF_INET, SOCK_STREAM, 0 );
	if(serversocketid < 0)
	{
		DBG_LOG("TCPServer socket create error");
		return serversocketid;
	}
	
	memset(&addr, 0x0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(tcp_port);
	addr.sin_addr.s_addr = nmi_inet_addr("0.0.0.0");
	if( bind( serversocketid, (struct sockaddr *)&addr, sizeof(addr)) != 0 )
	{
		DBG_LOG("TCPServer socket bind error");
		close(serversocketid);
		return SOCK_ERR_INVALID;
	}		

	DBG_LOG("TCP Server socket ID %d on port %d\n", serversocketid, tcp_port);
	return serversocketid;
}


void exit_gateway_application(void)
{
	DBG_FUNCTION_PRINT;
	mqtt_disconnect(&mqtt_inst, true);
	gateway_scan_timer_stop();
	memset(&g_gw_node_database.client_details, 0, sizeof(g_gw_node_database.client_details));
	gu32NodeAliveFlag = 0;
	gu8GwAppStatus = GW_APP_IDLE;
}
/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
void gateway_wifi_callback(uint8 msg_type, void *pvMsg)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) 
	{
		case M2M_WIFI_RESP_GET_SYS_TIME:
		{
			tstrSystemTime *sysTime = (tstrSystemTime *)pvMsg;
			DBG_LOG("System Date %02d/%02d/%04d", sysTime->u8Day, sysTime->u8Month, sysTime->u16Year);
		}
		break;
		
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			msg_wifi_state = (tstrM2mWifiStateChanged *)pvMsg;
			if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
				/* If Wi-Fi is connected. */
				DBG_LOG("Wi-Fi connected\r\n");
				m2m_wifi_request_dhcp_client();
			} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
				/* If Wi-Fi is disconnected. */
#ifndef TEST			
				m2m_wifi_connect((char *)GATEWAY_HOME_AP_NAME, sizeof(GATEWAY_HOME_AP_NAME),
						GATEWAY_HOME_AP_SECURITY, (char *)GATEWAY_HOME_AP_PASSWORD, M2M_WIFI_CH_ALL);
				/* Disconnect from MQTT broker. */
				/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
				if(gu8GwAppStatus > GW_APP_INIT_DONE)
				{
					DBG_LOG("Wi-Fi disconnected\r\n");
					//exit_gateway_application();
				}
#endif			
			}
		}
		break;
		
		case M2M_WIFI_REQ_DHCP_CONF:
		{
			msg_ip_addr = (uint8 *)pvMsg;
			DBG_LOG("Wi-Fi IP is %u.%u.%u.%u\r\n",
					msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
			/* Try to connect to MQTT broker when Wi-Fi was connected. */
			//g_gw_disc_sock_id = gateway_discovery_init(GATEWAY_DISCOVERY_PORT);		
			gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
			g_gw_disc_sock_id= gateway_discovery_server_init(GATEWAY_DISCOVERY_PORT);
			g_gw_wl_sock_id = gateway_whitelist_server_init(GATEWAY_WHITELIST_PORT);
			g_gw_test_sock_id = gateway_test_server_init(GATEWAY_TEST_PORT);			
#ifndef TEST		
			mqtt_connect(&mqtt_inst, main_mqtt_broker);
#else
			g_scanTimerId = gateway_scan_timer_start(GATEWAY_NODE_SCAN_INTERVAL);
#endif
		}
		break;
		
		case M2M_WIFI_RESP_SCAN_DONE:
		{
			tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;			
			if (gu8IsWiFiConnected == M2M_WIFI_DISCONNECTED) {
				gu8ScanIndex = 0;
				if (pstrInfo->u8NumofCh >= 1) {
					m2m_wifi_req_scan_result(gu8ScanIndex);
					gu8ScanIndex++;
				}
			}
		}
		break;
		
		case M2M_WIFI_RESP_SCAN_RESULT:
		{
			uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();

			if (gu8WiFiConnectionState != M2M_WIFI_CONNECTED)
			{
				tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult*) pvMsg;

				ble_prov_scan_result(pstrScanResult, u8NumFoundAPs-gu8ScanIndex);
				if(gu8ScanIndex < u8NumFoundAPs)
				{
					m2m_wifi_req_scan_result(gu8ScanIndex);
					gu8ScanIndex++;
				}
			}
		}
		break;
		
		case M2M_WIFI_RESP_BLE_API_RECV:
		{
			tstrM2mBleApiMsg *rx = (tstrM2mBleApiMsg *)pvMsg;
			platform_receive(rx->data, rx->u16Len);
		}
		break;

		default:
			break;
	}
}

/*****************************************************************************/
uint8_t acceptFlag = false;
uint8_t closeFlag = false;
void socket_callback(SOCKET sock, uint8 u8Msg, void * pvMsg)
{
	sint16 wlRetValue = SOCK_ERR_NO_ERROR;
	DBG_LOG_DEV("%s:sock = %d msg id = %d", __FUNCTION__, sock, u8Msg);	
#ifndef TEST	
	if(sock == mqtt_inst.sock)
	{
		mqtt_socket_event_handler(sock, u8Msg, pvMsg);
		return;
	}
#endif	
	switch(u8Msg)
	{
		case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg *sockBindMsg = (tstrSocketBindMsg *)pvMsg;
			DBG_LOG_DEV("Binding Status %d", sockBindMsg->status);
			if(sock == g_gw_wl_sock_id || sock == g_gw_test_sock_id)
			{
				if((wlRetValue = listen(sock, 0)) == SOCK_ERR_NO_ERROR)
					DBG_LOG_DEV("\n\rConnecting to mqtt broker\n\r");("Whitelist TCP socket Listen successful");
			}
			else if(sock == g_gw_disc_sock_id)
			{						
				if(recvfrom(sock, g_bcRecvBuf, GATEWAY_DISCOVERY_BUFFER_LEN, 0) == SOCK_ERR_NO_ERROR )
					DBG_LOG_DEV("Waiting for UDP Discovery data");
			}
		}
		break;
		case SOCKET_MSG_LISTEN:
		{
			tstrSocketListenMsg *sockListenMsg = (tstrSocketListenMsg *)pvMsg;
			if(sockListenMsg && sockListenMsg->status == SOCK_ERR_NO_ERROR)
			{
				DBG_LOG_DEV("Listening on whitelist socket Status %d", sockListenMsg->status);
				if((wlRetValue = accept(sock, NULL, NULL)) == SOCK_ERR_NO_ERROR)
				{
					DBG_LOG("Waiting for the connection on whitelist socket");
				}
			}

		}
		break;
		case SOCKET_MSG_ACCEPT:
		{
			tstrSocketAcceptMsg *sockAcceptMsg = (tstrSocketAcceptMsg *)pvMsg;
			DBG_LOG("Connection is accepted from %d.%d.%d.%d on [%d]",  (sockAcceptMsg->strAddr.sin_addr.s_addr & 0xFF), ((sockAcceptMsg->strAddr.sin_addr.s_addr >> 8) & 0xFF),
							((sockAcceptMsg->strAddr.sin_addr.s_addr >> 16) & 0xFF), ((sockAcceptMsg->strAddr.sin_addr.s_addr >> 24) & 0xFF), sockAcceptMsg->sock);
			if(sock == g_gw_wl_sock_id)
			{
				g_gw_wl_client_sock_id = sockAcceptMsg->sock;
				if((wlRetValue = recv(sockAcceptMsg->sock, g_gw_whiteListAddr, GATEWAY_WHITELIST_ASCII_ADDR_LEN, 0)) == SOCK_ERR_NO_ERROR)
					DBG_LOG("Waiting for data on the whitelist TCP port");
			}
			else if(sock == g_gw_test_sock_id)
			{
				acceptFlag = true;
				if(recv(sockAcceptMsg->sock, g_tcpTestData, 1024, 0) != SOCK_ERR_NO_ERROR)
				{
					DBG_LOG("TCP receive failure!\r\n");
				}	
			}
				
		}
		break;
		case SOCKET_MSG_SEND:
		{
			uint32_t recvSize;
			DBG_LOG_DEV("Successfully sent %d bytes\r\n", *((int16_t *)pvMsg));
			if((recvSize = recv(sock, g_tcpTestData, sizeof(g_tcpTestData), 0)) != SOCK_ERR_NO_ERROR)
			{
				DBG_LOG("Receive Error!\r\n", recvSize);
				if((wlRetValue = accept(g_gw_test_sock_id, NULL, NULL)) == SOCK_ERR_NO_ERROR)
					DBG_LOG("Waiting for the connection on whitelist/test socket");
			}
			
		}
		break;
		case SOCKET_MSG_RECV:				
		{
			tstrSocketRecvMsg	*strRecvMsg = (tstrSocketRecvMsg	*)pvMsg;
			//uint8_t idx;
			DBG_LOG("TCP Data is received on socket %d: ", sock);	
			//for(idx = 0; idx < strRecvMsg->s16BufferSize; idx++)
				//printf("0x%02X ", strRecvMsg->pu8Buffer[idx]);
			if(memcmp(strRecvMsg->pu8Buffer, "WL_CFG_DONE", strlen("WL_CFG_DONE")) == 0)
			{				
				wlRetValue = SOCK_ERR_INVALID;	
				gateway_trigger_ble_scan();
				close(g_gw_wl_client_sock_id);
			}
			else if(sock == g_gw_wl_client_sock_id)
			{		
#ifndef NEWLIB
		at_ble_gap_deviceinfo_t gapDevInfo = {AT_BLE_GAP_CENTRAL_MST, };		
		at_ble_set_gap_deviceinfo(&gapDevInfo);
#endif

				if(g_gw_wl_cfg_data.tot_wl_nodes < GATEWAY_SUPPORTED_NODES_MAX)
				{
					str_to_bdaddr(strRecvMsg->pu8Buffer+GATEWAY_WHITELIST_ASCII_ADDR_LEN-1, 
							g_gw_wl_cfg_data.whitelist_nodes[g_gw_wl_cfg_data.tot_wl_nodes].addr);
					if((g_gw_wl_cfg_data.tot_wl_nodes != 0))
					{
						uint8_t idx;	
						/* Search through existing WL and add only if its not already added */
						for(idx = 0; idx < g_gw_wl_cfg_data.tot_wl_nodes; idx++)
						{
							if(memcmp(g_gw_wl_cfg_data.whitelist_nodes[g_gw_wl_cfg_data.tot_wl_nodes].addr, 
								g_gw_wl_cfg_data.whitelist_nodes[idx].addr, AT_BLE_ADDR_LEN) == 0)
							{
								DBG_LOG("Already in WL configuration");
								if((wlRetValue = recv(g_gw_wl_client_sock_id, g_gw_whiteListAddr, GATEWAY_WHITELIST_ASCII_ADDR_LEN, 0)) == SOCK_ERR_NO_ERROR)
									DBG_LOG_DEV("Waiting for data on the whitelist TCP port");
								return;
									
							}
						}
						
					}
					g_gw_wl_cfg_data.whitelist_nodes[g_gw_wl_cfg_data.tot_wl_nodes].type = AT_BLE_ADDRESS_PUBLIC;	
					g_gw_wl_cfg_data.tot_wl_nodes++;
									
					DBG_LOG("Adding Node to WL configuration");
					gateway_config_write(GW_WL_CONFIG_NVM_ROW, 
						(uint8_t *)&g_gw_wl_cfg_data, sizeof(g_gw_wl_cfg_data));
				}											
				if((wlRetValue = recv(g_gw_wl_client_sock_id, g_gw_whiteListAddr, GATEWAY_WHITELIST_ASCII_ADDR_LEN, 0)) == SOCK_ERR_NO_ERROR)
					DBG_LOG_DEV("Waiting for data on the whitelist TCP port");
			}
			else
			{
				if(strRecvMsg->s16BufferSize <= 0)
				{
					acceptFlag = false;
					closeFlag = true;						
					DBG_LOG("Socket Closed! %d", strRecvMsg->s16BufferSize);									
				}
				else
				{
					DBG_LOG("Received TCP data size = %d\r\n", strRecvMsg->s16BufferSize);	
					if((send(sock, g_tcpTestData, 1024, 0)) != SOCK_ERR_NO_ERROR)
					{
						DBG_LOG("TCP send Error!");
					}									
				}
			}
		}
		break;
		case SOCKET_MSG_RECVFROM:
		{
			tstrSocketRecvMsg *sockRecvMsg = (tstrSocketRecvMsg *)pvMsg;
			if(sockRecvMsg->s16BufferSize > 0)
			{
				DBG_LOG("Received discovery message from the peer %d.%d.%d.%d",  (sockRecvMsg->strRemoteAddr.sin_addr.s_addr & 0xFF), ((sockRecvMsg->strRemoteAddr.sin_addr.s_addr >> 8) & 0xFF),
				((sockRecvMsg->strRemoteAddr.sin_addr.s_addr >> 16) & 0xFF), ((sockRecvMsg->strRemoteAddr.sin_addr.s_addr >> 24) & 0xFF));
				sendto(sock, GATEWAY_NAME, strlen(GATEWAY_NAME), 0, &sockRecvMsg->strRemoteAddr, sizeof(struct sockaddr));
				recvfrom(sock, g_bcRecvBuf, GATEWAY_DISCOVERY_BUFFER_LEN, 0);
			}
			
		}
		break;
	}
	if(wlRetValue != SOCK_ERR_NO_ERROR)
	{
		g_gw_wl_sock_id = gateway_whitelist_server_init(GATEWAY_WHITELIST_PORT);
	}		
}


void dns_callback(uint8* pu8DomainName, uint32 u32ServerIP)
{
	mqtt_socket_resolve_handler(pu8DomainName, u32ServerIP);
}


/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if(gu8GwAppStatus == GW_APP_WIFI_CONNECTED)
		{
			if (data->sock_connected.result >= 0) {
				DBG_LOG("\n\rConnecting to mqtt broker\n\r");
				mqtt_connect_broker(module_inst, 1, NULL, NULL, mqtt_user, NULL, NULL, 0, 0, 0);
			} else {
				DBG_LOG("Connect fail to server(%s)! retry it automatically. %d\r\n", main_mqtt_broker, data->sock_connected.result);			
				mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
			}
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
	{
		uint8_t tmpSubTop[MAIN_GATEWAY_TOPIC_MAX_SIZE];	
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			gu8GwAppStatus = GW_APP_MQTT_CONNECTED;
			sprintf(tmpSubTop, "%srx/#", g_mqtt_topic);
			mqtt_subscribe(module_inst, tmpSubTop, 0);	
			DBG_LOG("Subscribing to MQTT topic %s", tmpSubTop);				
			if(g_scanTimerId == -1)				
				g_scanTimerId = gateway_scan_timer_start(GATEWAY_NODE_SCAN_INTERVAL);
			DBG_LOG("\n\rPreparation of the gateway has been completed.\r\n");
			DBG_LOG("\n\rConfigure the BLE nodes whitelist using the mobile APP.\r\n");
		} else {
			/* Cannot connect for some reason. */
			gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
			DBG_LOG("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}
			}
	break;

	case MQTT_CALLBACK_RECV_PUBLISH:
	{
		struct mqtt_data_recv_publish *mqttSubData = (struct mqtt_data_recv_publish *)data;

	#ifndef TRAEGER
		at_ble_status_t status;
		uint8_t attribIdx, devIdx, topicLen = strlen(g_mqtt_topic)+3; //2bytes extra for rx
		at_ble_addr_t devBdAddr;
		at_ble_handle_t attribHndl;

		str_to_bdaddr(&mqttSubData->topic[topicLen+(AT_BLE_ADDR_LEN*2)-1], (uint8_t *)&devBdAddr);
		devIdx = get_node_table_index(&devBdAddr);
		if(devIdx != GATEWAY_SUPPORTED_NODES_MAX)
		{
			attribIdx = get_atrib_handl_uuid(devIdx, &mqttSubData->topic[topicLen+(AT_BLE_ADDR_LEN*2)+1], &attribHndl);
			if(attribIdx != GATEWAY_NODE_ATTRIBUTES_MAX)
			{
				bool signedWrite = false; 
				bool writeResp = true;
				switch(g_gw_node_database.client_details[devIdx].gatt_attribs[attribIdx].properties 
					& (AT_BLE_CHAR_WRITE_WITHOUT_RESPONSE | AT_BLE_CHAR_WRITE | AT_BLE_CHAR_SIGNED_WRITE
					|AT_BLE_CHAR_RELIABLE_WRITE))
				{
					case AT_BLE_CHAR_WRITE_WITHOUT_RESPONSE:
						writeResp = false;
						break;
					case AT_BLE_CHAR_SIGNED_WRITE:
						signedWrite = true;
						break;
				}
				status = at_ble_characteristic_write(g_gw_node_database.client_details[devIdx].client_conn_handle, attribHndl, 0,  mqttSubData->msg_size,
					mqttSubData->msg, signedWrite, writeResp);
				DBG_LOG("Sent data to BLE node %d, write status %d", devIdx, status);
			}
			else
			{
				DBG_LOG("Unable to find the attrib handle");
			}
		}
		else
		{
			DBG_LOG("Unable to find the BLE node");
		}
#endif
	
		g_mqttRxData.mqttMsgLen = mqttSubData->msg_size;
		memcpy(g_mqttRxData.mqttMsg, mqttSubData->msg, g_mqttRxData.mqttMsgLen);
		printf("C2B data: %.*s\r\n", mqttSubData->msg_size, (char*)mqttSubData->msg);
	}
	break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		DBG_LOG("MQTT disconnected\r\n");		
		//gateway_scan_timer_stop();
		//g_scanTimerId = -1;
		close(mqtt_inst.sock);
		gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
		gu32NodeAliveFlag = 0;
		mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		break;
	}
}

/**
 * \brief Configure Timer module.
 */
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

/**
 * \brief Configure MQTT service.
 */
static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;
	mqtt_conf.send_buffer_size = (MAIN_GATEWAY_TOPIC_MAX_SIZE + sizeof(gateway_node_value_t)*GATEWAY_SUPPORTED_NODES_MAX );
	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		DBG_LOG("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		DBG_LOG("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}


void gateway_node_disconnected_cb(at_ble_handle_t ble_conn_handle)
{
	DBG_LOG("gateway_node_disconnected_cb %d", ble_conn_handle);
	g_gw_node_database.total_connected_clients--;
	memset(&g_gw_node_database.client_details[ble_conn_handle], 0, sizeof(gw_client_details_t));
	memset(&g_client_primary_servies, 0, sizeof(g_client_primary_servies));
	if(g_scan_list_count)
		g_scan_list_count--;
#ifndef TEST	
	gu8GwAppStatus = GW_APP_MQTT_CONNECTED;
#else
		gu8GwAppStatus = GW_APP_WIFI_CONNECTED; 
#endif
}
void gateway_node_connected_cb(at_ble_handle_t ble_conn_handle)
{

	at_ble_status_t status;
	uint8_t index, idx = 0;
	if(g_client_primary_servies.discovery_state != GW_ATT_NODE_FREE)	
	{
		DBG_LOG("ERROR:Primary service resource is busy");
	}
	else
	{	
		at_ble_exchange_mtu(ble_conn_handle);		
		g_client_primary_servies.conn_handle = ble_conn_handle;	/* Assign the connection handle */
		g_gw_node_database.client_details[ble_conn_handle].client_conn_handle = ble_conn_handle;		
		g_client_primary_servies.total_services = 0;
		g_gw_node_database.client_details[ble_conn_handle].client_state = GW_ATT_NODE_CONNECTED;
		g_client_primary_servies.discovery_state = GW_ATT_PRIM_SERVICE_DISC;

		g_gw_node_database.total_connected_clients++;

		/* Prepare the MQTT client topic */

		memcpy(g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic, g_mqtt_topic, strlen(g_mqtt_topic));
		
		for(index = AT_BLE_ADDR_LEN; index > 0; index--)		
		{
			g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic[idx*2 + strlen(g_mqtt_topic)] 		= hex_to_ascii((g_client_primary_servies.bd_addr.addr[index-1] >> 4)& 0x0F);
			g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic[idx*2 + strlen(g_mqtt_topic) +1 ] 	= hex_to_ascii(g_client_primary_servies.bd_addr.addr[index-1]& 0x0F);
			idx++;
		}
		g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic[strlen(g_mqtt_topic) + (AT_BLE_ADDR_LEN*2)] = '/';
		g_gw_node_database.client_details[ble_conn_handle].mqtt_topic_len = strlen(g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic);
		DBG_LOG("Device %d Topic %s", ble_conn_handle, g_gw_node_database.client_details[ble_conn_handle].client_mqtt_topic);
		status = at_ble_primary_service_discover_all(ble_conn_handle, GATT_DISCOVERY_STARTING_HANDLE, GATT_DISCOVERY_ENDING_HANDLE);
		if(status != AT_BLE_SUCCESS)
		{
			DBG_LOG("ERROR:Failed to start service discovery, disconnecting the node. status = %d", status);
			at_ble_disconnect(ble_conn_handle, AT_BLE_TERMINATED_BY_USER);
		}
	}
	return AT_BLE_SUCCESS;
}


void gateway_primary_service_found_handler(at_ble_primary_service_found_t *primary_services)
{
	
	if(g_client_primary_servies.discovery_state == GW_ATT_PRIM_SERVICE_DISC)
	{
		if(g_client_primary_servies.total_services < GATEWAY_NODE_PRIMARY_SERVICES_MAX)
		{	
			memcpy(&g_client_primary_servies.primary_servies_list[g_client_primary_servies.total_services], &(primary_services->start_handle), sizeof(primary_service_data_t));
			g_client_primary_servies.total_services++; //Increment the total primary services
		}
	}
}


void gateway_characteristic_found_handler(at_ble_characteristic_found_t * char_found)
{

	uint8_t data[2] = {0}, idx = 0;
	at_ble_handle_t conn_handle = char_found->conn_handle;
	DBG_LOG("Char Found hndl = 0x%02X, prop = 0x%02X", char_found->char_handle, char_found->properties);
	if(char_found->properties & (AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE | AT_BLE_CHAR_WRITE | 
		AT_BLE_CHAR_WRITE_WITHOUT_RESPONSE | AT_BLE_CHAR_SIGNED_WRITE))
	{
		uint8_t freeIndex = g_gw_node_database.client_details[conn_handle].gatt_atrib_index;		
		if(char_found->properties & (AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE ))
		{
		
			data[0] = ((char_found->properties & (AT_BLE_CHAR_NOTIFY | AT_BLE_CHAR_INDICATE)) >> 4);
			DBG_LOG("Writing to attrib handle %d", char_found->value_handle+1);
			if(at_ble_characteristic_write(conn_handle, char_found->value_handle+1,
				0, 2, data, false, true) == AT_BLE_FAILURE) 
			{
				DBG_LOG("\r\nFailed to send characteristic Write Request");
				return;
			}
		}
		if(freeIndex < GATEWAY_NODE_ATTRIBUTES_MAX)
		{
			volatile uint8_t index, uuid_len, uuid_offset;
			g_gw_node_database.client_details[char_found->conn_handle].gatt_attribs[freeIndex].value_handle = char_found->value_handle;
			g_gw_node_database.client_details[char_found->conn_handle].gatt_attribs[freeIndex].properties = char_found->properties;
			g_gw_node_database.client_details[conn_handle].gatt_atrib_index++;
			switch(char_found->char_uuid.type)
			{
				case AT_BLE_UUID_16:
						uuid_offset = 2;
						break;
				case AT_BLE_UUID_32:
						uuid_offset = 4;
						break;
				case AT_BLE_UUID_128:
						uuid_offset = 14;
						break;
					
			}
			for(index = 2; index > 0; index--)
			{
				g_gw_node_database.client_details[conn_handle].gatt_attribs[freeIndex].ascii_uuid[idx *2] 	= hex_to_ascii((char_found->char_uuid.uuid[uuid_offset-1] >> 4) & 0x0F);
				g_gw_node_database.client_details[conn_handle].gatt_attribs[freeIndex].ascii_uuid[idx *2+1] 	= hex_to_ascii(char_found->char_uuid.uuid[uuid_offset-1] & 0x0F);
				uuid_offset--;				
				idx++;
			}	
			g_gw_node_database.client_details[conn_handle].gatt_attribs[freeIndex].ascii_uuid[idx*2] 	= '/';
			g_gw_node_database.client_details[conn_handle].gatt_attribs[freeIndex].ascii_uuid[idx*2+1] 	= 0;
		}
		else
		{
			DBG_LOG("\r\nFailed, the maximum supported attributes are reached");
		}
	}
}
uint8_t json_str[64] = {'{', '"', 'I', 'D', '"', ':', '"', 'F', 'F', '"', ',', '"', 'V', 'a', 'l', '"', ':', '"', '0', 'x' };
	
void gateway_ble_notification_handler(at_ble_notification_received_t * ble_notification)
{
	uint8_t index;
	int status;
	uint8_t idx=7;
	
	json_str[idx++] = hex_to_ascii((ble_notification->conn_handle >> 4) & 0x0F);
	json_str[idx++] = hex_to_ascii(ble_notification->conn_handle & 0x0F);

	idx  = 20;
	
	for(index = 0; index < ble_notification->char_len; index++)
	{
		json_str[idx++] = hex_to_ascii((ble_notification->char_value[index] >> 4) & 0x0F);
		json_str[idx++] = hex_to_ascii(ble_notification->char_value[index] & 0x0F);
	}
	json_str[idx++] = '"';
	json_str[idx++] = '}';
	if(gu8GwAppStatus > GW_APP_WIFI_CONNECTED)
	{
		status = mqtt_publish(&mqtt_inst, g_mqtt_topic,
		(const char *)json_str, idx, 0, 0);
		if(status != 0)
		{
			DBG_LOG("Failed to publish %d\r\n", status);
		}
	}
}

void gateway_ble_indication_handler(at_ble_indication_received_t * ble_indication)
{
	int status;
	uint8_t index;
	uint8_t idx=7;

	json_str[idx++] = hex_to_ascii((ble_indication->conn_handle >> 4) & 0x0F);
	json_str[idx++] = hex_to_ascii(ble_indication->conn_handle & 0x0F);

	idx  = 20;
	
	for(index = 0; index < ble_indication->char_len; index++)
	{
		json_str[idx++] = hex_to_ascii((ble_indication->char_value[index] >> 4) & 0x0F);
		json_str[idx++] = hex_to_ascii(ble_indication->char_value[index] & 0x0F);
	}		
	
	json_str[idx++] = '"';
	json_str[idx++] = '}';
	
	if(gu8GwAppStatus > GW_APP_WIFI_CONNECTED)
	{
		status = mqtt_publish(&mqtt_inst, g_mqtt_topic,
		(const char *)json_str, idx, 0, 0);
		if(status != 0)
		{
			DBG_LOG("Failed to publish %d\r\n", status);
		}
	}
}

void gateway_discovery_complete_handler(at_ble_discovery_complete_t *discovery_complete)
{
	at_ble_status_t status;
	DBG_LOG_DEV("gateway_discovery_complete_handler");
	if(g_client_primary_servies.total_services--)
	{
		if(discovery_complete->status == AT_BLE_DISCOVERY_SUCCESS)
		{
			g_client_primary_servies.discovery_state = GW_ATT_CHAR_SERVICE_DISC;
			status = at_ble_characteristic_discover_all(g_client_primary_servies.conn_handle, 
					g_client_primary_servies.primary_servies_list[g_client_primary_servies.total_services].start_handle, 
					g_client_primary_servies.primary_servies_list[g_client_primary_servies.total_services].end_handle);
			if(status != AT_BLE_SUCCESS)
			{
				DBG_LOG("Failed to start the descriptor discovery, disconnecting the node. status = %d", status);
				g_client_primary_servies.discovery_state = GW_ATT_COMPLETED_DISC;
				at_ble_disconnect(g_client_primary_servies.conn_handle, AT_BLE_TERMINATED_BY_USER);
			}
		}		
	}
	else
	{
		//sw_timer_enable_callback(&swt_module_inst, g_scanTimerId, GATEWAY_NODE_SCAN_INTERVAL);	
		/* Decrement the connection index*/	
		g_client_primary_servies.discovery_state = GW_ATT_COMPLETED_DISC;
		memset(&g_client_primary_servies, 0, sizeof(g_client_primary_servies));
		if(g_scan_list_count)
			g_scan_list_count--;
		DBG_LOG("Discovery Completed for node %d", g_scan_list_count);

#ifndef TEST	
		gu8GwAppStatus = GW_APP_MQTT_CONNECTED;
#else
			gu8GwAppStatus = GW_APP_WIFI_CONNECTED; 
#endif
	}
}

void gateway_event_task(void)
{
	uint8_t index;


#ifndef TEST	
	if(gu8GwAppStatus == GW_APP_MQTT_CONNECTED)
#else
	if(gu8GwAppStatus == GW_APP_WIFI_CONNECTED)
#endif
	{
		if(g_scan_list_count)
		{
			index = get_free_node(&g_scan_list[g_scan_list_count-1].dev_addr);			
					
			g_gw_node_database.client_details[index].client_state = GW_ATT_NODE_CONNECTING;
			memcpy(&g_client_primary_servies.bd_addr, &g_scan_list[g_scan_list_count-1].dev_addr, sizeof(at_ble_addr_t));
			at_ble_scan_stop();			
			if(gap_dev_connect(&g_scan_list[g_scan_list_count-1].dev_addr) == AT_BLE_SUCCESS)
			{
				DBG_LOG("Connecting to BLE Node %d", index);					
				gu8GwAppStatus = GW_APP_BLE_CONNECTING;
			}
			else
			{
				g_gw_node_database.client_details[index].client_state = GW_ATT_NODE_FREE;
				DBG_LOG("Connecting to BLE Node Failed!");
			}
			
			//else if(g_gw_node_database.client_details[index].client_state == GW_ATT_NODE_CONNECTED)
			//{
				//g_scan_list_count--;
			//}
		}
		else
		{
			static uint8_t nodeIdx = 0;
			if(nodeIdx < g_gw_node_database.total_connected_clients)
			{			
				if(g_gw_node_database.client_details[nodeIdx].connParamUpdateFlag)
				{
					at_ble_tx_power_get(g_gw_node_database.client_details[nodeIdx].client_conn_handle);
					g_gw_node_database.client_details[nodeIdx].connParamUpdateFlag = false;						
				}
				nodeIdx++;			
			}
		}
		
		if(g_mqttRxData.mqttMsgLen)
		{
			mqtt_publish(&mqtt_inst, g_mqtt_topic, g_mqttRxData.mqttMsg, g_mqttRxData.mqttMsgLen, 0, 0);
			g_mqttRxData.mqttMsgLen = 0;
		}
		
		if(g_sendHugePublish)
		{
			g_sendHugePublish = false;
			mqtt_publish(&mqtt_inst, g_mqtt_topic, g_jasonTestData, sizeof(g_jasonTestData), 0, 0);
		}
		if(g_reconnectMqtt)
		{
			g_reconnectMqtt = false;
			gu8GwAppStatus = GW_APP_WIFI_CONNECTED;
			mqtt_disconnect(&mqtt_inst, 1);
		}
	}
	

}


int gateway_init(void)
{
	uint32_t idx = 0;
	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the MQTT service. */
	configure_mqtt();
	   
	/*Initialize NVM */
	configure_nvm();
	
	while(idx < 1024)
	{
		g_jasonTestData[idx++] = 'T';
		g_jasonTestData[idx++] = 'e';
		g_jasonTestData[idx++] = 's';
		g_jasonTestData[idx++] = 't';
	}

	if(SW0_ACTIVE == port_pin_get_input_level(SW0_PIN))
	{
		gateway_config_reset(GW_AP_CONFIG_NVM_ROW);
		gateway_config_reset(GW_WL_CONFIG_NVM_ROW);
		memset(&g_gw_wl_cfg_data, 0xFF, sizeof(g_gw_wl_cfg_data));
		memset(&g_gw_ap_cfg_data, 0xFF, sizeof(g_gw_ap_cfg_data));
		g_gw_wl_cfg_data.tot_wl_nodes = 0;
		gateway_config_write(GW_WL_CONFIG_NVM_ROW, (uint8_t *)&g_gw_wl_cfg_data, sizeof(g_gw_wl_cfg_data));
		port_pin_set_output_level(LED0_PIN, false);
		delay_s(2);
		port_pin_set_output_level(LED0_PIN, true);
		DBG_LOG("Configuration is reset");
	}
	else
	{
		/*Read Config Data */
		gateway_config_read(GW_AP_CONFIG_NVM_ROW, (uint8_t *)&g_gw_ap_cfg_data, sizeof(g_gw_ap_cfg_data));
		gateway_config_read(GW_WL_CONFIG_NVM_ROW, (uint8_t *)&g_gw_wl_cfg_data, sizeof(g_gw_wl_cfg_data));
	}
#ifdef TRAEGER 

	g_gw_ap_cfg_data.dev_mode = GW_DEV_MODE_STA;
	memcpy(g_gw_ap_cfg_data.gw_ap_details.ssid, GATEWAY_HOME_AP_NAME, strlen(GATEWAY_HOME_AP_NAME));
	g_gw_ap_cfg_data.gw_ap_details.ssid_length = strlen(GATEWAY_HOME_AP_NAME);
	g_gw_ap_cfg_data.gw_ap_details.passphrase_length = strlen(GATEWAY_HOME_AP_PASSWORD);
	g_gw_ap_cfg_data.gw_ap_details.sec_type = GATEWAY_HOME_AP_SECURITY;
	memcpy(g_gw_ap_cfg_data.gw_ap_details.passphrase, GATEWAY_HOME_AP_PASSWORD, strlen(GATEWAY_HOME_AP_PASSWORD));
	g_gw_ap_cfg_data.gw_ap_details.passphrase[g_gw_ap_cfg_data.gw_ap_details.passphrase_length] = '\0';
	
	g_gw_wl_cfg_data.tot_wl_nodes = 6;

#ifdef TEST	
	g_gw_wl_cfg_data.whitelist_nodes[0].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[0] = 0xF3;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[1] = 0xF8;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[2] = 0xE2;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[5] = 0xF8;
	
	g_gw_wl_cfg_data.whitelist_nodes[1].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[0] = 0xA7;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[1] = 0xFA;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[2] = 0xE2;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[5] = 0xF8;
	
	g_gw_wl_cfg_data.whitelist_nodes[2].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[0] = 0x2E;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[1] = 0xFB;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[2] = 0xE2;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[5] = 0xF8;
	
	g_gw_wl_cfg_data.whitelist_nodes[3].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[0] = 0x44;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[1] = 0x72;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[2] = 0xE5;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[5] = 0xF8;
	
	g_gw_wl_cfg_data.whitelist_nodes[4].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[0] = 0xD1;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[1] = 0x72;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[2] = 0xE5;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[5] = 0xF8;
	
	g_gw_wl_cfg_data.whitelist_nodes[5].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[0] = 0xBD;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[1] = 0x72;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[2] = 0xE5;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[5].addr[5] = 0xF8;
#else
	g_gw_wl_cfg_data.tot_wl_nodes = 5;
	
	g_gw_wl_cfg_data.whitelist_nodes[0].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[0] = 0x61;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[1] = 0xEF;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[2] = 0x52;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[3] = 0x5E;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[4] = 0x1F;
	g_gw_wl_cfg_data.whitelist_nodes[0].addr[5] = 0xB8;


	g_gw_wl_cfg_data.whitelist_nodes[1].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[0] = 0xCE;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[1] = 0x66;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[2] = 0x36;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[3] = 0x5E;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[4] = 0x1F;
	g_gw_wl_cfg_data.whitelist_nodes[1].addr[5] = 0xB8;


	g_gw_wl_cfg_data.whitelist_nodes[2].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[0] = 0x90;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[1] = 0x65;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[2] = 0x36;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[3] = 0x5E;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[4] = 0x1F;
	g_gw_wl_cfg_data.whitelist_nodes[2].addr[5] = 0xB8;

	g_gw_wl_cfg_data.whitelist_nodes[3].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[0] = 0x3A;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[1] = 0xEF;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[2] = 0x52;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[3] = 0x5E;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[4] = 0x1F;
	g_gw_wl_cfg_data.whitelist_nodes[3].addr[5] = 0xB8;


	g_gw_wl_cfg_data.whitelist_nodes[4].type = AT_BLE_ADDRESS_PUBLIC;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[0] = 0xBD;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[1] = 0x72;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[2] = 0xE5;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[3] = 0x05;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[4] = 0xF0;
	g_gw_wl_cfg_data.whitelist_nodes[4].addr[5] = 0xF8;

#endif

#endif 
	if(g_gw_ap_cfg_data.dev_mode == GW_DEV_MODE_AP)
	{
		return GW_DEV_MODE_AP;
	}
	else
	{
		uint8_t idx;

		
		DBG_LOG("Total Number of WL nodes = %d", g_gw_wl_cfg_data.tot_wl_nodes);	
		for(idx=0; idx < g_gw_wl_cfg_data.tot_wl_nodes; idx++)
		{
			DBG_LOG("Dev %d BD Addr  = %02X:%02X:%02X:%02X:%02X:%02X", 
				idx,
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[5],
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[4],
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[3],
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[2],
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[1],
				g_gw_wl_cfg_data.whitelist_nodes[idx].addr[0]);
		}
		
	
		at_ble_set_dev_config(AT_BLE_GAP_CENTRAL_MST);

		//register_ble_paired_event_cb(gateway_node_connected_cb);
		register_ble_connected_event_cb(gateway_node_connected_cb);
		register_ble_disconnected_event_cb(gateway_node_disconnected_cb);

		/* Initialize socket interface. */
		socketInit();
		registerSocketCallback(socket_callback, dns_callback);

		/* Setup user name first */
		//printf("Enter the user name (Max %d characters)\r\n", MAIN_GATEWAY_USER_NAME_SIZE);
		//scanf("%64s", mqtt_user);
		DBG_LOG("User : %s\r\n", mqtt_user);
		sprintf(g_mqtt_topic, "%s%s/", MAIN_GATEWAY_TOPIC, mqtt_user);
	
		// Connect to router.
		DBG_LOG("Connecting to %s, please wait...", (char *)g_gw_ap_cfg_data.gw_ap_details.ssid);
		m2m_wifi_connect((char *)g_gw_ap_cfg_data.gw_ap_details.ssid, g_gw_ap_cfg_data.gw_ap_details.ssid_length, 
				g_gw_ap_cfg_data.gw_ap_details.sec_type, (char *)g_gw_ap_cfg_data.gw_ap_details.passphrase, M2M_WIFI_CH_ALL);

		gu8GwAppStatus = GW_APP_INIT_DONE;

		return GW_DEV_MODE_STA;
	}
}


