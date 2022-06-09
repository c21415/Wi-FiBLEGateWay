#include <asf.h>
#include <string.h>
#include <asf.h>
#include "bsp/include/nm_bsp_samd21_app.h"
#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_periph.h"
#include "main21.h"
#include "m2m_ble.h"
#include "at_ble_api.h"
#include "wifi_prov.h"
#include "utils/ble_utils.h"

#include "gateway_service.h"

#include "gw_app.h"
#include "gw_nvm.h"

GW_AP_Conf_Data_t g_gw_ap_cfg_data;

GW_WL_Conf_Data_t g_gw_wl_cfg_data;


void gateway_config_write(uint16_t nvm_row_num, uint8_t *config_data, uint8_t config_len)
{	
	uint16_t index;
	uint32_t configDstAddr = (nvm_row_num * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE);
	uint8_t nvmBuffer[NVMCTRL_PAGE_SIZE];
	enum status_code error_code;
	do 
	{			
		memset(nvmBuffer, 0xFF, NVMCTRL_PAGE_SIZE);
		for(index = 0; index < (config_len>NVMCTRL_PAGE_SIZE?NVMCTRL_PAGE_SIZE:config_len); index++)
		{
			nvmBuffer[index] = *config_data++;
		}
		
		error_code = STATUS_BUSY;
		do
		{	
			error_code = nvm_erase_row(configDstAddr);
		}while(error_code == STATUS_BUSY);
		
		error_code = STATUS_BUSY;
		do
		{
			error_code = nvm_write_buffer((configDstAddr), nvmBuffer, NVMCTRL_PAGE_SIZE);
		}while(error_code == STATUS_BUSY);
	
		config_len -= index;
		configDstAddr = configDstAddr + index;
	} while(config_len);
}

void gateway_config_read(uint16_t nvm_row_num, uint8_t *config_data, uint8_t config_len)
{
	uint16_t index;
	uint32_t configSrcAddr = (nvm_row_num * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE);
	uint8_t nvmBuffer[NVMCTRL_PAGE_SIZE];
	
	do
	{
		enum status_code error_code = STATUS_BUSY;
		do
		{
			error_code = nvm_read_buffer((configSrcAddr), nvmBuffer, NVMCTRL_PAGE_SIZE);
		}while(error_code == STATUS_BUSY);
		
		for(index = 0; index < (config_len>NVMCTRL_PAGE_SIZE?NVMCTRL_PAGE_SIZE:config_len); index++)
		{
			*config_data++ = nvmBuffer[index];
		}
		config_len -= index;
		configSrcAddr = configSrcAddr + index;
	} while(config_len);
	
}


void gateway_config_reset(uint16_t nvm_row_num)
{
	enum status_code error_code = STATUS_BUSY;
	do
	{	
		error_code = nvm_erase_row(nvm_row_num * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE);
	}while(error_code == STATUS_BUSY);

}

void configure_nvm(void)
{
	struct nvm_config config_nvm;

	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);

}


