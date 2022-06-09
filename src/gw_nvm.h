#include "gw_app.h"
#include "wifi_prov.h"

/* NVM ROW number to store the AP details */
#define GW_AP_CONFIG_NVM_ROW	0x100

/* NVM ROW number to store the WL data */
#define GW_WL_CONFIG_NVM_ROW	0x10A


typedef enum
{
	GW_DEV_MODE_AP 	= 0xFFFFFFFF,
	GW_DEV_MODE_STA = 0x00000000

}GW_Dev_Mode_t;

typedef struct
{
	GW_Dev_Mode_t 	dev_mode;
	credentials 	gw_ap_details;
}GW_AP_Conf_Data_t;

typedef struct 
{
	uint8_t 		tot_wl_nodes;
	at_ble_addr_t	whitelist_nodes[GATEWAY_SUPPORTED_NODES_MAX];

}GW_WL_Conf_Data_t;


extern GW_AP_Conf_Data_t g_gw_ap_cfg_data;
extern GW_WL_Conf_Data_t g_gw_wl_cfg_data;

/* Function Definitions */

extern void gateway_config_read(uint16_t nvm_row_num, uint8_t *config_data, uint8_t config_len);
extern void gateway_config_write(uint16_t nvm_row_num, uint8_t *config_data, uint8_t config_len);
extern void gateway_config_reset(uint16_t nvm_row_num);
extern void configure_nvm(void);


