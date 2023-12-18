#ifndef __USER_BLE_H__
#define __USER_BLE_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>

#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define CONFIG_SET_RAW_ADV_DATA
/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_CHAR_VAL_LEN_MAX 500
#define CHAR_DECLARATION_SIZE       	(sizeof(uint8_t))
#define CHAR_CONFIGURATION_SIZE       	(sizeof(uint16_t))

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

struct gatts_server_dev {
	const char * name;

    uint16_t gatts_if;			/* interface */
    uint16_t app_id;			/* esp_ble_gatts_app_register() use */	

    uint16_t conn_id;			/* record conn_id when connected */
#if 0				
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
#endif

#ifdef CONFIG_SET_RAW_ADV_DATA
    uint8_t * raw_adv_data;
    uint32_t raw_adv_data_len;
    
    uint8_t * raw_scan_rsp_data;
    uint32_t raw_scan_rsp_data_len;
#else
	esp_ble_adv_data_t * adv_data;
	esp_ble_adv_data_t * scan_rsp_data;
#endif
    prepare_type_env_t prepare_write_env;
    
    esp_gatts_attr_db_t * gatt_db;
	uint16_t * gatt_db_handle;
    uint32_t gatt_db_len;

	esp_gatts_cb_t gatts_cb;	/* callback fn */
};

const uint16_t primary_service_uuid;         
const uint16_t character_declaration_uuid;   
const uint16_t character_client_config_uuid;

const uint8_t char_prop_read;
const uint8_t char_prop_write;  
const uint8_t char_prop_read_write_notify;

int gatts_server_table_send(esp_gatt_if_t gatts_if, uint8_t handle_idx, uint8_t * pbuf, size_t len);
int gatts_server_table_dev_init(struct gatts_server_dev * dev);
int gatts_server_table_init(void);

#endif