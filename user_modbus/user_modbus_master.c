// Copyright 2016-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include <sys/queue.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "mbcontroller.h"

#include "user_modbus.h"

#define MB_PORT_NUM     (CONFIG_MB_MASTER_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (CONFIG_MB_MASTER_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Timeout between polls
#define POLL_TIMEOUT_MS                 (500)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

/* user define */
#define DEV_TBL_SZ		128		

static const char * TAG = "user mb_master";
static SemaphoreHandle_t rw_mutex;

static mb_parameter_descriptor_t device_parameters[DEV_TBL_SZ];
static uint16_t num_device_parameters;

SLIST_HEAD(dev_list, mb_dev_desc);
static struct dev_list dev_head = SLIST_HEAD_INITIALIZER(dev_head);

int mb_master_dev_register(struct mb_dev_desc * desc)
{
	if(num_device_parameters >= DEV_TBL_SZ){
		ESP_LOGE(TAG, "device_parameters OVER FLOW, please enlarge the table");
		return -1;
	}

	/* link to the table */
	SLIST_INSERT_HEAD(&dev_head, desc, next);

	mb_parameter_descriptor_t * p = &device_parameters[num_device_parameters];
	memcpy(p, &desc->params, sizeof(mb_parameter_descriptor_t));
	p->cid = num_device_parameters;
	desc->params.cid = num_device_parameters;

	//ESP_LOGI(TAG, "%s, cid: %d, key: %s", __func__, desc->params.cid, desc->params.param_key);

	num_device_parameters++;

	return 0;

}

esp_err_t modbus_master_rw(enum mb_rw_mode mode, const char * param_key, uint8_t * val)
{
	esp_err_t err = ESP_OK;
	struct mb_dev_desc * np = NULL;
	mb_parameter_descriptor_t* param_descriptor = NULL;

	uint8_t type = 0;

	SLIST_FOREACH(np, &dev_head, next){
		if(strcmp(np->params.param_key, param_key) == 0){
			break;
		}
	}

	if(np == NULL){
		return ESP_FAIL;
	}

	param_descriptor = &np->params;

	xSemaphoreTake(rw_mutex, portMAX_DELAY);

	ESP_LOGI(TAG, "mbc_master_get_parameter() cid: %d, param_key: %s", param_descriptor->cid, param_descriptor->param_key);

	if(mode == MB_MODE_READ){
	    err = mbc_master_get_parameter(param_descriptor->cid, (char*)param_descriptor->param_key,
			(uint8_t*)val, &type);
	}else if(mode == MB_MODE_WRITE){
        err = mbc_master_set_parameter(param_descriptor->cid, (char*)param_descriptor->param_key,
			 (uint8_t*)val, &type);
	}

	xSemaphoreGive(rw_mutex);

	return err;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    bool alarm_state = false;
    mb_parameter_descriptor_t* param_descriptor = NULL;

	uint8_t val[2048];
	struct mb_dev_desc * np;

	vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Start modbus test...");

	for(;(!alarm_state);){
        // Read all found characteristics from slave(s)
		SLIST_FOREACH(np, &dev_head, next){
			param_descriptor = &np->params;

			if (param_descriptor == NULL) {
				continue;
			}

			if((param_descriptor->access & PAR_PERMS_READ) == 0){
				continue;
			}

            if (param_descriptor->param_type == PARAM_TYPE_ASCII) {
				// Check for long array of registers of type PARAM_TYPE_ASCII
				err = modbus_master_rw(MB_MODE_READ, (const char*)param_descriptor->param_key, (uint8_t *)val);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Characteristic #%d %s (%s) value = (0x%08x) read successful.",
                                             param_descriptor->cid,
                                             (char*)param_descriptor->param_key,
                                             (char*)param_descriptor->param_units,
                                             *(uint32_t *)val);


                } else {
                    ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (int)err,
                                            (char*)esp_err_to_name(err));
                }

            } else {
				err = modbus_master_rw(MB_MODE_READ, (const char*)param_descriptor->param_key, (uint8_t *)val);

                if (err == ESP_OK) {
					char ack_params_key[32] = {0};
					uint8_t ack_val[8];
					if(np->cb){
						np->cb(param_descriptor, val, ack_params_key, ack_val);
					}

					if(ack_params_key[0] != 0){
						struct mb_dev_desc * ack_np = NULL;
						SLIST_FOREACH(ack_np, &dev_head, next){
							if(strcmp(ack_np->params.param_key, ack_params_key) == 0)
								break;
						}

						if(ack_np){
							ESP_LOGI(TAG, "reply: cid = %d, key = %s", ack_np->params.cid, ack_np->params.param_key);
							err = modbus_master_rw(MB_MODE_WRITE, (const char*)param_descriptor->param_key, (uint8_t *)val);

                            if (err != ESP_OK) {
								ESP_LOGE(TAG, "Characteristic, write failed.");
							}							
						}

					}
					
                } else {
                    ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                }
            }
            vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls			
		}

    }

    ESP_LOGI(TAG, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_MASTER_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_MASTER_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_MASTER_UART_TXD, CONFIG_MB_MASTER_UART_RXD,
                              CONFIG_MB_MASTER_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

void modbus_master_init()
{
    // Initialization of device peripheral and objects
	/* 调用初始化前必须先注册所有设备描述符 */
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(pdMS_TO_TICKS(10));
	rw_mutex = xSemaphoreCreateMutex();	
	xTaskCreate(master_operation_func, "master_operation_func", 40 * 1024, NULL, 5, NULL);
}


