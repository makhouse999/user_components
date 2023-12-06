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


#define MB_PORT_NUM     (CONFIG_MB_SLAVE_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   (CONFIG_MB_SLAVE_ADDR)      // The address of device in Modbus network
#define MB_DEV_SPEED    (CONFIG_MB_SLAVE_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

static const char *TAG = "user_modbus_slave";

SLIST_HEAD(dev_list, mb_slave_dev_desc);
static struct dev_list dev_head = SLIST_HEAD_INITIALIZER(dev_head);

int mb_slave_dev_register(struct mb_slave_dev_desc * desc)
{
	/* link to the table */
	SLIST_INSERT_HEAD(&dev_head, desc, next);
	return 0;
}

static void slave_operation_func(void *arg)
{
	mb_param_info_t reg_info; // keeps the Modbus registers access information
	struct mb_slave_dev_desc * np;

    for( ;; ) {
        // Check for read/write events of Modbus master for certain events
        mb_event_group_t event = mbc_slave_check_event(MB_READ_WRITE_MASK);
		ESP_LOGI(TAG, "mbc_slave_check_event rcv event");
        const char* rw_str = (event & MB_READ_MASK) ? "READ" : "WRITE";

		if(event == MB_EVENT_NO_EVENTS){
			ESP_LOGI(TAG, "NO event Quit");
			continue;			
		}

		// Get parameter information from parameter queue
		ESP_ERROR_CHECK(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));

        ESP_LOGI(TAG, "EVENT %d %s (%u us), ADDR:%u, TYPE:%u, INST_ADDR:0x%.4x, SIZE:%u",
		(uint8_t)event,
        rw_str,
        (uint32_t)reg_info.time_stamp,
        (uint32_t)reg_info.mb_offset,
        (uint32_t)reg_info.type,
        (uint32_t)reg_info.address,
        (uint32_t)reg_info.size);

		SLIST_FOREACH(np, &dev_head, next){
			if(np->cb){
				np->cb(event, &reg_info);
			}
		}

        // Filter events and process them accordingly
        if(event & MB_EVENT_HOLDING_REG_WR) {

		} else if (event & MB_EVENT_HOLDING_REG_RD){

		} else if (event & MB_EVENT_INPUT_REG_RD) {

        } else if (event & MB_EVENT_DISCRETE_RD) {

        } else if (event & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR)) {

        }
    }
    // Destroy of Modbus controller on alarm
    ESP_LOGI(TAG,"Modbus controller destroyed.");
    vTaskDelay(100);
    ESP_ERROR_CHECK(mbc_slave_destroy());	
}

void modbus_slave_init(uint32_t baud, uint8_t addr)
{

    mb_communication_info_t comm_info; // Modbus communication parameters
	void* mbc_slave_handler = NULL;
	struct mb_slave_dev_desc * np;

    ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller

    // Setup communication parameters and start stack
#if CONFIG_MB_SLAVE_COMM_MODE_ASCII
    comm_info.mode = MB_MODE_ASCII,
#elif CONFIG_MB_SLAVE_COMM_MODE_RTU
    comm_info.mode = MB_MODE_RTU,
#endif
    comm_info.slave_addr = addr;//;MB_SLAVE_ADDR
    comm_info.port = MB_PORT_NUM;
    comm_info.baudrate = baud;	//MB_DEV_SPEED
    comm_info.parity = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.

	SLIST_FOREACH(np, &dev_head, next){
		ESP_ERROR_CHECK(mbc_slave_set_descriptor(np->reg_area));
	}

    // Starts of modbus controller and stack
    ESP_ERROR_CHECK(mbc_slave_start());

    // Set UART pin numbers
    ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, CONFIG_MB_SLAVE_UART_TXD,
                            CONFIG_MB_SLAVE_UART_RXD, CONFIG_MB_SLAVE_UART_RTS,
                            UART_PIN_NO_CHANGE));

    // Set UART driver mode to Half Duplex
    ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    ESP_LOGI(TAG, "Start modbus slave...");
	
	xTaskCreate(slave_operation_func, "slave_operation_func", 40 * 1024, NULL, 5, NULL);
}


