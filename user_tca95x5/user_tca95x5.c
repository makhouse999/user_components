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

#include "tca95x5.h"
#include "user_tca95x5.h"

#define TCA95X5_I2C_PORT			CONFIG_TCA95X5_I2C_PORT
#define TCA95X5_I2C_ADDR			CONFIG_TCA95X5_I2C_ADDR
#define TCA95X5_I2C_MASTER_SDA		CONFIG_TCA95X5_I2C_MASTER_SDA
#define TCA95X5_I2C_MASTER_SCL		CONFIG_TCA95X5_I2C_MASTER_SCL

static const char *TAG = "user_tca95x5";
static i2c_dev_t tca9555 = { 0 };

uint16_t tca95x5_get_val()
{
	uint16_t val;

    esp_err_t res = tca95x5_port_read(&tca9555, &val);

    if (res != ESP_OK){
        ESP_LOGE(TAG, "Error reading TCA9555: %d (%s)", res, esp_err_to_name(res));
		return 0;
    }

    ESP_LOGI(TAG, "TCA9555 port value: 0x%04x", val);
	
	return val;	
}

/* 
	io_mode bits P17 ... P10 P07 ... P00 
	input: set bit 1
	output: set bit 0
	example: b10000000 00000001 = 0x8001, means P17, P00 set input, others set output
*/
int tca95x5_init(uint16_t io_mode)
{
    ESP_ERROR_CHECK(i2cdev_init());	/* TODO put it in ctrl layer */

	// Init descriptor
    ESP_ERROR_CHECK(tca95x5_init_desc(&tca9555, TCA95X5_I2C_ADDR, TCA95X5_I2C_PORT, TCA95X5_I2C_MASTER_SDA, TCA95X5_I2C_MASTER_SCL));

    // Setup P00, P01 and P02 as input, others as output
    ESP_ERROR_CHECK(tca95x5_port_set_mode(&tca9555, io_mode));

	return 0;
}
