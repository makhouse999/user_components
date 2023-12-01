#ifndef __USER_MODBUS_H__
#define __USER_MODBUS_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>

#include "mbcontroller.h"

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

struct mb_dev_desc {
	mb_parameter_descriptor_t params;
	int (* cb)(mb_parameter_descriptor_t * params, uint8_t * val, char * ack_params_key, uint8_t * ack_val);
	SLIST_ENTRY(mb_dev_desc) next;
};

void modbus_master_init(void);
int mb_master_dev_register(struct mb_dev_desc * desc);

struct mb_slave_dev_desc {
	mb_register_area_descriptor_t reg_area;
	int (* cb)( mb_event_group_t event, mb_param_info_t * reg_info);
	SLIST_ENTRY(mb_slave_dev_desc) next;
};

void modbus_slave_init(uint32_t baud, uint8_t addr);
int mb_slave_dev_register(struct mb_slave_dev_desc * desc);
#endif