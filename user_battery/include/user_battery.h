#ifndef __USER_BATTERY_H__
#define __USER_BATTERY_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>


int battery_get_voltage(void);
int battery_get_percentage(void);
esp_err_t battery_voltage_init(void);

#endif