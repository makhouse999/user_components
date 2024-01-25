#ifndef __USER_DRV883X_H__
#define __USER_DRV883X_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

int drv8833_init(void);
int drv8833_set_speed(int motor, int32_t spd);

#endif