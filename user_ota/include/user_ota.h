#ifndef __USER_DRV883X_H__
#define __USER_DRV883X_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>


struct ota_info {
	SemaphoreHandle_t update_mutex;
};

void user_ota(char * url);

#endif