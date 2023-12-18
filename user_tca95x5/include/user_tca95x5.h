#ifndef __USER_TCA95X5_H__
#define __USER_TCA95X5_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>

uint16_t tca95x5_get_val(void);
int tca95x5_init(uint16_t io_mode);

#endif