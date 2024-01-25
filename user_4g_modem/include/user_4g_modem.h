#ifndef __USER_4G_MODEM_H__
#define __USER_4G_MODEM_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>


int user_4g_modem_init(void);

#endif