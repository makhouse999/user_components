#ifndef __USER_WIFI_H__
#define __USER_WIFI_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/queue.h>

#include "esp_wifi.h"

int user_wifi_init(wifi_mode_t mode);
esp_netif_t * user_wifi_get_netif(void);
int user_wifi_sta_get_rssi(void);
#endif
