#ifndef __USER_NMEA_H__
#define __USER_NMEA_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>
#include "nmea_parser.h"

gps_t * nmea_get_gps();
int nmea_init(void);

#endif

