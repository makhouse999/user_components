#ifndef __USER_NMEA_H__
#define __USER_NMEA_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>
#include "nmea_parser.h"

struct user_nmea_info {
	gps_t gps;
};

struct user_nmea_info * user_nmea_get_info(void);
int nmea_init(void);

#endif

