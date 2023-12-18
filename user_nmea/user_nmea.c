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


#include "nmea_parser.h"
#include "user_nmea.h"

#define GPIO_NMEA_PARSER_UART_RXD 	CONFIG_NMEA_PARSER_UART_RXD
#define NMEA_UART_PORT_NUM			CONFIG_NMEA_UART_PORT_NUM

#define TIME_ZONE (+8)   //Beijing Time
#define YEAR_BASE (2000) //date in GPS starts from 2000

static const char *TAG = "user_nmea";
static gps_t gps;
/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case GPS_UPDATE:
        memcpy(&gps, event_data, sizeof(gps_t));

        /* print information parsed from GPS statements */
        ESP_LOGD(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps.date.year + YEAR_BASE, gps.date.month, gps.date.day,
                 gps.tim.hour + TIME_ZONE, gps.tim.minute, gps.tim.second,
                 gps.latitude, gps.longitude, gps.altitude, gps.speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGD(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

gps_t * nmea_get_gps()
{
	return &gps;
}

int nmea_init()
{
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
	config.uart.uart_port = NMEA_UART_PORT_NUM;
	config.uart.rx_pin = GPIO_NMEA_PARSER_UART_RXD;

    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);	

	return 0;
}