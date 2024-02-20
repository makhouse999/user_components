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
#include "esp_check.h"

#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#include "user_ota.h"


static const char *TAG = "ota";

struct ota_info ota_info;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void simple_ota_task(void *pvParameter)
{
	xSemaphoreTake(ota_info.update_mutex, pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Starting OTA example");
	const char * url = (const char *)pvParameter;
	ESP_LOGW(TAG, "url = %s", url);
	/* switch wifi to no power save mode */
	esp_wifi_set_ps(WIFI_PS_NONE);

#ifdef CONFIG_OTA_FIRMWARE_UPGRADE_BIND_IF
    esp_netif_t *netif = get_example_netif_from_desc(bind_interface_name);
    if (netif == NULL) {
        ESP_LOGE(TAG, "Can't find netif from interface description");
        abort();
    }
    struct ifreq ifr;
    esp_netif_get_netif_impl_name(netif, ifr.ifr_name);
    ESP_LOGI(TAG, "Bind interface name is %s", ifr.ifr_name);
#endif
    esp_http_client_config_t config = {
        .url = url,
        //.cert_pem = (char *)server_cert_pem_start,
        .event_handler = _http_event_handler,
        .keep_alive_enable = true,
#ifdef CONFIG_OTA_FIRMWARE_UPGRADE_BIND_IF
        .if_name = &ifr,
#endif
    };

#ifdef CONFIG_OTA_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
		while (1) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }

	xSemaphoreGive(ota_info.update_mutex);
}

void user_ota(char * url)
{
	if(ota_info.update_mutex == NULL){
		ota_info.update_mutex = xSemaphoreCreateMutex();	
	}
	xTaskCreate(&simple_ota_task, "simple_ota_task", 1024 * 50, url, 5, NULL);
}
