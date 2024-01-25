idf_component_register(SRCS "user_4g_modem.c" "modem_http_config.c"
                    INCLUDE_DIRS "include"
					REQUIRES iot_usbh_modem led_indicator esp_http_server spiffs esp-tls nvs_flash
					)
