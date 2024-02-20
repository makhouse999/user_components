idf_component_register(SRCS "user_ota.c"
                    INCLUDE_DIRS "include"
					REQUIRES app_update esp_http_client esp_https_ota)
