#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_mac.h"

#include "user_utils.h"

static const char *TAG = "user_utils";

/* setup zones for differen types of var */
static struct nvs_user_zone * str_hdl;		/* string typs vars */
static struct nvs_user_zone * blob_hdl;		/* blob type vars */

SLIST_HEAD(nvs_key_list, nvs_user_key);
static struct nvs_key_list nvs_key_head = SLIST_HEAD_INITIALIZER(nvs_key_head);

uint32_t get_time()
{
	struct timeval ts_now;
	gettimeofday(&ts_now, NULL);
	return (uint32_t)ts_now.tv_sec;
}

int set_time(uint32_t ts)
{
	struct timeval now = { .tv_sec = (time_t)ts };
    settimeofday(&now, NULL);
	ESP_LOGI(TAG, "sys set ts: %d", get_time());
	return 0;
}

int str_trim_space(char * in, char * out)
{
	if(in == NULL || out == NULL){
		return -1;
	}

	char * pin = in, * pout = out;
	while(*pin != '\0'){
		if(*pin != ' '){
			*pout = *pin;
			pout++;
		}
		pin++;
	}
	return 0;
}


char * get_nvs(char * key, void * pbuf, size_t * len)
{
	nvs_handle_t handle;
	char * str = NULL;
	esp_err_t ret = ESP_OK;

	struct nvs_user_key * np = NULL;

	SLIST_FOREACH(np, &nvs_key_head, next){
		if(strcmp(np->name, key) == 0){
			break;
		}
	}

	if(np == NULL){
		return NULL;
	}

	if(nvs_open(np->handle->name, NVS_READONLY, &handle) != ESP_OK){
		ESP_LOGE(TAG, "nvs_open %s fail", np->handle->name);
		return NULL;
	}

	if(np->handle->ty == TY_STR){
		ret = nvs_get_str(handle, key, NULL, len);
	} else if(np->handle->ty == TY_BLOB){
		ret = nvs_get_blob(handle, key, NULL, len);
	}

	if(ret != ESP_OK){
		ESP_LOGE(TAG, "nvs get len fail [%s], ret = 0x%x", key, ret);
		goto EXIT;
	}

	str = pbuf == NULL ? malloc(*len + 32) : pbuf;

	if(np->handle->ty == TY_STR){
		ret = nvs_get_str(handle, key, str, len);
	} else if(np->handle->ty == TY_BLOB) {
		ret = nvs_get_blob(handle, key, str, len);
	}

	if(ret != ESP_OK){
		ESP_LOGE(TAG, "nvs get data fail [%s], len = %d, ret = 0x%x", key, *len, ret);
		if(pbuf == NULL){
			free(str);
		}
		str = NULL;
		goto EXIT;
	}

EXIT:
	nvs_close(handle);
	return str;
}

int set_nvs(char * key, void * pbuf, size_t len)
{
	nvs_handle_t handle;
	esp_err_t ret;

	struct nvs_user_key * np = NULL;

	SLIST_FOREACH(np, &nvs_key_head, next){
		if(strcmp(np->name, key) == 0){
			break;
		}
	}

	if(np == NULL){
		return -1;
	}

	if((ret = nvs_open(np->handle->name, NVS_READWRITE, &handle)) != ESP_OK){
		ESP_LOGE(TAG, "nvs_open error ret = 0x%x", ret);
		return -1;
	}

	if(np->handle->ty == TY_STR){
		ret = nvs_set_str(handle, key, pbuf);
	}else if(np->handle->ty == TY_BLOB){
		ret = nvs_set_blob(handle, key, pbuf, len);
	}
	ESP_ERROR_CHECK(ret);

	nvs_commit(handle);
	nvs_close(handle);

	return (int)ret;
}

int clear_nvs(char * key)
{
	nvs_handle_t handle;

	struct nvs_user_key * np = NULL;

	SLIST_FOREACH(np, &nvs_key_head, next){
		if(strcmp(np->name, key) == 0){
			break;
		}
	}

	if(np == NULL){
		return -1;
	}

	if(nvs_open(np->handle->name, NVS_READWRITE, &handle) != ESP_OK){
		return -1;
	}

	if(key == NULL){
		nvs_erase_all(handle);
	}else{
		nvs_erase_key(handle, key);
	}

	nvs_commit(handle);
	nvs_close(handle);

	return 0;
}

static struct nvs_user_zone * create_nvs(const char * name, enum nvs_dat_ty ty)
{
	struct nvs_user_zone * hdl;
	hdl = malloc(sizeof(struct nvs_user_zone));
	strcpy(hdl->name, name);
	hdl->ty = ty;
	return hdl;
}

int create_nvs_key(const char * name, enum nvs_dat_ty ty)
{
	if(name == NULL){
		return -1;
	}

	struct nvs_user_key * np = malloc(sizeof(struct nvs_user_key));
	strcpy(np->name, name);

	if(ty == TY_STR){
		np->handle = str_hdl;
	}else if(ty == TY_BLOB){
		np->handle = blob_hdl;
	}else{
		free(np);
		return -1;
	}

	/* link to the table */
	SLIST_INSERT_HEAD(&nvs_key_head, np, next);

	return 0;
}

int user_utils_init()
{
#if 0
	/* erase nvs infomation mannualy if needed */
	ESP_ERROR_CHECK(nvs_flash_erase());
#endif
#if 1
    /* Initialize NVS for storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs to be erased
         * Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
	ESP_ERROR_CHECK( ret );
#endif


	str_hdl = create_nvs("user_str", TY_STR);
	blob_hdl = create_nvs("user_blob", TY_BLOB);

	return 0;
}