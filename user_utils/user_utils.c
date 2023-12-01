#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include "time.h"
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "user_utils.h"

static const char *TAG = "UTILS";

/* 设置大类保存区 */
static struct nvs_user_zone * str_hdl;
static struct nvs_user_zone * blob_hdl;

#define NVS_KEY_SZ	16
static uint8_t nvs_user_key_num = 0;
static struct nvs_user_key nvs_key[NVS_KEY_SZ]; 

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
	uint8_t i;

	for(i = 0;i < nvs_user_key_num;i++){
		if(strcmp(nvs_key[i].name, key) == 0){
			break;
		}
	}

	if(i == nvs_user_key_num){
		ESP_LOGE(TAG, "NO registe nvs key was found");
		return NULL;
	}


	if(nvs_open(nvs_key[i].handle->name, NVS_READONLY, &handle) != ESP_OK){
		ESP_LOGE(TAG, "nvs_open %s fail", nvs_key[i].handle->name);
		return NULL;
	}

	/* 获取数据长度 */
	if(nvs_key[i].handle->ty == TY_STR){
		ret = nvs_get_str(handle, key, NULL, len);
	} else if(nvs_key[i].handle->ty == TY_BLOB){
		ret = nvs_get_blob(handle, key, NULL, len);
	}

	if(ret != ESP_OK){
		ESP_LOGE(TAG, "nvs get len fail [%s], ret = 0x%x", key, ret);
		goto EXIT;
	}

	str = pbuf == NULL ? malloc(*len + 32) : pbuf;

	if(nvs_key[i].handle->ty == TY_STR){
		ret = nvs_get_str(handle, key, str, len);
	} else if(nvs_key[i].handle->ty == TY_BLOB) {
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
	uint8_t i;

	for(i = 0;i < nvs_user_key_num;i++){
		if(strcmp(nvs_key[i].name, key) == 0){
			break;
		}
	}

	if(i == nvs_user_key_num){
		ESP_LOGE(TAG, "NO registe nvs key was found");
		return -1;
	}

	if((ret = nvs_open(nvs_key[i].handle->name, NVS_READWRITE, &handle)) != ESP_OK){
		ESP_LOGE(TAG, "nvs_open error ret = 0x%x", ret);
		return -1;
	}

	if(nvs_key[i].handle->ty == TY_STR){
		ret = nvs_set_str(handle, key, pbuf);
	}else if(nvs_key[i].handle->ty == TY_BLOB){
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
	uint8_t i;

	for(i = 0;i < nvs_user_key_num;i++){
		if(strcmp(nvs_key[i].name, key) == 0){
			break;
		}
	}

	if(i == nvs_user_key_num){
		ESP_LOGE(TAG, "NO registe nvs key was found");
		return -1;
	}

	if(nvs_open(nvs_key[i].handle->name, NVS_READWRITE, &handle) != ESP_OK){
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
	strcpy(nvs_key[nvs_user_key_num].name, name);

	if(ty == TY_STR){
		nvs_key[nvs_user_key_num].handle = str_hdl;
	}else if(ty == TY_BLOB){
		nvs_key[nvs_user_key_num].handle = blob_hdl;
	}else{
		return -1;
	}

	nvs_user_key_num++;
	return 0;
}

int user_utils_init()
{
	str_hdl = create_nvs("user_str", TY_STR);
	blob_hdl = create_nvs("user_blob", TY_BLOB);

	return 0;
}