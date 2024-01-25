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

#include "driver/i2c.h"
#include "driver/i2s.h"

#include "es8311.h"
#include "user_es8311.h"

#define ES8311_I2C_PORT				CONFIG_ES8311_I2C_PORT
#define ES8311_I2C_ADDR				CONFIG_ES8311_I2C_ADDR
#define ES8311_I2C_MASTER_SDA		CONFIG_ES8311_I2C_MASTER_SDA
#define ES8311_I2C_MASTER_SCL		CONFIG_ES8311_I2C_MASTER_SCL

#define ES8311_I2S_PORT					CONFIG_ES8311_I2S_PORT
#define ES8311_I2S_MCK					CONFIG_ES8311_I2S_MCK
#define ES8311_I2S_BCK					CONFIG_ES8311_I2S_BCK
#define ES8311_I2S_WS					CONFIG_ES8311_I2S_WS
#define ES8311_I2S_DO					CONFIG_ES8311_I2S_DO
#define ES8311_I2S_DI					CONFIG_ES8311_I2S_DI

/* sample params configurations */
#define ES8311_RECV_BUF_SIZE   (2048)
#define ES8311_SAMPLE_RATE     (24000)
#define ES8311_MCLK_MULTIPLE   I2S_MCLK_MULTIPLE_256
#define ES8311_VOICE_VOLUME    CONFIG_ES8311_VOICE_VOLUME
#define ES8311_MIC_GAIN        CONFIG_ES8311_MIC_GAIN

static const char * TAG = "user_es8311";

static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"
                                     };
static SemaphoreHandle_t rw_mutex;

SLIST_HEAD(track_list, track_params);
static struct track_list track_head = SLIST_HEAD_INITIALIZER(track_head);
static es8311_handle_t es_handle;

static esp_err_t es8311_codec_init(void)
{
#if 1
    /* Initialize I2C peripheral */
    i2c_config_t es_i2c_cfg = {
        .sda_io_num = ES8311_I2C_MASTER_SDA,
        .scl_io_num = ES8311_I2C_MASTER_SCL,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(ES8311_I2C_PORT, &es_i2c_cfg), TAG, "config i2c failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(ES8311_I2C_PORT, I2C_MODE_MASTER,  0, 0, 0), TAG, "install i2c driver failed");
#endif
    /* Initialize es8311 codec */
    es_handle = es8311_create(ES8311_I2C_PORT, ES8311_I2C_ADDR);
    ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
    es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = ES8311_SAMPLE_RATE * ES8311_MCLK_MULTIPLE,
        .sample_frequency = ES8311_SAMPLE_RATE
    };

    es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, ES8311_SAMPLE_RATE * ES8311_MCLK_MULTIPLE, ES8311_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, ES8311_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");

    //ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
    //ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN), TAG, "set es8311 microphone gain faield");

    return ESP_OK;
}

static esp_err_t i2s_driver_init(void)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
        .sample_rate = ES8311_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .tx_desc_auto_clear = true,
#if SOC_I2S_SUPPORTS_TDM
        .total_chan = 2,
        .chan_mask = I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1,
        .left_align = false,
        .big_edin = false,
        .bit_order_msb = false,
        .skip_msk = false,
#endif
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .mclk_multiple = ES8311_MCLK_MULTIPLE,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    };

    ESP_RETURN_ON_ERROR(i2s_driver_install(ES8311_I2S_PORT, &i2s_cfg, 0, NULL), TAG, "install i2s failed");
    i2s_pin_config_t i2s_pin_cfg = {
        .mck_io_num = ES8311_I2S_MCK,
        .bck_io_num = ES8311_I2S_BCK,
        .ws_io_num = ES8311_I2S_WS,
        .data_out_num = ES8311_I2S_DO,
        .data_in_num = ES8311_I2S_DI
    };
    ESP_RETURN_ON_ERROR(i2s_set_pin(ES8311_I2S_PORT, &i2s_pin_cfg), TAG, "set i2s pins failed");
    return ESP_OK;
}


/* TODO 添加播放线程 */
static void es8311_play_thread(void *pvParameters)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
	struct track_params * np = (struct track_params *)pvParameters;

	if(xSemaphoreTake(rw_mutex, pdMS_TO_TICKS(np->blk_fg ? portMAX_DELAY : 100)) != pdTRUE){
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "%s: %s", __func__, np->name);
    /* Write music to earphone */
    ret = i2s_write(ES8311_I2S_PORT, np->pcm_start,  np->pcm_end - np->pcm_start, &bytes_write, portMAX_DELAY);
	if (ret != ESP_OK) {
        /* Since we set timeout to 'portMAX_DELAY' in 'i2s_write'
           so you won't reach here unless you set other timeout value,
           if timeout detected, it means write operation failed. */
        ESP_LOGE(TAG, "[music] i2s read failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
		goto EXIT;
    }

    /* Clear DMA buffer to avoid noise from legacy data in buffer */
	    i2s_zero_dma_buffer(ES8311_I2S_PORT);
    if (bytes_write > 0) {
        ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.", bytes_write);
    } else {
        ESP_LOGE(TAG, "[music] i2s music play falied.");
    }
EXIT:
	xSemaphoreGive(rw_mutex);

	vTaskDelete(NULL);
}

int es8311_play(char * name, int vol)
{
	struct track_params * np = NULL;
	static int pre_vol = 0;

	SLIST_FOREACH(np, &track_head, next){
		if(strcmp(np->name, name) == 0){
			break;
		}
	}

	if(np == NULL){
		return -1;
	}

	if(pre_vol != vol){
		es8311_voice_volume_set(es_handle, vol, NULL);
		pre_vol = vol;
	}
	xTaskCreate(es8311_play_thread, "es8311_play_thread", 50 * 1024, np, 4, NULL);

	return 0;
}

int user_es8311_init(struct es8311_dev_desc * desc)
{
    /* Initialize i2s peripheral */
    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    }
    /* Initialize i2c peripheral and config es8311 codec by i2c */
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    }

	rw_mutex = xSemaphoreCreateMutex();

	/* regist track */
	for(size_t i = 0;i < desc->track_nbr;i++){
		SLIST_INSERT_HEAD(&track_head, &desc->track_handle[i], next);
	}

	return 0;
}

int user_es8311_volume_set(int vol)
{
	es8311_voice_volume_set(es_handle, vol, NULL);
	return 0;
}
