/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file hx710a.c
 *
 * ESP-IDF driver for hx710a 24-bit ADC for weigh scales
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include <sys/queue.h>
#include <inttypes.h>


#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include <esp_timer.h>

#include "user_hx710a.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif


#define HX710A_RES	CONFIG_HX710A_RES

static const char *TAG = "user_hx710a";
struct hx71x_info Hx71x_info;

static uint32_t read_raw(gpio_num_t dout, gpio_num_t pd_sck, hx710a_gain_t gain)
{
#if HELPER_TARGET_IS_ESP32
    portENTER_CRITICAL(&mux);
#elif HELPER_TARGET_IS_ESP8266
    portENTER_CRITICAL();
#endif

    // read data
    uint32_t data = 0;
    for (size_t i = 0; i < 24; i++)
    {
        gpio_set_level(pd_sck, 1);
        ets_delay_us(1);
        data |= gpio_get_level(dout) << (23 - i);
        gpio_set_level(pd_sck, 0);
        ets_delay_us(1);
    }

    // config gain + channel for next read
    for (size_t i = 0; i <= gain; i++)
    {
        gpio_set_level(pd_sck, 1);
        ets_delay_us(1);
        gpio_set_level(pd_sck, 0);
        ets_delay_us(1);
    }

#if HELPER_TARGET_IS_ESP32
    portEXIT_CRITICAL(&mux);
#elif HELPER_TARGET_IS_ESP8266
    portEXIT_CRITICAL();
#endif

    return data;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t hx710a_init(hx710a_t *dev)
{
    CHECK_ARG(dev);

    CHECK(gpio_set_direction(dev->dout, GPIO_MODE_INPUT));
    CHECK(gpio_set_direction(dev->pd_sck, GPIO_MODE_OUTPUT));

    CHECK(hx710a_power_down(dev, false));

    return hx710a_set_gain(dev, dev->gain);
}

esp_err_t hx710a_power_down(hx710a_t *dev, bool down)
{
    CHECK_ARG(dev);

    CHECK(gpio_set_level(dev->pd_sck, down));
    vTaskDelay(1);

    return ESP_OK;
}

esp_err_t hx710a_set_gain(hx710a_t *dev, hx710a_gain_t gain)
{
    CHECK_ARG(dev && gain <= HX710A_GAIN_A_64);

    CHECK(hx710a_wait(dev, 1000)); // 200 ms timeout

    read_raw(dev->dout, dev->pd_sck, gain);
    dev->gain = gain;

    return ESP_OK;
}

esp_err_t hx710a_is_ready(hx710a_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);

    *ready = !gpio_get_level(dev->dout);

    return ESP_OK;
}

esp_err_t hx710a_wait(hx710a_t *dev, size_t timeout_ms)
{
    uint64_t started = esp_timer_get_time() / 1000;
    while (esp_timer_get_time() / 1000 - started < timeout_ms)
    {
        if (!gpio_get_level(dev->dout))
            return ESP_OK;
        vTaskDelay(1);
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t hx710a_read_data(hx710a_t *dev, int32_t *data)
{
    CHECK_ARG(dev && data);

    uint32_t raw = read_raw(dev->dout, dev->pd_sck, dev->gain);
    if (raw & 0x800000)
        raw |= 0xff000000;

    *data = *((int32_t *)&raw);

    return ESP_OK;
}

esp_err_t hx710a_read_average(hx710a_t *dev, size_t times, int32_t *data)
{
    CHECK_ARG(dev && times && data);

    int32_t v;
    *data = 0;
    for (size_t i = 0; i < times; i++)
    {
        CHECK(hx710a_wait(dev, 200));
        CHECK(hx710a_read_data(dev, &v));
        *data += v;
    }
    *data /= (int32_t) times;

    return ESP_OK;
}

static void hx710a_thread(void *pvParameters)
{
    hx710a_t dev = {
        .dout = CONFIG_HX710A_DOUT_GPIO,
        .pd_sck = CONFIG_HX710A_PD_SCK_GPIO,
        .gain = HX710A_GAIN_A_128
    };

    // initialize device
    ESP_ERROR_CHECK(hx710a_init(&dev));

    // read from device
    while (1)
    {
        esp_err_t r = hx710a_wait(&dev, 500);
        if (r != ESP_OK){
            ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

        int32_t data;
        r = hx710a_read_average(&dev, CONFIG_HX710A_AVG_TIMES, &data);
        if (r != ESP_OK){
            ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
            continue;
        }

		Hx71x_info.raw_data = data;
		Hx71x_info.weight = (float)data / HX710A_RES;

        ESP_LOGI(TAG, "Raw data: %" PRIi32, data);
		ESP_LOGI(TAG, "Weight: %.1f", Hx71x_info.weight);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }	
}

int hx71x_init()
{
	xTaskCreate(hx710a_thread, "hx710a", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
	return 0;
}

struct hx71x_info * hx71x_get_info()
{
	return &Hx71x_info;
}