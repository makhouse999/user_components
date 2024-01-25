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
 * @file hx710a.h
 * @defgroup hx710a hx710a
 * @{
 *
 * ESP-IDF driver for hx710a 24-bit ADC for weigh scales
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __USER_HX710A_H__
#define __USER_HX710A_H__

#include <driver/gpio.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Gain/channel
 */
typedef enum {
    HX710A_GAIN_A_128 = 0, //!< Channel A, gain factor 128
    HX710A_GAIN_B_32,      //!< Channel B, gain factor 32
    HX710A_GAIN_A_64       //!< Channel A, gain factor 64
} hx710a_gain_t;

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t dout;
    gpio_num_t pd_sck;
    hx710a_gain_t gain;
} hx710a_t;

struct hx71x_info {
	uint8_t raw_data;
	float weight;	/* kg */
};

/**
 * @brief Initialize device
 *
 * Prepare GPIO pins, power up device and set gain
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_TIMEOUT` if device not found
 */
esp_err_t hx710a_init(hx710a_t *dev);

/**
 * @brief Set device power up/down
 *
 * @param dev Device descriptor
 * @param down Set device power down if true, power up otherwise
 * @return `ESP_OK` on success
 */
esp_err_t hx710a_power_down(hx710a_t *dev, bool down);

/**
 * @brief Set device gain and channel
 *
 * @param dev Device descriptor
 * @param gain Gain + channel value
 * @return `ESP_OK` on success, `ESP_ERR_TIMEOUT` if device not found
 */
esp_err_t hx710a_set_gain(hx710a_t *dev, hx710a_gain_t gain);

/**
 * @brief Check if device ready to send data
 *
 * @param dev Device descriptor
 * @param[out] ready true if data ready
 * @return `ESP_OK` on success
 */
esp_err_t hx710a_is_ready(hx710a_t *dev, bool *ready);

/**
 * @brief Wait for device to become ready
 *
 * @param dev Device descriptor
 * @param timeout_ms Maximum time to wait, milliseconds
 * @return `ESP_OK` on success
 */
esp_err_t hx710a_wait(hx710a_t *dev, size_t timeout_ms);

/**
 * @brief Read raw data from device.
 *
 * Please call this function only when device is ready,
 * otherwise communication errors may occur
 *
 * @param dev Device descriptor
 * @param[out] data Raw ADC data
 * @return `ESP_OK` on success
 */
esp_err_t hx710a_read_data(hx710a_t *dev, int32_t *data);

/**
 * @brief Read average data
 *
 * @param dev Device descriptor
 * @param times Count of samples to read
 * @param[out] data Average ADC data
 * @return `ESP_OK` on success
 */
esp_err_t hx710a_read_average(hx710a_t *dev, size_t times, int32_t *data);

int hx71x_init(void);
struct hx71x_info * hx71x_get_info(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __USER_HX710A_H__ */