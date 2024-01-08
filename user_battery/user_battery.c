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

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "user_battery.h"

#define BATTERY_ADC_CH 				CONFIG_BATTERY_ADC_CH
#define BATTERY_ADC_ATTEN 			CONFIG_BATTERY_ADC_ATTEN
#define BATTERY_ADC_CALI_SCHEME 	CONFIG_BATTERY_ADC_CALI_SCHEME
#define BATTERY_ADC_VOLTAGE_DIV		CONFIG_BATTERY_ADC_VOLTAGE_DIV

#define BATTERY_VOLTAGE_MAX			4100	/* mV */
#define BATTERY_VOLTAGE_MIN			3550	/* mV */

static esp_adc_cal_characteristics_t bsp_adc_chars;

esp_err_t bsp_adc_initialize(void)
{
    esp_err_t ret = ESP_OK;
    ESP_ERROR_CHECK(esp_adc_cal_check_efuse(BATTERY_ADC_CALI_SCHEME));
    esp_adc_cal_characterize(ADC_UNIT_1, BATTERY_ADC_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &bsp_adc_chars);

    /* ADC1 config */
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    return ret;
}

esp_err_t battery_voltage_init(void)
{
    ESP_ERROR_CHECK(bsp_adc_initialize());
    ESP_ERROR_CHECK(adc1_config_channel_atten(BATTERY_ADC_CH, BATTERY_ADC_ATTEN));

    return ESP_OK;
}

/* unit: mV */
int battery_get_voltage(void)
{
    int voltage, adc_raw;

    adc_raw = adc1_get_raw(BATTERY_ADC_CH);
    voltage = esp_adc_cal_raw_to_voltage(adc_raw, &bsp_adc_chars);
    return voltage * BATTERY_ADC_VOLTAGE_DIV;
}

/* unit: % */
int battery_get_percentage(void)
{
	int vol = (int)(((float)(battery_get_voltage() - BATTERY_VOLTAGE_MIN) * 100 / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)));
	vol = vol > 100 ? 100 : vol;
	vol = vol < 0 ? 0 : vol;
	return vol;

}