
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <sys/queue.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "user_hp303s.h"


#define HC138_A0		CONFIG_74HC138_A0_GPIO
#define HC138_A1		CONFIG_74HC138_A1_GPIO
#define HC138_A2		CONFIG_74HC138_A2_GPIO

#define PIN_NUM_MOSI	CONFIG_HP303S_MOSI_GPIO
#define PIN_NUM_MISO	CONFIG_HP303S_MISO_GPIO
#define PIN_NUM_CLK		CONFIG_HP303S_CLK_GPIO

#define PIN_NUM_E2		GPIO_NUM_45

#define SPI_HOST    	SPI2_HOST

static const char TAG[] = "user_hp303s";

uint32_t kt, kp;

enum hp303s_id bad_id;

const uint32_t k_val[] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

static struct hp303s_params Hp303s_params[ID_MAX];
static struct hp303s_info Hp303s_info;
static spi_device_handle_t spi_handle;

void hp303s_ll_init()
{
	esp_err_t ret;

	for(enum hp303s_id i = ID_0;i < ID_MAX;i++){
		Hp303s_params[i].id = i;
	}
	
	/* 初始化74hc138 */
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << HC138_A0) | (1ULL << HC138_A1) | (1ULL << HC138_A2) | (1ULL << PIN_NUM_E2));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = false;
    gpio_config(&io_conf);
	gpio_set_level(HC138_A0, 0);
	gpio_set_level(HC138_A1, 0);
	gpio_set_level(HC138_A2, 0);
	gpio_set_level(PIN_NUM_E2, 0);

	/* 初始化hp303s */
    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPI_HOST + 1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=4*1000*1000,           	//Clock out at 8 MHz
        .mode=3,                                //SPI mode 3
        .spics_io_num=-1,               		//CS pin sw
        .queue_size=1,                          //We want to be able to queue 1 transactions at a time
		//.command_bits=8,
		.address_bits=8,
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
	ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle));
	
}

static void hp303s_reset_cs(enum hp303s_id id)
{
	/*
		A2 = id & (1 << 2)
		A1 = id & (1 << 1)
		A0 = id & (1 << 0)
	*/
	gpio_set_level(HC138_A2, (id & (1 << 2)) ? 1 : 0);
	gpio_set_level(HC138_A1, (id & (1 << 1)) ? 1 : 0);
	gpio_set_level(HC138_A0, (id & (1 << 0)) ? 1 : 0);
	gpio_set_level(PIN_NUM_E2, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void hp303s_set_cs(enum hp303s_id id)
{
	gpio_set_level(PIN_NUM_E2, 0);
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void hp303s_rd(enum hp303s_id id, uint8_t reg_addr, uint8_t * dat, uint8_t len)
{
	hp303s_reset_cs(id);

	spi_transaction_t t = {
		.addr = reg_addr,
        .length = 8 * len,
		.rx_buffer = dat,
    };
    esp_err_t err = spi_device_polling_transmit(spi_handle, &t);
    if (err!= ESP_OK) {
		ESP_LOGE(TAG, "hp303s_rd ERR %d", err);
	}

	hp303s_set_cs(id);

}

static void hp303s_wr(enum hp303s_id id, uint8_t reg_addr, const uint8_t * dat, uint8_t len)
{
	hp303s_reset_cs(id);

	spi_transaction_t t = {
		.flags = SPI_TRANS_USE_RXDATA,
		.addr = reg_addr,
        .length = 8 * len,
		.tx_buffer = dat,
    };
    esp_err_t err = spi_device_polling_transmit(spi_handle, &t);
    if (err!= ESP_OK) {
		ESP_LOGE(TAG, "hp303s_wr ERR %d", err);
	}
	
	hp303s_set_cs(id);	
}

static void hp303s_wr_byte(enum hp303s_id id, uint8_t reg_addr, uint8_t dat)
{
	hp303s_reset_cs(id);
	
	spi_transaction_t t = {
		.flags = SPI_TRANS_USE_RXDATA,
		.addr = reg_addr,
        .length = 8 * 1,
		.tx_buffer = &dat,
    };
    esp_err_t err = spi_device_polling_transmit(spi_handle, &t);

    if (err!= ESP_OK) {
		ESP_LOGE(TAG, "hp303s_wr ERR %d", err);
	}
	
	hp303s_set_cs(id);		
}

static void hp303s_get_coef(enum hp303s_id id)
{
	#define COEF_SZ		18
	uint8_t dat[COEF_SZ];
	
	uint8_t tmp_sta;

	do{
			vTaskDelay(pdMS_TO_TICKS(50));
			hp303s_rd(id, HP303S_REG_MEAS_CFG | HP303S_REG_RD, &tmp_sta, 1); 
	}while((tmp_sta & (1 << 7)) == 0);
	
	hp303s_rd(id, HP303S_REG_COEF | HP303S_REG_RD, dat, sizeof(dat));
	
	Hp303s_params[id].c0 = (uint16_t)dat[0] << 4 | (dat[1] & 0xf0) >> 4;
	if(Hp303s_params[id].c0 > 2047){
			Hp303s_params[id].c0 -= 4096;
	}
		 
	Hp303s_params[id].c1 = ((uint16_t)dat[1] & 0x0f) << 8 | dat[2]; 
	if(Hp303s_params[id].c1 > 2047){
			Hp303s_params[id].c1 -= 4096;
	}
	
	Hp303s_params[id].c00 = (uint32_t)dat[3] << 12 | (uint16_t)dat[4] << 4 | (dat[5] & 0xf0) >> 4;
	if(Hp303s_params[id].c00 > 524287){
			Hp303s_params[id].c00 -= 1048576;
	}
	
	Hp303s_params[id].c10 = ((uint32_t)dat[5] & 0x0f) << 16 | (uint16_t)dat[6] << 8 | dat[7];
	if(Hp303s_params[id].c10 > 524287){
			Hp303s_params[id].c10 -= 1048576;
	}
	
	Hp303s_params[id].c01 = (uint16_t)dat[8] << 8 | dat[9];
	if(Hp303s_params[id].c01 > 32767){
			Hp303s_params[id].c01 -= 65536;
	}
	
	Hp303s_params[id].c11 = (uint16_t)dat[10] << 8 | dat[11];
	if(Hp303s_params[id].c11 > 32767){
			Hp303s_params[id].c11 -= 65536;
	}
	
	Hp303s_params[id].c20 = (uint16_t)dat[12] << 8 | dat[13];
	if(Hp303s_params[id].c20 > 32767){
			Hp303s_params[id].c20 -= 65536;            
	}
	
	Hp303s_params[id].c21 = (uint16_t)dat[14] << 8 | dat[15];   
	if(Hp303s_params[id].c21 > 32767){
			Hp303s_params[id].c21 -= 65536;
	} 
	
	Hp303s_params[id].c30 = (uint16_t)dat[16] << 8 | dat[17]; 
	if(Hp303s_params[id].c30 > 32767){
			Hp303s_params[id].c30 -= 65536;
	}

	for(uint8_t i = 0;i < sizeof(dat);i++){
		ESP_LOGI(TAG, "dat[%d] = %d", i, dat[i]);
	}
	
	ESP_LOGI(TAG, "Hp303s_params[%d].c0 = %d", id, Hp303s_params[id].c0);
	ESP_LOGI(TAG, "Hp303s_params[%d].c1 = %d", id, Hp303s_params[id].c1);
	ESP_LOGI(TAG, "Hp303s_params[%d].c00 = %d", id, Hp303s_params[id].c00);
	ESP_LOGI(TAG, "Hp303s_params[%d].c10 = %d", id, Hp303s_params[id].c10);
	ESP_LOGI(TAG, "Hp303s_params[%d].c01 = %d", id, Hp303s_params[id].c01);
	ESP_LOGI(TAG, "Hp303s_params[%d].c11 = %d", id, Hp303s_params[id].c11);
	ESP_LOGI(TAG, "Hp303s_params[%d].c20 = %d", id, Hp303s_params[id].c20);
	ESP_LOGI(TAG, "Hp303s_params[%d].c21 = %d", id, Hp303s_params[id].c21);
	ESP_LOGI(TAG, "Hp303s_params[%d].c30 = %d", id, Hp303s_params[id].c30);
	ESP_LOGI(TAG, "\n");
}



static void hp303s_read()
{
	uint8_t rdy;
	uint8_t dat[3];
	int32_t tmp_raw;
	int32_t psr_raw;
	
	float tmp_tmp;
	float psr_tmp;
	
	uint8_t tmp_sta;
	uint8_t * tmp_dat;
	
	for(enum hp303s_id i = ID_0;i < ID_MAX;i++){
		
		/* 启动温度测量 */
		hp303s_wr_byte(i, HP303S_REG_MEAS_CFG | HP303S_REG_WR, 0x02);
		
		do{
			vTaskDelay(pdMS_TO_TICKS(1));
			hp303s_rd(i, HP303S_REG_MEAS_CFG | HP303S_REG_RD, &tmp_sta, 1); 
		}while((tmp_sta & (1 << 5)) == 0);


		/* 读取温度值 */
		hp303s_rd(i, HP303S_REG_TMP_B2 | HP303S_REG_RD, dat, sizeof(dat));

#if 0
		ESP_LOGI(TAG, "dat[0,1,2] = 0x%02x, 0x%02x, 0x%02x", dat[0], dat[1], dat[2]);
#endif		
		
		tmp_raw = (uint32_t)dat[0] << 16 | (uint32_t)dat[1] << 8 | dat[2];
#if 0
		ESP_LOGI(TAG, "tmp_raw 1 = %d", tmp_raw);
#endif
		
		if(tmp_raw > 8388607){
			tmp_raw -= 16777216;
		}
#if 0
		ESP_LOGI(TAG, "tmp_raw 2 = %d", tmp_raw);
#endif

		/* 启动气压测量 */
		hp303s_wr_byte(i, HP303S_REG_MEAS_CFG | HP303S_REG_WR, 0x01);
		
		do{
			vTaskDelay(pdMS_TO_TICKS(1));
			hp303s_rd(i, HP303S_REG_MEAS_CFG | HP303S_REG_RD, &tmp_sta, 1); 
		}while((tmp_sta & (1 << 4)) == 0);

		
		/* 读取温度值 */
		hp303s_rd(i, HP303S_REG_PSR_B2 | HP303S_REG_RD, dat, sizeof(dat));
		
		psr_raw = (uint32_t)dat[0] << 16 | (uint32_t)dat[1] << 8 | dat[2];
		
		if(psr_raw > 8388607){
			psr_raw -= 16777216;
		}
		
		tmp_tmp = (float)tmp_raw / kt;
		psr_tmp = (float)psr_raw / kp;
#if 0		
		ESP_LOGI(TAG, "kt = %d", kt);
		ESP_LOGI(TAG, "tmp_tmp = %d", (int)tmp_tmp);
#endif		
		
		Hp303s_params[i].temp = (double)Hp303s_params[i].c0 * 0.5 + (double)Hp303s_params[i].c1 * tmp_tmp;

		Hp303s_params[i].psr = Hp303s_params[i].c00 \
												+ psr_tmp * (Hp303s_params[i].c10 + psr_tmp * (Hp303s_params[i].c20 + psr_tmp * Hp303s_params[i].c30)) \
												+ tmp_tmp * Hp303s_params[i].c01 \
												+ tmp_tmp * psr_tmp * (Hp303s_params[i].c11 + psr_tmp * Hp303s_params[i].c21);

#if 0		
		ESP_LOGI(TAG, "Hp303s_params[%d].temp * 1000 = %d", i, (uint32_t)(Hp303s_params[i].temp * 1000));
		ESP_LOGI(TAG, "Hp303s_params[%d].psr * 1000 = %d", i, (uint32_t)(Hp303s_params[i].psr * 1000));
		
		tmp_dat = (uint8_t *)&Hp303s_params[i].temp;
		for(uint8_t j = 0;j < sizeof(float);j++){
			ESP_LOGI(TAG, "temp[%d] = 0x%02x", j, tmp_dat[j]);
		}
	
		ESP_LOGI(TAG, "\r\n");
		
		tmp_dat = (uint8_t *)&Hp303s_params[i].psr;
		for(uint8_t j = 0;j < sizeof(float);j++){
			ESP_LOGI(TAG, "psr[%d] = 0x%02x", j, tmp_dat[j]);
		}
#endif		
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

static void hp303s_selfcheck()
{
	#define SELF_CHECK_TIMES	5

	uint16_t i;
	uint16_t cnt = 0;
	enum hp303s_id j;
	float dat[ID_MAX][SELF_CHECK_TIMES];
	float dat_start[ID_MAX];
	float dat_end[ID_MAX];
	float max = 0;
	
	memset(dat_start, 0, sizeof(float) * ID_MAX);
	memset(dat_end, 0, sizeof(float) * ID_MAX);
	
	for(i = 0;i < SELF_CHECK_TIMES;i++){
		hp303s_read();
		for(j = ID_0;j < ID_MAX;j++){
			dat[j][i] = Hp303s_params[j].psr;
		}
		cnt++;
		ESP_LOGI(TAG, "times = %d", cnt);
		vTaskDelay(pdMS_TO_TICKS(30));
	}
	
	for(j = ID_0;j < ID_MAX;j++){
		ESP_LOGI(TAG, "ID_%d:", j);
		for(i = 0;i < SELF_CHECK_TIMES;i++){
			ESP_LOGI(TAG, "%.3f", dat[j][i]);
		}
		
		ESP_LOGI(TAG, "\r\n\r\n");
	}
	
	for(j = ID_0;j < ID_MAX;j++){
		for(i = 0;i < 5;i++){
			dat_start[j] += dat[j][i];
			ESP_LOGI(TAG, "dat[%d][%d] = %.3f", j, i, dat[j][i]);
			if(i != 0){
				dat_start[j] /= 2;
			}
		}
		Hp303s_params[j].psr_ori = dat_start[j];
		ESP_LOGI(TAG, "Hp303s_params[%d].psr_ori = %.3f", j, Hp303s_params[j].psr_ori);
	}
	
	for(j = ID_0;j < ID_MAX;j++){
		for(i = SELF_CHECK_TIMES - 5;i < SELF_CHECK_TIMES;i++){
			dat_end[j] += dat[j][i];
			if(i != 0){
				dat_end[j] /= 2;
			}
		}
	}
	
	for(j = ID_0;j < ID_MAX;j++){
		dat_end[j] -= dat_start[j];
	}
	
	for(j = ID_0;j < ID_MAX;j++){
		if((int)(dat_end[j] * 1000) > (int)(max * 1000)){
			max = dat_end[j];
			bad_id = j;
		}
	}
	
	ESP_LOGI(TAG, "bad_id = %d", bad_id);
	
}

static void hp303s_sample(void *pvParameters)
{
	#define SAMPLE_TIMES	10
	float dat_psr[ID_MAX][SAMPLE_TIMES];
	float dat_temp[ID_MAX][SAMPLE_TIMES];
	float dat_psr_avg[ID_MAX];
	float dat_temp_avg[ID_MAX];
	float psr_avg;
	float temp_avg;
	
	uint16_t i;
	
	while(true){
		memset(dat_psr_avg, 0, sizeof(dat_psr_avg));
		memset(dat_temp_avg, 0, sizeof(dat_temp_avg));
		
		for(i = 0;i < SAMPLE_TIMES;i++){
			hp303s_read();
			for(enum hp303s_id j = ID_0;j < ID_MAX;j++){
				dat_psr[j][i] = Hp303s_params[j].psr;
				dat_temp[j][i] = Hp303s_params[j].temp;
			}
		}
		
		for(enum hp303s_id j = ID_0;j < ID_MAX;j++){
			for(i = 0;i < SAMPLE_TIMES;i++){
				dat_psr_avg[j] += dat_psr[j][i];
				dat_temp_avg[j] += dat_temp[j][i];
				if(i != 0){
					dat_psr_avg[j] /= 2;
					dat_temp_avg[j] /= 2;
				}
			}
		}

		psr_avg = 0; temp_avg = 0;
		for(enum hp303s_id j = ID_0;j < ID_MAX;j++){

			if(j == bad_id){
				continue;
			}

			psr_avg += dat_psr_avg[j];
			temp_avg += dat_temp_avg[j];
			if((bad_id == 0 && j != 1) || (bad_id != 0 && j != 0)){
				psr_avg /= 2;
				temp_avg /= 2;
			}
		}
		
		ESP_LOGI(TAG, "psr_avg = %f", psr_avg);
		ESP_LOGI(TAG, "temp_avg = %f", temp_avg);
		//i2c_update_reg(0x01, (uint8_t *)&delta_avg);

		/* record data to protocol here  */
		Hp303s_info.psr = psr_avg;
		Hp303s_info.temp = temp_avg;
		Hp303s_info.alt = hp303s_calc_altitude(Hp303s_info.psr, Hp303s_info.temp);

		vTaskDelay(pdMS_TO_TICKS(5000));		
	}


}


void hp303s_read_id(/* enum hp303s_id id */)
{
	uint8_t prod_id;
	
	for(enum hp303s_id i = ID_0;i < ID_MAX;i++){
		hp303s_rd(i, HP303S_REG_ID | HP303S_REG_RD, &prod_id, 1);
		ESP_LOGI(TAG, "hp303s[%d] product id = 0x%02x", i, prod_id);
		
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}


void hp303s_init()
{
	uint8_t pm_rate, tmp_rate;
	uint8_t pm_prc, tmp_prc;
	
	memset(Hp303s_params, 0, sizeof(Hp303s_params));
	
	hp303s_ll_init();
	hp303s_read_id();
	
	pm_rate = 3;		/* 8 measurements per sec */
	pm_prc = 6;			/* 64 times */
	kp = k_val[pm_prc];
	
	tmp_rate = 3;		/* 8 measurements per sec */
	tmp_prc = 0;		/* 1 time */
	kt = k_val[tmp_prc];
	
	uint8_t tmp_sta;
#if 1	
	for(enum hp303s_id i = ID_0;i < ID_MAX;i++){
		hp303s_wr_byte(i, HP303S_REG_RESET | HP303S_REG_WR, 0x09);

		ESP_LOGI(TAG, "HP303S_REG_MEAS_CFG start");	
		
		do{
			vTaskDelay(pdMS_TO_TICKS(50));
			hp303s_rd(i, HP303S_REG_MEAS_CFG | HP303S_REG_RD, &tmp_sta, 1); 
		}while((tmp_sta & (1 << 6)) == 0);

		ESP_LOGI(TAG, "HP303S_REG_MEAS_CFG ok");
		
		/* 气压传感器设置reg0x06 */
		hp303s_wr_byte(i, HP303S_REG_PRS_CFG | HP303S_REG_WR, pm_rate << 4 | pm_prc);
		
		/* 温度传感器设置reg0x07 */
		ESP_LOGI(TAG, "1 << 7 | tmp_rate << 4 | tmp_prc = 0x%02x", 1 << 7 | tmp_rate << 4 | tmp_prc);
		hp303s_wr_byte(i, HP303S_REG_TMP_CFG | HP303S_REG_WR, 1 << 7 | tmp_rate << 4 | tmp_prc);
		
		/* 气压与温度结果值右对齐设置0x09 */
//		hp303s_wr_byte(i, HP303S_REG_CFG | HP303S_REG_WR, 0x0c);
		hp303s_wr_byte(i, HP303S_REG_CFG | HP303S_REG_WR, 0x04);
		
#if 1
		/* 温度补偿源设置 */
		hp303s_wr_byte(i, HP303S_REG_TMP_COEF_SRCE | HP303S_REG_WR, (1 << 7));
#endif
		
		/* 配置为停止模式reg0x08 */
		hp303s_wr_byte(i, HP303S_REG_MEAS_CFG | HP303S_REG_WR, 0x00);

		/* 获取补偿系数 */
		hp303s_get_coef(i);		
		vTaskDelay(pdMS_TO_TICKS(300));
	}
	ESP_LOGI(TAG, "hp303s init finish");
	hp303s_selfcheck();

	xTaskCreate(hp303s_sample, "hp303s", 4096, NULL, 5, NULL);
#endif
}

float hp303s_calc_altitude(float psr, float temp)
{
	
    float R = 8.314; // gas constant in J/(mol*K)
    float T = temp + 273.15; // temperature in Kelvin
    float g = 9.81; // acceleration due to gravity in m/s^2
    float P0 = 101325; // standard atmospheric pressure in Pa
    float P = psr; // atmospheric pressure at the given height
    float h; // height

#if 0
    // Input temperature in Celsius
    double tempCelsius;
    printf("Enter temperature in Celsius: ");
    scanf("%lf", &tempCelsius);

    // Convert temperature to Kelvin
    T = tempCelsius + 273.15;

    // Input atmospheric pressure at the given height
    printf("Enter atmospheric pressure at the given height in Pa: ");
    scanf("%lf", &P);
#endif
    // Calculate height using the formula
    h = (R * T / g) * log(P0 / P);

    // Output the result
    printf("The height is: %.2f meters\n", h);

	return h;
}

struct hp303s_info * hp303s_get_info()
{
	return &Hp303s_info;
}
