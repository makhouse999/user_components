
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

#include "driver/mcpwm.h"

#include "user_drv883x.h"



static const gpio_num_t drv8833_ain1 = GPIO_NUM_39;
static const gpio_num_t drv8833_ain2 = GPIO_NUM_40;
static const gpio_num_t drv8833_bin1 = GPIO_NUM_41;
static const gpio_num_t drv8833_bin2 = GPIO_NUM_42;

// MCPWM peripheral components to use
static const mcpwm_unit_t drv8833_mcpwm_unit = MCPWM_UNIT_0;
static const uint32_t drv8833_mcpwm_freq = 5000;

// Duty cycle limit: restrict duty cycle to be no greater than this value.
// Necessary when the power supply exceeds voltage rating of motor.
// TT gearmotors are typically quoted for 3-6V operation.
// Fully charged 2S LiPo is 8.4V. 6/8.4 ~= 72%.
// Feeling adventurous? Nominal 2S LiPo is 7.4V. 7.4/8.4 ~= 88%
static const uint32_t duty_cycle_max = 100;

static const char * TAG = "user_drv883x";

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, drv8833_ain1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, drv8833_ain2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, drv8833_bin1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, drv8833_bin2);
}

void set_channel(bool bBrake, int32_t iSpeed, mcpwm_timer_t timer)
{
    if (bBrake)
    {
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 100);
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 100);
    }
    else if (iSpeed > 0)
    {
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, abs(iSpeed) * duty_cycle_max / 100);
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 0);
    }
    else if (iSpeed < 0)
    {
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, abs(iSpeed) * duty_cycle_max / 100);
    }
    else
    {
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(drv8833_mcpwm_unit, timer, MCPWM_GEN_B, 0);
    }
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void motor_drv8833_task(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = drv8833_mcpwm_freq; // 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;                     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_1, &pwm_config); //Configure PWM1A & PWM1B with above settings
#if 0

    while (1)
    {
        set_channel(false, 0, MCPWM_TIMER_0);
        set_channel(false, 0, MCPWM_TIMER_1);
        vTaskDelay(5000 / portTICK_RATE_MS);

        set_channel(false, 80, MCPWM_TIMER_0);
        set_channel(false, -80, MCPWM_TIMER_1);
        vTaskDelay(5000 / portTICK_RATE_MS);

        set_channel(false, 0, MCPWM_TIMER_0);
        set_channel(false, 0, MCPWM_TIMER_1);
        vTaskDelay(5000 / portTICK_RATE_MS);

        set_channel(false, -80, MCPWM_TIMER_0);
        set_channel(false, 80, MCPWM_TIMER_1);
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
#endif
}

int drv8833_set_speed(int motor, int32_t spd)
{
	mcpwm_timer_t timer = motor == 0 ? MCPWM_TIMER_0 : MCPWM_TIMER_1;
	ESP_LOGI(TAG, "motor: %d, speed %d", motor, spd);
	set_channel(false, spd, timer);
	return 0;
}

int drv8833_init(void)
{
    printf("Testing brushed motor...\n");
    //xTaskCreate(motor_drv8833_task, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);

    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = drv8833_mcpwm_freq; // 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;                     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    mcpwm_init(drv8833_mcpwm_unit, MCPWM_TIMER_1, &pwm_config); //Configure PWM1A & PWM1B with above settings

	return 0;
}


