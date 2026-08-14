#include "statusLed.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#define RED_LED_GPIO GPIO_NUM_12 //RED LED
#define GREEN_LED_GPIO GPIO_NUM_11 //GREEN LED
#define LED_ON_DUTY 500 // max brightness is 1023
#define LED_OFF_DUTY 0

static void gpioInit(void);
static void setLedDuty(ledc_channel_t channel, uint32_t duty);
static void redLedTask(void *arg);
static void greenLedTask(void *arg);

void statusLedInit(void)
{
    gpioInit();
    xTaskCreate(redLedTask, "redLedTask", 2048, NULL, 1, NULL);
    xTaskCreate(greenLedTask, "greenLedTask", 2048, NULL, 1, NULL);
}

static void gpioInit(void) // Initializing simple gpios
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t redChannel = {
        .gpio_num = RED_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = LED_OFF_DUTY,
        .hpoint = 0
    };

    ledc_channel_config_t greenChannel = {
        .gpio_num = GREEN_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = LED_OFF_DUTY,
        .hpoint = 0
    };

    ledc_channel_config(&redChannel);
    ledc_channel_config(&greenChannel);
}

static void setLedDuty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void redLedTask(void *arg) // Simple LED task for test
{
    while (1)
    {
        setLedDuty(LEDC_CHANNEL_4, LED_ON_DUTY);
        vTaskDelay(pdMS_TO_TICKS(500));

        setLedDuty(LEDC_CHANNEL_4, LED_OFF_DUTY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void greenLedTask(void *arg)
{
    int currentDuty = LED_OFF_DUTY;
    int dutyChange = 1;

    while (1)
    {
        setLedDuty(LEDC_CHANNEL_5, (uint32_t)currentDuty);

        if (currentDuty >= LED_ON_DUTY) {
            dutyChange = -1;
        } else if (currentDuty <= LED_OFF_DUTY) {
            dutyChange = 1;
        }

        currentDuty += dutyChange;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
