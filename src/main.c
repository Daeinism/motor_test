#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h" //this is for PWM (NOT necessarily for LED)


/*|Macro|--------------------------------------------------------------------*/
#define TEST_LED_GPIO GPIO_NUM_36
#define MOTOR1_IN1 GPIO_NUM_11
#define MOTOR1_IN2 GPIO_NUM_12
#define MOTOR2_IN1 GPIO_NUM_9
#define MOTOR2_IN2 GPIO_NUM_10


/*|Function Prototype|-------------------------------------------------------*/
static void gpioInit(void);
static void motorPwmInit(void);
static void ledTask(void *arg);
static void motorTask(void *arg);
static void userInputTask(void *arg);

static volatile int motorDuty = 0; 
// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."

/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    gpioInit();
    motorPwmInit();

    xTaskCreate(ledTask, "ledTask", 2048, NULL, 1, NULL);
    xTaskCreate(motorTask, "motorTask", 2048, NULL, 1, NULL);
    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
}


/*|Function Definition|------------------------------------------------------*/
static void gpioInit(void)
{
    gpio_reset_pin(TEST_LED_GPIO);
    gpio_set_direction(TEST_LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(MOTOR1_IN2);
    gpio_set_direction(MOTOR1_IN2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR2_IN2);
    gpio_set_direction(MOTOR2_IN2, GPIO_MODE_OUTPUT);

    /* Keep the motor stopped during initialization */
    gpio_set_level(MOTOR1_IN2, 0);
    gpio_set_level(MOTOR2_IN2, 0);
}
static void motorPwmInit(void) 
{

    ledc_timer_config_t timer = { //configuration setting (won't need timer for anything else)
        .speed_mode = LEDC_LOW_SPEED_MODE, //default for ESP32 S3 Hardware (no need change) 
        .timer_num = LEDC_TIMER_0, // ESP32 S3 has 4 LEDC hardware timer & 8 Channels
        .duty_resolution = LEDC_TIMER_10_BIT, //get up to 1024 possible duty value
        .freq_hz = 20000, // common choice for DC motor
        .clk_cfg = LEDC_AUTO_CLK // (default) the driver picks clock source automatically 
    };

    ledc_timer_config(&timer); // pass the data to the function that programs the hardware

    ledc_channel_config_t channel1 = {
        .gpio_num = MOTOR1_IN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, // 1 0f 8 available channels
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // proportion of how long it is HIGH vs LOW per cycle (400/1024)
        .hpoint = 0 // leave it (advanced setting)
    };
    ledc_channel_config_t channel2 = {
        .gpio_num = MOTOR2_IN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1, // No.2 of 8 available channels
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel1);
    ledc_channel_config(&channel2);
}
static void ledTask(void *arg)
{
    while (1) 
    {
        gpio_set_level(TEST_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(TEST_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void motorTask(void *arg)
{
    (void)arg; // telling compiler "Yes, we are not using the arguments. Stop asking."

    while (1) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, motorDuty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, motorDuty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void userInputTask(void *arg)
{
    (void)arg;

    char inputBuffer[32];
    int inputDuty;

    printf("Enter duty from 0 to 1023:\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        inputDuty = strtol(inputBuffer, NULL, 10); //strtol = string to long integer

        if (inputDuty < 0) {
            inputDuty = 0;
        }

        if (inputDuty > 1023) {
            inputDuty = 1023;
        }

        motorDuty = inputDuty;

        printf("Duty set to %d\n", motorDuty);
    }
}