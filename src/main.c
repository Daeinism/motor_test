#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h" //this is for PWM (NOT necessarily for LED)


/*|Macro|--------------------------------------------------------------------*/
#define TEST_LED_GPIO GPIO_NUM_36
#define LIMIT_SWITCH_GPIO GPIO_NUM_42

#define MOTOR1_IN1 GPIO_NUM_11
#define MOTOR1_IN2 GPIO_NUM_12
#define MOTOR2_IN1 GPIO_NUM_9
#define MOTOR2_IN2 GPIO_NUM_10

#define MOTOR_MAX_DUTY 1023


/*|Function Prototype|-------------------------------------------------------*/
static void gpioInit(void);
static void motorPwmInit(void);
static void ledTask(void *arg);
static void motorTask(void *arg);
static void userInputTask(void *arg);
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty);
static void limitSwitchTask(void *arg);

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
    xTaskCreate(limitSwitchTask, "limitSwitchTask", 2048, NULL, 5, NULL);
}

/*|Function Definition|------------------------------------------------------*/
static void gpioInit(void)
{
    gpio_reset_pin(TEST_LED_GPIO);
    gpio_set_direction(TEST_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TEST_LED_GPIO, 0);
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

    ledc_channel_config_t channels[] = {
        {
            .gpio_num = MOTOR1_IN1,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR1_IN2,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR2_IN1,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_2,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = MOTOR2_IN2,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_3,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        }
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]); //applying the above configuration for all 4 channels
    }
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

    int previousDuty = 1;

    while (1) {
        int requestedDuty = motorDuty; // making a snap shot of motorDuty to prevent potential error

        if (requestedDuty != previousDuty) {
            /* Stop briefly before changing speed or direction. */
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, 0);
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, 0);
            vTaskDelay(pdMS_TO_TICKS(10));

            /* And then start moving again with new duty*/
            setMotorDuty(LEDC_CHANNEL_0, LEDC_CHANNEL_1, requestedDuty);
            setMotorDuty(LEDC_CHANNEL_2, LEDC_CHANNEL_3, requestedDuty);
            previousDuty = requestedDuty;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static void setMotorDuty(ledc_channel_t in1Channel, ledc_channel_t in2Channel, int signedDuty)
{
    // 0. Declaring duty values (initially 0)
    uint32_t in1Duty = 0; //unsigned int 32bit type (required by LEDC)
    uint32_t in2Duty = 0;

    // 1. Capping to max duty (1023)
    if (signedDuty > MOTOR_MAX_DUTY) {
        signedDuty = MOTOR_MAX_DUTY;
    } else if (signedDuty < -MOTOR_MAX_DUTY) {
        signedDuty = -MOTOR_MAX_DUTY;
    }

    // 2. Determining the direction
    if (signedDuty > 0) {
        in1Duty = (uint32_t)signedDuty;
    } else if (signedDuty < 0) {
        in2Duty = (uint32_t)(-signedDuty);
    }

    // 3. Updating the duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, in1Channel, in1Duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, in1Channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, in2Channel, in2Duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, in2Channel);
}
static void userInputTask(void *arg)
{
    (void)arg; // Telling compiler "Don't need argument for this particular task, so don't ask"

    char inputBuffer[32];
    int inputDuty;

    printf("Enter duty from -1023 to 1023:\n");
    printf("Positive = forward, negative = reverse, 0 = stop\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        inputDuty = strtol(inputBuffer, NULL, 10); //strtol = string to long integer

        // Capping to max duty
        if (inputDuty > MOTOR_MAX_DUTY) {
            inputDuty = MOTOR_MAX_DUTY;
        } else if (inputDuty < -MOTOR_MAX_DUTY) {
            inputDuty = -MOTOR_MAX_DUTY;
        }

        // Updating duty
        motorDuty = inputDuty;

        printf("Motor duty set to %d (%s)\n",
               motorDuty,
               motorDuty > 0 ? "forward" :
               motorDuty < 0 ? "reverse" : "stop");
    }
}
static void limitSwitchTask(void *arg)
{
    // 0. setting up the gpio configuation
    gpio_config_t switchConfig = {
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_GPIO), 
            // << means, moving that 1(ON) sign to the left multiple times (# of gpio number)
            // bit mask is used because same config can also be applied to multiple gpio if wanted
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // setting interrupt function OFF
    };

    gpio_config(&switchConfig); // applying the above gpio configuration

    // 1. Detecting the change in limit switch state
    int previousState = gpio_get_level(LIMIT_SWITCH_GPIO); //get_level to get the value

    while (1) {
        int currentState = gpio_get_level(LIMIT_SWITCH_GPIO);

        if (currentState != previousState) {
            if (currentState == 0) {
                printf("Limit switch PRESSED\n");
            } else {
                printf("Limit switch RELEASED\n");
            }

            previousState = currentState;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}