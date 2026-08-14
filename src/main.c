/*|LP-SCARA main|------------------------------------------------------------
#
# Project: Summer Project 2026
# Program: scara_main.c
#
# Description:
#  
#
# Author: Dain Kim
# Date Created: 2026-07-15
# Last Modified: 2026-08-13
# -----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdint.h> // For: Encoder
#include <stdbool.h> // For: PID
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "encoder.h"
#include "motor.h"
#include "statusLed.h"


/*|Macro|--------------------------------------------------------------------*/
#define LINK1_LEFT_LIMIT_GPIO GPIO_NUM_4
#define LINK1_RIGHT_LIMIT_GPIO GPIO_NUM_6
#define LIMIT_SWITCH_DEBOUNCE_MS 25

// Bottom Motor & Encoder

#define ENCODER_COUNTS_PER_REVOLUTION 1320 //Full Quadrature  Reading (Bottom Motor: 1320 )


/*|Function Prototype|-------------------------------------------------------*/
static void userInputTask(void *arg);
static void limitSwitchTask(void *arg);
static void limitSwitchISR(void *arg);
static void encoderPrintTask(void *arg);
static float getCurrentAngle(void);

/*|Variable Declaration|-----------------------------------------------------*/
// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static volatile bool limitSwitchPressed = false;
static TaskHandle_t limitSafetyTaskHandle = NULL; // Create an Empty Handle (works like a container)

/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    statusLedInit();
    encoderInit();
    motorInit(encoderGetCount);

    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
    xTaskCreate(limitSwitchTask, "limitSwitchTask", 4096, NULL, 10, NULL); // Priority 10 (Higher than others)
    xTaskCreate(encoderPrintTask, "encoderPrintTask", 4096, NULL, 1, NULL);
}

/*|Function Definition|------------------------------------------------------*/
static void userInputTask(void *arg) // Create targetEncoderCount from user angle input
{
    (void)arg; // Telling compiler "Don't need argument for this particular task, so don't ask"

    char inputBuffer[32];
    float inputDegrees;

    printf("Enter target angle from home in degrees, or type home/release/hold:\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20)); // loop every 20ms
            continue;
        }

        /*------------------------|Simple Homing Command|--------------------------*/
        if (strncmp(inputBuffer, "home", 4) == 0) {
            encoderResetCount();
            motorSetTargetCount(0);
            printf("Current position set as home: 0.00 degrees\n");
            continue;
        }

        /*------------------------|Simple Release Command|--------------------------*/
        if (strncmp(inputBuffer, "release", 7) == 0) {
            motorRelease();
            printf("Position control released\n");
            continue;
        }

        /*------------------------|Simple Hold Command|-----------------------------*/
        if (strncmp(inputBuffer, "hold", 4) == 0) {
            if (limitSwitchPressed) {
                printf("Cannot hold while a limit switch is pressed\n");
                continue;
            }

            motorHold();
            printf("Current position hold enabled\n");
            continue;
        }

        if (!motorIsControlEnabled()) {
            printf("Position control is released. Type hold first.\n");
            continue;
        }

        inputDegrees = strtof(inputBuffer, NULL); // save degrees in float
 
        /*-------------------------|Angle to Target Count|----------------------------*/
        // Getting the targetCounts(float) value from the input degree
        float targetCountsFloat = inputDegrees * ENCODER_COUNTS_PER_REVOLUTION / 360.0f;
         
        // Round the float value to the nearest integer.
        int32_t targetCounts;

        // adding 0.5 or -0.5 before truncating into int for rounding
        if (targetCountsFloat >= 0.0f) { 
            targetCounts = (int32_t)(targetCountsFloat + 0.5f);
        } 
        else {
            targetCounts = (int32_t)(targetCountsFloat - 0.5f);
        }

        // Setting the targetcount based on home
        motorSetTargetCount(targetCounts);

        printf("Target angle: %.2f degrees (%ld counts)\n",
               inputDegrees,
               (long)targetCounts);
    }
}

static void limitSwitchTask(void *arg) 
{
    (void)arg;

    limitSafetyTaskHandle = xTaskGetCurrentTaskHandle(); // Put the current task's handle into the container
        // "Give me the handle for this task, so I can refer to it later"

    // 0. setting up the gpio configuation ------------------------------------------------
    gpio_config_t switchConfig = {
        .pin_bit_mask =
            (1ULL << LINK1_LEFT_LIMIT_GPIO) |
            (1ULL << LINK1_RIGHT_LIMIT_GPIO),
            // << means, moving that 1(ON) sign to the left multiple times (# of gpio number)
            // bit mask is used because same config can also be applied to multiple gpio if wanted
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE // Any change will trigger the interrupt  
    };

    gpio_config(&switchConfig); // applying the above gpio configuration

    // 1. Detecting the change in limit switch state --------------------------------------
        // (1 = Unpressed as default state)
    int previousLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO); //get_level to get the value
    int previousRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

    limitSwitchPressed = (previousLeftState == 0) || (previousRightState == 0);
        // if either previous state is 0, then it means the limit switch is newly pressed

    // Connecting / Registering the GPIOs to ISR
    gpio_isr_handler_add(LINK1_LEFT_LIMIT_GPIO, limitSwitchISR, NULL);
    gpio_isr_handler_add(LINK1_RIGHT_LIMIT_GPIO, limitSwitchISR, NULL);

    if (limitSwitchPressed) {
        xTaskNotifyGive(limitSafetyTaskHandle);
    } 
        /*“If a limit switch is already pressed when this task starts, give the task 
        one notification so the first loop iteration checks and stops the motors.”*/

    while (1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Run the rest of the loop ONLY IF the notify is taken.
        /* pd = Portable Definition
        pdTRUE: Clear all pending notifications after waking up.
        pdFALSE: Consume one notification at a time.
        portMAX_DELAY: Wait indefinitely until a notification is received.
        */

        while (1) {
            int currentLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            int currentRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

            if ((currentLeftState == 0) || (currentRightState == 0) || limitSwitchPressed) {
                limitSwitchPressed = true;
                motorEmergencyStop();
            }

            if ((currentLeftState == 0) && (previousLeftState != 0)) {
                printf("Link 1 Left limit switch PRESSED\n");
                previousLeftState = 0;
            }

            if ((currentRightState == 0) && (previousRightState != 0)) {
                printf("Link 1 Right limit switch PRESSED\n");
                previousRightState = 0;
            }

            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LIMIT_SWITCH_DEBOUNCE_MS)) != 0) {
                continue;
            }

            currentLeftState = gpio_get_level(LINK1_LEFT_LIMIT_GPIO);
            currentRightState = gpio_get_level(LINK1_RIGHT_LIMIT_GPIO);

            if ((currentLeftState != previousLeftState) && (currentLeftState != 0)) {
                printf("Link 1 Left limit switch RELEASED\n");
                previousLeftState = currentLeftState;
            }

            if ((currentRightState != previousRightState) && (currentRightState != 0)) {
                printf("Link 1 Right limit switch RELEASED\n");
                previousRightState = currentRightState;
            }

            limitSwitchPressed = (currentLeftState == 0) || (currentRightState == 0);
                // 0 = button is pressed

            break;
        }
    }
}
static void IRAM_ATTR limitSwitchISR(void *arg)
{
    (void)arg;

    BaseType_t higherPriorityTaskWoken = pdFALSE; //pdFALSE = FALSE (no other meaning)
        // "Initially, no higher-priority task has been woken"

    bool anyLimitSwitchPressed =
        (gpio_get_level(LINK1_LEFT_LIMIT_GPIO) == 0) ||
        (gpio_get_level(LINK1_RIGHT_LIMIT_GPIO) == 0);

    if (anyLimitSwitchPressed) {
        limitSwitchPressed = true;
        motorDisableControlFromISR();
    }

    if (limitSafetyTaskHandle != NULL) { // If the handle has been assigned to a valid task
        vTaskNotifyGiveFromISR(
            limitSafetyTaskHandle, // "Give the notification to the task that is waiting for it"
            &higherPriorityTaskWoken /*"If the task that is waiting for the notification 
            has a higher priority than the currently running task, set this variable to pdTRUE"*/
        ); 
    }

    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // Immediately switch to the higher-priority task after the ISR
    }
}

static float getCurrentAngle(void)
{
    int32_t currentCount = encoderGetCount();
    return ((float)currentCount * 360.0f) / ENCODER_COUNTS_PER_REVOLUTION;
}

static void encoderPrintTask(void *arg) // Prints encoder value & Angle
{
    (void)arg;

    int32_t previousCount = encoderGetCount();

    while (1) {
        int32_t currentCount = encoderGetCount();

        if (currentCount != previousCount) {
            printf("Encoder count: %ld, Angle: %.2f degrees\n", (long)currentCount, getCurrentAngle());
            previousCount = currentCount;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
