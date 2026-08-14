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

#include "motor.h"
#include "statusLed.h"


/*|Macro|--------------------------------------------------------------------*/
#define LINK1_LEFT_LIMIT_GPIO GPIO_NUM_4
#define LINK1_RIGHT_LIMIT_GPIO GPIO_NUM_6
#define LIMIT_SWITCH_DEBOUNCE_MS 25

// Top Motor & Encoder 
#define ENCODER_A_GPIO GPIO_NUM_39
#define ENCODER_B_GPIO GPIO_NUM_38

// Bottom Motor & Encoder

#define ENCODER_COUNTS_PER_REVOLUTION 1320 //Full Quadrature  Reading (Bottom Motor: 1320 )


/*|Function Prototype|-------------------------------------------------------*/
static void userInputTask(void *arg);
static void limitSwitchTask(void *arg);
static void limitSwitchISR(void *arg);
static void encoderISR(void *arg);
static void encoderInit(void);
static void encoderTask(void *arg);
static float getCurrentAngle(void);
static int32_t getEncoderCount(void);

/*|Variable Declaration|-----------------------------------------------------*/
    // static = makes the variable private for the lifetime of the program
    // volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."
static volatile int32_t encoderCount = 0;
static volatile bool limitSwitchPressed = false;
static TaskHandle_t limitSafetyTaskHandle = NULL; // Create an Empty Handle (works like a container)
static volatile uint8_t previousEncoderState = 0;
static const int8_t DRAM_ATTR encoderTransitionTable[16] = { //DRAM for variables/arrays
     0, -1,  1,  0,  // transition 0~3
     1,  0,  0, -1,  // transition 4~7
    -1,  0,  0,  1,  // transition 8~11
     0,  1, -1,  0   // transition 12~15
};

/*|Newly Added|---------------------------------------------------------------*/



/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    statusLedInit();
    encoderInit();
    motorInit(getEncoderCount);

    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
    xTaskCreate(limitSwitchTask, "limitSwitchTask", 4096, NULL, 10, NULL); // Priority 10 (Higher than others)
    xTaskCreate(encoderTask, "encoderTask", 4096, NULL, 1, NULL);
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
            encoderCount = 0;
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

static void IRAM_ATTR encoderISR(void *arg) // Determine direction & Update encoder value
{ 
    // IRAM_ATTR: "Put this function inside the IRAM"
    // ISR: "Interrupt Service Routine"

    // 1. Putting A/B pin readings into one 2-digit format
    uint8_t currentState = (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);
        // reading bitwise, A is at 2nd digit, B is at 1st digit (from the right)
        // if A=1, B=1, then it reads 11
        // | sign is for combining two digits.

    // 2. Combining current & previous to make one 4-digit format (0~15 available)
    uint8_t transition = (previousEncoderState << 2) | currentState;

    // 3. Add 1, 0 ,-1 depending on the transition status according to the Table
    encoderCount += encoderTransitionTable[transition];

    // 4. Updating previous value
    previousEncoderState = currentState;

}
static void encoderInit(void) // Create encoder interrupt service
{
    // 0. Setting up the GPIO pin config for encoder wires
    gpio_config_t encoderConfig = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO),
            // 1ULL = 1 Unsigned Long Long (64bit)
            // If gpio is 9, then "Move 1 to the left 9 times" -> 000100000000
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&encoderConfig); //apply the above setup

    previousEncoderState = (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);

    /* 1. Setting the interruption condition. Options:
        POSEDGE: LOW → HIGH     
        NEGEDGE: HIGH → LOW    
        ANYEDGE: ANY
        LOW_LEVEL: while LOW
        HIGH_LEVEL: while HIGH                        */
    gpio_set_intr_type(ENCODER_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER_B_GPIO, GPIO_INTR_ANYEDGE);
    
    /* 2. Installing a service that can handle above gpio interrupt
        - installing just once is sufficient for the entire program (ESP32 firmware)
        - the public ISR service is now saved in GPIO driver internally */  
    gpio_install_isr_service(0); // 0 = default setting

    // 3. Registering encoderISR to selected GPIO pins
    gpio_isr_handler_add(ENCODER_A_GPIO, encoderISR, NULL);
    gpio_isr_handler_add(ENCODER_B_GPIO, encoderISR, NULL);
}
static float getCurrentAngle(void)
{
    int32_t currentCount = encoderCount;
    return ((float)currentCount * 360.0f) / ENCODER_COUNTS_PER_REVOLUTION;
}
static int32_t getEncoderCount(void)
{
    return encoderCount;
}
static void encoderTask(void *arg) // Prints encoder value & Angle
{
    int32_t previousCount = encoderCount;

    while (1) {
        int32_t currentCount = encoderCount;

        if (currentCount != previousCount) {
            printf("Encoder count: %ld, Angle: %.2f degrees\n", (long)currentCount, getCurrentAngle());
            previousCount = currentCount;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
