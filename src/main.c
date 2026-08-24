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
# Last Modified: 2026-08-24
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
#include "limitSwitch.h"
#include "motor.h"
#include "statusLed.h"
#include "voltageReader.h"


/*|Function Prototype|-------------------------------------------------------*/
static void userInputTask(void *arg);

/*|Variable Declaration|-----------------------------------------------------*/
// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."

/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    statusLedInit();
    encoderInit();
    motorInit(encoderGetLink1Count);
    limitSwitchInit(motorEmergencyStop);
        // motorEmergencyStop is just a function pointer, not a function call.
    voltageReaderInit();

    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
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
            encoderResetLink1Count();
            encoderResetLink2Count();
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
            if (limitSwitchAnyIsPressed()) {
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
