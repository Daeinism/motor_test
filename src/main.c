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

#define MOTOR_MOVE_TIMEOUT_MS 15000 // 15 seconds


/*|Function Prototype|-------------------------------------------------------*/
static void userInputTask(void *arg);
static int32_t degreesToEncoderCount(float inputDegrees);
static void setScaraTargetAngles(float theta1Degrees, float theta2Degrees);

/*|Variable Declaration|-----------------------------------------------------*/
// static = makes the variable private for the lifetime of the program
// volatile = "value can change unexpectedly, so it must always read it from memory, not cache it."

/*|Main|---------------------------------------------------------------------*/
void app_main(void)
{
    statusLedInit();
    encoderInit();
    motorInit(encoderGetLink1Count, encoderGetLink2Count);
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
    float link1InputDegrees;
    float link2InputDegrees;

    printf("Enter Link 1 and Link 2 angles separated by a space, or type home/release/hold:\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20)); // loop every 20ms
            continue;
        }

        /*------------------------|Simple Homing Command|--------------------------*/
        if (strncmp(inputBuffer, "home", 4) == 0) {
            encoderResetLink1Count();
            encoderResetLink2Count();
            motorSetLink1TargetCount(0);
            motorSetLink2TargetCount(0);
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

        /*------------------------|Battery Command|-------------------------------*/
        if (strncmp(inputBuffer, "battery", 7) == 0) {
            voltageReaderPrintStatus();
            continue;
        }

        if (!motorIsControlEnabled()) {
            printf("Position control is released. Type hold first.\n");
            continue;
        }

        if (sscanf(inputBuffer, "%f %f", &link1InputDegrees, &link2InputDegrees) != 2) {
            printf("Invalid input. Enter two angles, for example: -20 40\n");
            continue;
        }

        setScaraTargetAngles(link1InputDegrees, link2InputDegrees);
    }
}

static void setScaraTargetAngles(float theta1Degrees, float theta2Degrees)
{
    float link1PhysicalDegrees = theta1Degrees;
    float link2PhysicalDegrees = theta1Degrees + theta2Degrees;

    int32_t link1TargetCounts = degreesToEncoderCount(link1PhysicalDegrees);
    int32_t link2TargetCounts = degreesToEncoderCount(link2PhysicalDegrees);

    // Setting the targetcount based on home
    motorSetLink1TargetCount(link1TargetCounts);
    motorSetLink2TargetCount(link2TargetCounts);

    printf("SCARA target: theta1 %.2f degrees, theta2 %.2f degrees | Physical Link 1: %.2f degrees (%ld counts) | Physical Link 2: %.2f degrees (%ld counts)\n",
           theta1Degrees,
           theta2Degrees,
           link1PhysicalDegrees,
           (long)link1TargetCounts,
           link2PhysicalDegrees,
           (long)link2TargetCounts);

    if (motorWaitUntilTargetReached(MOTOR_MOVE_TIMEOUT_MS)) {
        printf("Movement complete\n");
    } else {
        printf("Movement did not complete: control was released, the target changed, or the movement timed out\n");
    }
}

static int32_t degreesToEncoderCount(float inputDegrees)
{
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

    return targetCounts;
}
