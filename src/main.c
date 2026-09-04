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
# Last Modified: 2026-09-04
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
#include "scara.h"
#include "scaraCommandQueue.h"
#include "scaraConsole.h"
#include "scaraMotion.h"
#include "statusLed.h"
#include "voltageReader.h"
#include "wifiManager.h"


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
    motorInit(encoderGetLink1Count, encoderGetLink2Count);
    limitSwitchInit(motorEmergencyStop);
        // motorEmergencyStop is just a function pointer, not a function call.
    voltageReaderInit();

    if (!wifiManagerInit()) {
        printf("Wi-Fi manager is unavailable\n");
    }

    if (!scaraCommandQueueInit()) {
        printf("Failed to initialize SCARA command queue\n");
        return;
    }

    xTaskCreate(userInputTask, "userInputTask", 4096, NULL, 1, NULL);
}

/*|Function Definition|------------------------------------------------------*/
static void userInputTask(void *arg) // Create targetEncoderCount from user angle input
{
    (void)arg; // Telling compiler "Don't need argument for this particular task, so don't ask"

    char inputBuffer[MAX_SCARA_STRING];
    printf("Enter Link 1 and Link 2 angles separated by a space, or type home/release/hold:\n");

    while (1) {
        if (fgets(inputBuffer, sizeof(inputBuffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20)); // loop every 20ms
            continue;
        }

        if (!scaraCommandQueueSend(inputBuffer)) {
            printf("Command rejected: queue is full or the command is too long\n");
        }
    }
}
