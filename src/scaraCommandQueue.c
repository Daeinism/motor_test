/*|SCARA-LP MK II Command Queue| ---------------------------------------------
#
# Project: Summer Project 2026
# Program: scaraCommandQueue.c
#
# Description:
#   This program contains the command queue and command executor for the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scaraCommandQueue.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "scaraConsole.h"

/*|CONSTANTS|------------------------------------------------------------------*/
#define SCARA_COMMAND_QUEUE_LENGTH 10
#define SCARA_COMMAND_EXECUTOR_STACK_SIZE 4096
#define SCARA_COMMAND_EXECUTOR_PRIORITY 1

/*|Structures|-----------------------------------------------------------------*/
typedef struct SCARA_COMMAND_MESSAGE {
    char text[MAX_SCARA_STRING];
} SCARA_COMMAND_MESSAGE;

/*|Function Prototype|---------------------------------------------------------*/
static void commandExecutorTask(void *arg);

/*|Variable Declaration|-------------------------------------------------------*/
static QueueHandle_t commandQueue = NULL;

/*|Function Definitions|-------------------------------------------------------*/
bool scaraCommandQueueInit(void)
{
    // The queue copies each complete command string into its own storage.
    commandQueue = xQueueCreate(SCARA_COMMAND_QUEUE_LENGTH, sizeof(SCARA_COMMAND_MESSAGE));
    if (commandQueue == NULL) {
        return false;
    }

    // Only this task parses and executes commands, so robot commands cannot overlap.
    if (xTaskCreate(commandExecutorTask,
                    "commandExecutorTask",
                    SCARA_COMMAND_EXECUTOR_STACK_SIZE,
                    NULL,
                    SCARA_COMMAND_EXECUTOR_PRIORITY,
                    NULL) != pdPASS) {
        vQueueDelete(commandQueue);
        commandQueue = NULL;
        return false;
    }

    return true;
}

bool scaraCommandQueueSend(const char *command)
{
    if (commandQueue == NULL || command == NULL) {
        return false;
    }

    SCARA_COMMAND_MESSAGE message;
    size_t commandLength = strlen(command);

    // Reject the command instead of executing a silently truncated command.
    if (commandLength >= sizeof(message.text)) {
        return false;
    }

    memcpy(message.text, command, commandLength + 1);

    // Do not block an input task when all queue slots are already occupied.
    return xQueueSend(commandQueue, &message, 0) == pdPASS;
}

static void commandExecutorTask(void *arg)
{
    (void)arg;

    SCARA_CONSOLE console = initScaraConsole();
    SCARA_COMMAND_MESSAGE message;

    while (1) {
        // Wait without consuming CPU until an input task submits a command.
        if (xQueueReceive(commandQueue, &message, portMAX_DELAY) == pdPASS) {
            processScaraCommand(&console, message.text);
        }
    }
}
