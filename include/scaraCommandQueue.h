/*|SCARA-LP MK II Command Queue| ---------------------------------------------
#
# Project: Summer Project 2026
# Program: scaraCommandQueue.h
#
# Description:
#   This program contains the command queue interface for the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/
#ifndef SCARA_COMMAND_QUEUE_H_
#define SCARA_COMMAND_QUEUE_H_

#include <stdbool.h>

typedef enum SCARA_COMMAND_SOURCE {
    SCARA_COMMAND_SOURCE_USB,
    SCARA_COMMAND_SOURCE_TCP
} SCARA_COMMAND_SOURCE;

typedef enum SCARA_COMMAND_RESULT {
    SCARA_COMMAND_RESULT_DONE,
    SCARA_COMMAND_RESULT_ERROR
} SCARA_COMMAND_RESULT;

bool scaraCommandQueueInit(void);
bool scaraCommandQueueSend(const char *command, SCARA_COMMAND_SOURCE source);
bool scaraCommandResultReceive(SCARA_COMMAND_RESULT *result);

#endif /* SCARA_COMMAND_QUEUE_H_ */
