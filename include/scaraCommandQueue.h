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

bool scaraCommandQueueInit(void);
bool scaraCommandQueueSend(const char *command);

#endif /* SCARA_COMMAND_QUEUE_H_ */
