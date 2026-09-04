/*|SCARA-LP MK II Console| ---------------------------------------------------
#
# Project: Summer Project 2026
# Program: scaraConsole.h
#
# Description:
#   This program contains the code for the Scara Command Console.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/
#ifndef SCARA_CONSOLE_H_
#define SCARA_CONSOLE_H_

/*|Includes|-------------------------------------------------------------------*/
#include "scara.h"
#include <string.h> // strcmp,

/*|CONSTANTS|------------------------------------------------------------------*/
#define MAX_CMD 8
#define MAX_ARGS 3
#define MAX_SCARA_STRING 64

enum scaraCmd {SCARA_HOME,
                SCARA_RELEASE,
                SCARA_HOLD,
                SCARA_BATTERY,
                SCARA_FK,
                SCARA_IK,
                SET_SCARA_ANGLES,
                SCARA_WIFI_STATUS};

/*|Structures|-----------------------------------------------------------------*/
typedef struct CMD{ // Command
    const char* name; // ex) "moveScaraJ"
    int nArgs; // ex) 2
} CMD;

typedef struct SCARA_CONSOLE{ // Console
    char userInput[MAX_SCARA_STRING];
    char* command;
    char* args[MAX_ARGS + 1]; // (e.g., args[0] = "400", args[1] = "300")
    int nArgs; // number of arguments
    int cmdInd; // command index to connect with enum scaraCmd
} SCARA_CONSOLE;

/*|Function Declarations|------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole(void);
int executeScaraCommand(SCARA_CONSOLE *con);
int parseScaraCommand(SCARA_CONSOLE *con);
int validateScaraCommand(SCARA_CONSOLE *con);
int processScaraCommand(SCARA_CONSOLE *con, const char *input);

#endif /* SCARA_CONSOLE_H_ */
