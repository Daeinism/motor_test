/*|ROBT 1270: SCARA Console| --------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Advanced Control
# Program: scaraConsole.cpp
#
# Description:
#   This porogram contains the code for the Scara Command Console.
#
# Author: Dain Kim
# Date Created: 2026-04-22
# Last Modified: 2026-05-04
# -----------------------------------------------------------------------------*/
#ifndef SCARA_CONSOLE_H_
#define SCARA_CONSOLE_H_

/*|Includes|-------------------------------------------------------------------*/
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.
#include "scara.h"
#include <math.h>
#include <string.h> // strcmp,

/*|CONSTANTS|------------------------------------------------------------------*/
#define MAX_CMD 9
#define MAX_ARGS 5
#define MAX_SCARA_STRING 50
#define MAX_COLOR 255
#define MIN_COLOR 0
#define MAX_COORD 600.0
#define MIN_COORD -600.0
#define MAX_POINTS 50
#define MIN_POINTS 1

enum scaraCmd {MOVE_SCARA_J, 
                MOVE_SCARA_L, 
                SCARA_PEN_UP, 
                SCARA_PEN_DOWN, 
                SCARA_SPEED, 
                SCARA_PEN_COLOR,
                CLEAR_TRACE, 
                ROTATE_JOINTS,
                QUIT};

/*|Structures|-----------------------------------------------------------------*/
struct CMD{ // Command
    const char* name; // ex) "moveScaraJ"
    const int nArgs; // ex) 2
};

struct SCARA_CONSOLE{ // Console
    char userInput[MAX_SCARA_STRING];
    SCARA_ROBOT scaraRobot;
    char* command;
    char* args[MAX_ARGS + 1]; // (e.g., args[0] = "400", args[1] = "300")
    int nArgs; // number of arguments
    int cmdInd; // command index to connect with enum scaraCmd
};

/*|Function Declarations|------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole(void);
void readScaraConsole(SCARA_CONSOLE *con);
void executeScaraCommand(SCARA_CONSOLE *con);
int parseScaraCommand(SCARA_CONSOLE *con);
int validateScaraCommand(SCARA_CONSOLE *con);

#endif /* SCARA_CONSOLE_H_ */