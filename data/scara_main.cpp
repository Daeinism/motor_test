/*|SCARA Simulator|------------------------------------------------------------
#
# Project: ROBT 1270 - Lab #9 - Command Console For the SCARA
# Program: scara_main.c
#
# Description:
# - This lab will build upon the Lab 8 and introduce a command console for the SCARA simulator. 
# 	This console will allow users to manually type commands which will be interpreted 
# 	by a command parser, validated, and executed.
#
# Other Programs:
#   ScaraRobotSimulator.exe (Version 4.3)
#
# Parsable simulator Commands:
#  - PEN_UP
#  - PEN_DOWN
#  - PEN_COLOR <r> <g> <b>
#  - CLEAR_TRACE
#  - MOTOR_SPEED HIGH/MEDIUM/LOW
#  - MOVESCARAJ
#  - MOVESCARAL
#  - QUIT
#
# Commands NOT included for parsing: 
#  - CYCLE_PEN_COLORS ON/OFF
#  - ROTATE_JOINT ANG1 <deg1> ANG2 <deg2>
#  - CLEAR_REMOTE_COMMAND_LOG
#  - CLEAR_POSITION_LOG
#  - SHUTDOWN_SIMULATION
#  - MESSAGE <"string">
#  - HOME
#  - END
#
#
# Other Information:
#  - IP Address: 127.0.0.1 Port 1270
#  - BCIT Blue: 10 64 109
#  - If using VS Code, add the following args to tasks.json g++ build task.
#     "-std=c++14"
#		"-lwsock32"
#		"-Wno-deprecated"
#  - Also change the "${file}" argument to "*.cpp". This is a .cpp wildcard
#  - that will grab other .cpp files in the folder.
#
# Author: Dain Kim
# Date Created: April 22nd 2026
# Last Modified: May 4th 2026
# -----------------------------------------------------------------------------*/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>  // <list of functions used>
#include <string.h>
#include <math.h>   // <list of functions used>
#include <time.h>   // <list of functions used>
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.

#include "scara.h"
#include "scaraConsole.h"

/*|Globals|--------------------------------------------------------------------*/
extern CRobot robot;     // the global robot Class instance.  Can be used everywhere
						// robot.Initialize()
						// robot.Send()
						// robot.Close()

/*|CONSTANTS|------------------------------------------------------------------*/
#define TEST                  0
#define MIN_ERROR             0.01

/*|main|-----------------------------------------------------------------------
#
# Description:
#   - 
#   - 
#   - 
#   - 
#
# Inputs:
#   - void
#
# Returns: 
#   - int (0 upon successful termination)
#
# Last Modified: May 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {

	if (argc > 1) { // Playback Mode: Validate extension and open

        if (strstr(argv[1], ".slog") != NULL) { // if ".slog" is within the file name
            FILE* file = fopen(argv[1], "r"); // initialize & open file in reading mode
            // TODO: Read lines and execute

            if(file) fclose(file);
        } else {
            return 1; // End program if invalid extension
        }
    } else { // Record Mode: Generate unique name[cite: 1]
        
        char filename[50];
        time_t now = time(NULL);
        strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S.slog", localtime(&now));
        
        FILE* file = fopen(filename, "w");
        // TODO: Get user input and record commands[cite: 1]
        if(file) fclose(file);
    }



	// Initialization & Declaration
	SCARA_CONSOLE console = initScaraConsole(); 
	unsigned char exit = 0;
	unsigned char isValid = 0;

	while(!exit){
		
		// 1. Read & Parse 
		readScaraConsole(&console); //make sure to use & sign
		parseScaraCommand(&console);

		// 2. Check validity
		isValid = validateScaraCommand(&console);

		// 3. Execute if valid
		if (isValid == 1){
			executeScaraCommand(&console);
		}
			
		// 4. Exit if exit is requested
		if (console.cmdInd == QUIT){
			exit = 1;
		}
	} 

   robot.Close(); // close remote connection

   return 0;
}

/*|Notes|----------------------------------------------------------------------
- Pointer related
	- when giving a function with a varible as a parameter but want to use it 
	as a pointer, don't forget to use the address symbol
		ex) readScaraConsole(&console);

	- the difference between struct.member VS struct->member
		- struct.member - is used when struct is a regular varialbe
		- struct->member - is used when struct is a pointer varialble
		- both of them point to a regular variable "member" nonetheless.

- Comparing string
	- you cannot just use == for check if the string and targeted words are matching
	- you MUST use strcmp to do the comparison!!
	ex) Instead of using   `if (con->args[0] == "H")`
		use `if (strcmp(con->args[0], "H")==0)`

*/