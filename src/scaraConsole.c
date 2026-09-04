/*|SCARA-LP MK II Console| ---------------------------------------------------
#
# Project: Summer Project 2026
# Program: scaraConsole.c
#
# Description:
#   This program contains the code for the Scara Command Console.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scaraConsole.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "encoder.h"
#include "limitSwitch.h"
#include "motor.h"
#include "scaraMotion.h"
#include "voltageReader.h"
#include "wifiManager.h"

/*|Global Variables|-----------------------------------------------------------*/
CMD scaraCommands[MAX_CMD] = { // format: {"commandName", number of arguments}
    {"home", 0},
    {"release", 0},
    {"hold", 0},
    {"battery", 0},
    {"fk", 2},
    {"ik", 3},
    {"setScaraAngles", 2},
    {"wifiStatus", 0}
};

static int parseDoubleArgument(const char *text, double *value);
static int parseIntArgument(const char *text, int *value);

/*|Function Definitions|-------------------------------------------------------*/
/*|initScaraConsole|------------------------------------------------------------
#
# Description:
#   - Initializes the SCARA console environment.
#
# Inputs:
#   - void
#
# Returns:
#   - SCARA_CONSOLE (The initialized console structure)
#
# Last Modified: September 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole(void){
    // Variable Declarations
    SCARA_CONSOLE con = {0};
    con.cmdInd = -1;

    return con;
}

/*|parseScaraCommand|-----------------------------------------------------------
#
# Description:
#   - Breaks down the raw user input string into a command and individual arguments.
#   - Uses delimiters (space, comma, newline) to tokenize the input string.
#   - Resets and counts the number of arguments found, up to the maximum limit.
#
# Inputs:
#   - SCARA_CONSOLE* con (Pointer to the console structure to be updated)
#
# Returns:
#   - int (The total number of arguments successfully parsed)
#
# Last Modified: May 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int parseScaraCommand(SCARA_CONSOLE* con){

    // 1. re-initializing the number of Arguments for safety
    con->nArgs = 0;

    // 2. parse the first section
    con->command = strtok(con->userInput, " ,\n\r");

    if (con->command != NULL) {

        // utilizing token for parsing the rest of the arguments
        char* token = strtok(NULL, " ,\n\r");

        // only if token exists and not more than the max number (currently 3) of arguments
        while (token != NULL && con->nArgs <= MAX_ARGS) {

            // updating the argument content
            con->args[con->nArgs] = token;
            // add 1 to the nArgs before loop to the next parsing
            con->nArgs++;
            token = strtok(NULL, " ,\n\r");
        }
    }

    return con->nArgs; //this will be used to validate later
}

/*|validateScaraCommand|--------------------------------------------------------
#
# Description:
#   - Validates the parsed command against the list of available commands.
#   - Verifies that the correct number of arguments was provided for the command.
#   - Performs range checking on specific arguments.
#
# Inputs:
#   - SCARA_CONSOLE *con (Pointer to the console structure to validate)
#
# Returns:
#   - 1 (Success: Command and arguments are valid)
#   - 0 (Error: Command not found, wrong argument count, or out-of-range value)
#
# Last Modified: September 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int validateScaraCommand(SCARA_CONSOLE *con){

    // 1. Safety check
    if (con->command == NULL) {
        return 0;
    }

    con->cmdInd = -1; // as the default starting value for safety check
    // 2. loop through commands to find the right match
    for (int i = 0; i < MAX_CMD; i++){

        if(strcmp(con->command, scaraCommands[i].name ) == 0) {

            if(con->nArgs == scaraCommands[i].nArgs) {
                con->cmdInd = i;
                break; // move on to the step number 4
            }
            else {
                printf("Error: Expected %d arguments, received %d\n", scaraCommands[i].nArgs, con->nArgs);
                return 0; // fail
            }
        }
    }

    // 3. if no matching command, return "fail"
    if (con->cmdInd == -1){ //if no matching command is found
        printf("Error: Command '%s' not found.\n", con->command);
        return 0; // fail
    }

    // 4. Validation specific for each command
    switch(con->cmdInd) {
        case SCARA_FK:
        case SET_SCARA_ANGLES:
        {
            double theta1;
            double theta2;

            if (!parseDoubleArgument(con->args[0], &theta1) ||
                !parseDoubleArgument(con->args[1], &theta2)) {
                printf("Error: angle arguments must be valid numbers.\n");
                return 0;
            }

            if (theta1 < -MAX_ABS_THETA1_DEG || theta1 > MAX_ABS_THETA1_DEG ||
                theta2 < -MAX_ABS_THETA2_DEG || theta2 > MAX_ABS_THETA2_DEG) {
                printf("Error: invalid argument!\n");
                return 0;
            }
            break;
        }
        case SCARA_IK:
        {
            double toolX;
            double toolY;
            int armSolution;

            if (!parseDoubleArgument(con->args[0], &toolX) ||
                !parseDoubleArgument(con->args[1], &toolY) ||
                !parseIntArgument(con->args[2], &armSolution)) {
                printf("Error: IK arguments must be valid numbers.\n");
                return 0;
            }

            if (armSolution != RIGHT_ARM_SOLUTION && armSolution != LEFT_ARM_SOLUTION) {
                printf("Error: arm solution must be 0 (right) or 1 (left)\n");
                return 0;
            }
            break;
        }
        case SCARA_HOME:
        case SCARA_RELEASE:
        case SCARA_HOLD:
        case SCARA_BATTERY:
        case SCARA_WIFI_STATUS:
            // nothing to check
            break;
    }

    return 1; // otherwise, success
}

/*|executeScaraCommand|---------------------------------------------------------
#
# Description:
#   - Maps validated commands to their respective logic or lower-level functions.
#   - Handles data conversion (atof/atoi) for arguments before execution.
#   - Updates the physical robot through its lower-level functions.
#
# Inputs:
#   - SCARA_CONSOLE* con (Pointer to the console structure containing the command)
#
# Returns:
#   - 1 (Success)
#   - 0 (Execution failed)
#
# Last Modified: September 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int executeScaraCommand(SCARA_CONSOLE* con){
    switch(con->cmdInd){
        /*------------------------|Simple Homing Command|--------------------------*/
        case SCARA_HOME:
            encoderResetLink1Count();
            encoderResetLink2Count();
            motorSetLink1TargetCount(0);
            motorSetLink2TargetCount(0);
            printf("Current position set as home: 0.00 degrees\n");
            return 1;

        /*------------------------|Simple Release Command|--------------------------*/
        case SCARA_RELEASE:
            motorRelease();
            printf("Position control released\n");
            return 1;

        /*------------------------|Simple Hold Command|-----------------------------*/
        case SCARA_HOLD:
            if (limitSwitchAnyIsPressed()) {
                printf("Cannot hold while a limit switch is pressed\n");
                return 0;
            }

            motorHold();
            printf("Current position hold enabled\n");
            return 1;

        /*------------------------|Battery Command|-------------------------------*/
        case SCARA_BATTERY:
            voltageReaderPrintStatus();
            return 1;

        /*------------------------|Wi-Fi Status Command|---------------------------*/
        case SCARA_WIFI_STATUS:
            wifiManagerPrintStatus();
            return 1;

        /*------------------------|Kinematics Command|-----------------------------*/
        case SCARA_FK:
        {
            double toolX;
            double toolY;
            double theta1 = atof(con->args[0]);
            double theta2 = atof(con->args[1]);

            if (scaraFK(theta1, theta2, &toolX, &toolY) == 0) {
                printf("FK result: X %.2f mm, Y %.2f mm\n", toolX, toolY);
                return 1;
            }

            printf("FK failed: joint angles are outside the allowed range\n");
            return 0;
        }

        /*------------------------|Inverse Kinematics Command|---------------------*/
        case SCARA_IK:
        {
            double theta1;
            double theta2;
            double toolX = atof(con->args[0]);
            double toolY = atof(con->args[1]);
            int armSolution = atoi(con->args[2]);

            if (scaraIK(toolX, toolY, &theta1, &theta2, armSolution) == 0) {
                printf("IK result: theta1 %.2f degrees, theta2 %.2f degrees\n", theta1, theta2);
                return 1;
            }

            printf("IK failed: target is unreachable or violates joint limits\n");
            return 0;
        }

        /*------------------------|Set SCARA Angles Command|------------------------*/
        case SET_SCARA_ANGLES:
        {
            if (!motorIsControlEnabled()) {
                printf("Position control is released. Type hold first.\n");
                return 0;
            }

            double theta1 = atof(con->args[0]);
            double theta2 = atof(con->args[1]);

            if (setScaraAngles(theta1, theta2)) {
                printf("Movement complete\n");
                return 1;
            }

            printf("Movement did not complete: control was released, the target changed, or the movement timed out\n");
            return 0;
        }
    }

    return 0;
}

/*|processScaraCommand|---------------------------------------------------------
#
# Description:
#   - Accepts a command from any input source.
#   - Copies, parses, validates, and executes the command.
#   - Preserves the original two-angle terminal input format.
#
# Inputs:
#   - SCARA_CONSOLE *con (Pointer to the shared console state)
#   - const char *input (Raw command string)
#
# Returns:
#   - 1 (Success)
#   - 0 (Invalid command or execution failed)
#
# Last Modified: September 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int processScaraCommand(SCARA_CONSOLE *con, const char *input){
    double theta1;
    double theta2;
    char extraCharacter;
    int copiedLength;

    if (con == NULL || input == NULL) {
        return 0;
    }

    if (sscanf(input, " %lf %lf %c", &theta1, &theta2, &extraCharacter) == 2) {
        copiedLength = snprintf(con->userInput,
                                sizeof(con->userInput),
                                "setScaraAngles %.17g %.17g",
                                theta1,
                                theta2);
    } else {
        copiedLength = snprintf(con->userInput, sizeof(con->userInput), "%s", input);
    }

    if (copiedLength < 0 || copiedLength >= (int)sizeof(con->userInput)) {
        printf("Error: command is too long.\n");
        return 0;
    }

    parseScaraCommand(con);

    if (!validateScaraCommand(con)) {
        return 0;
    }

    return executeScaraCommand(con);
}

static int parseDoubleArgument(const char *text, double *value)
{
    // strtod stores the first character that was not converted into end.
    char *end;

    // Clear the previous conversion error before checking this argument.
    errno = 0;
    double parsedValue = strtod(text, &end);

    // Reject empty input, leftover characters, values outside the double range, NaN, and infinity.
    if (text == end || *end != '\0' || errno == ERANGE || !isfinite(parsedValue)) {
        return 0;
    }

    // Only copy the result to the caller after the entire argument is validated.
    *value = parsedValue;
    return 1;
}

static int parseIntArgument(const char *text, int *value)
{
    // strtol stores the first character that was not converted into end.
    char *end;

    // Clear the previous conversion error before checking this argument.
    errno = 0;
    long parsedValue = strtol(text, &end, 10);

    // Reject empty input, leftover characters, and values outside the int range.
    if (text == end || *end != '\0' || errno == ERANGE ||
        parsedValue < INT_MIN || parsedValue > INT_MAX) {
        return 0;
    }

    // Only copy the result to the caller after the entire argument is validated.
    *value = (int)parsedValue;
    return 1;
}
