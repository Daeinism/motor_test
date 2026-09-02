/*|ROBT 1270: SCARA Console| --------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Advanced Control
# Program: scaraConsole.cpp
#
# Description:
#   This program contains the code for the Scara Command Console.
#
# Author: Dain Kim
# Date Created: April 22nd 2026
# Last Modified: May 4th 2026
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scaraConsole.h"
#include "scara.h"

/*|Global Variables|-----------------------------------------------------------*/
extern CRobot robot;     // the global robot Class instance.  Can be used everywhere

CMD scaraCommands[MAX_CMD] = { // format: {"commandName", number of arguments}
    {"moveScaraJ", 2}, 
    {"moveScaraL", 5},
    {"scaraPenUp", 0},
    {"scaraPenDown", 0},
    {"scaraSpeed", 1},
    {"scaraPenColor", 3},
    {"clearTrace", 0},
    {"rotateJoints", 2},
    {"quit", 0}
};

/*|Function Definitions|-------------------------------------------------------*/
/*|initScaraConsole|------------------------------------------------------------
#
# Description:
#   - Initializes the SCARA console environment and the robot simulator.
#   - Sets terminal colors, clears the screen, and resets the robot to home.
#   - Establishes the default starting state (coordinates, arm solution, pen, speed).
#
# Inputs:
#   - void
#
# Returns: 
#   - SCARA_CONSOLE (The initialized console structure containing the robot state)
#
# Last Modified: May 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole(void){
    // Variable Declarations
    SCARA_CONSOLE con; 
    // **SCARA_CONSOLE also contains SCARA_ROBOT struct**

    // Customize Output
	system("COLOR 0A");
	system("CLS");

	// Initialize SCARA Simulator V3
	if(!robot.Initialize()) exit(0);
	robot.Send("PEN_UP\n");
	robot.Send("HOME\n");
	robot.Send("CLEAR_TRACE\n");
	robot.Send("CLEAR_LOG\n");

    // Default Position for Scara
    con.scaraRobot = initScaraState(600, 0, LEFT_ARM_SOLUTION,{'u',{255, 0, 0}},'H');
    scaraSetState(con.scaraRobot);

    return con;
}
/*|readScaraConsole|------------------------------------------------------------
#
# Description:
#   - Displays a command prompt (>>>) to the user and captures their input.
#   - Reads a line of text from the console and stores it in the console structure.
#   - Limits the input length to MAX_SCARA_STRING to prevent buffer overflow.
#
# Inputs:
#   - SCARA_CONSOLE *con (Pointer to the console structure where input is stored)
#
# Returns: 
#   - void
#
# Last Modified: May 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void readScaraConsole(SCARA_CONSOLE *con){
    printf("\n>>> "); // this acts like a cursor for the user
    gets_s(con->userInput, MAX_SCARA_STRING); // it reads only up to MAX_SCARA_STRING(50)
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
    con->command = strtok(con->userInput, " ,\n");

    if (con->command != NULL) {
        
        // utilizing token for parsing the rest of the arguments
        char* token = strtok(NULL, " ,\n");

        // only if token exists and not more than the max number (currently 5) of arguments
        while (token != NULL && con->nArgs < MAX_ARGS) {

            // updating the argument content 
            con->args[con->nArgs] = token; 
            // add 1 to the nArgs before loop to the next parsing
            con->nArgs++;
            token = strtok(NULL, " ,\n");
        }
    }

    return con->nArgs; //this will be used to validate later
}
/*|validateScaraCommand|--------------------------------------------------------
#
# Description:
#   - Validates the parsed command against the list of available commands.
#   - Verifies that the correct number of arguments was provided for the command.
#   - Performs range checking on specific arguments (coordinates, colors, points).
#   - Ensures motor speed characters ('H', 'M', 'L') are valid.
#
# Inputs:
#   - SCARA_CONSOLE *con (Pointer to the console structure to validate)
#
# Returns: 
#   - 1 (Success: Command and arguments are valid)
#   - 0 (Error: Command not found, wrong argument count, or out-of-range value)
#
# Last Modified: May 4th 2026 by Dain Kim
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
            printf("Error: Command \'%s\' not found.\n", con->command);
            return 0; // fail
        }

    // 4. Validation specific for each command
    switch(con->cmdInd) {
        case MOVE_SCARA_L:
            {
                int numPts = atoi(con->args[4]);
                if ( numPts < MIN_POINTS || numPts > MAX_POINTS) {
                    printf("Error: invalid argument!\n");
                    return 0;
                    break;
                }
            }
        case MOVE_SCARA_J:
            {
                double coordinates;
                for (int i = 0; i < con->nArgs; i++){
                    coordinates = atof(con->args[i]);
                    if (coordinates < MIN_COORD || coordinates > MAX_COORD) {
                        printf("Error: invalid argument!\n");
                        return 0;
                        break;
                    }
                }
                break;
            }
        case SCARA_PEN_UP: 
            // nothing to check
            break;
        case SCARA_PEN_DOWN: 
            // nothing to check
            break;
        case SCARA_SPEED:
        {
            if (strcmp(con->args[0],"H") == 0 ||
                strcmp(con->args[0],"h") == 0 ||
                strcmp(con->args[0],"M") == 0 ||
                strcmp(con->args[0],"m") == 0 ||
                strcmp(con->args[0],"L") == 0 ||
                strcmp(con->args[0],"l") == 0) {
                break;
            }
            else {
                printf("Error: invalid argument!\n");
                return 0;
            }
        }
        case SCARA_PEN_COLOR:
        { // this curly bracket is added to prevent scope error with int colorValue
            int colorValue;
            for (int i = 0; i < con->nArgs; i++){
                colorValue = atoi(con->args[i]);
                if (colorValue < MIN_COLOR || colorValue > MAX_COLOR) {
                    printf("Error: invalid argument!\n");
                    return 0;
                    break;
                }
            }
            break;
        }
        case CLEAR_TRACE:
            // nothing to check
            break;
        case ROTATE_JOINTS: 
        {
            double theta1, theta2;
            theta1 = atof(con->args[0]);
            theta2 = atof(con->args[1]);

            if (theta1 < -MAX_ABS_THETA1_DEG || theta1 > MAX_ABS_THETA1_DEG || 
                theta2 < -MAX_ABS_THETA2_DEG || theta2 > MAX_ABS_THETA2_DEG) {
                printf("Error: invalid argument!\n");
                return 0;
                break;
            }
            break;
        }
        case QUIT:
            break;
    }

    return 1; // otherwise, success
}
/*|executeScaraCommand|---------------------------------------------------------
#
# Description:
#   - Maps validated commands to their respective logic or lower-level functions.
#   - Handles data conversion (atof/atoi) for arguments before execution.
#   - Updates the robot state for movements, pen controls, and speed settings.
#   - Sends formatted command strings to the simulator for hardware-level actions.
#
# Inputs:
#   - SCARA_CONSOLE* con (Pointer to the console structure containing the command)
#
# Returns: 
#   - void
#
# Last Modified: May 4th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void executeScaraCommand(SCARA_CONSOLE* con){

    char commandString[MAX_STRING];

    switch(con->cmdInd){
        case MOVE_SCARA_J:
            {
                // Updating the X, Y coordinates
                con->scaraRobot.armPos.x = atof(con->args[0]);
                con->scaraRobot.armPos.y = atof(con->args[1]);

                // Sending the coordinates to moveScaraJ
                moveScaraJ(&con->scaraRobot);
            }
            break;
        case MOVE_SCARA_L:
            {
                LINE_DATA lineData;
                double x1 = atof(con->args[0]);
                double y1 = atof(con->args[1]);
                double x2 = atof(con->args[2]);
                double y2 = atof(con->args[3]);
                int numPts = atoi(con->args[4]);

                lineData = initLine(x1, y1, x2, y2, numPts); // updating the color depending on the angle of the line
	            moveScaraL(&con->scaraRobot, lineData);
            }
            break;
        case SCARA_PEN_UP:
            robot.Send("PEN_UP\n");
            break;
        case SCARA_PEN_DOWN:
            robot.Send("PEN_DOWN\n");
            break;
        case SCARA_SPEED:
            {
            if (strcmp(con->args[0],"H") == 0 || strcmp(con->args[0],"h") == 0) {
                sprintf(commandString, "MOTOR_SPEED HIGH\n");
            } 
            else if (strcmp(con->args[0],"M" ) == 0 || strcmp(con->args[0],"m") == 0) {
                sprintf(commandString, "MOTOR_SPEED MEDIUM\n");
            } 
            else {
                sprintf(commandString, "MOTOR_SPEED LOW\n");
            }

            robot.Send(commandString);
            }
            break;
        case SCARA_PEN_COLOR:
        {
            con->scaraRobot.toolPos.penColor.r = atoi(con->args[0]); // Updating RED
            con->scaraRobot.toolPos.penColor.g = atoi(con->args[1]); // Updating GREEN
            con->scaraRobot.toolPos.penColor.b = atoi(con->args[2]); // Updating BLUE

            scaraSetState(con->scaraRobot);

            break;
        }
        case CLEAR_TRACE:
            robot.Send("CLEAR_TRACE\n");
            break;
        case ROTATE_JOINTS: 
        {
            double theta1, theta2;
            theta1 = atof(con->args[0]);
            theta2 = atof(con->args[1]);

            sprintf(commandString, "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", theta1, theta2);
            robot.Send(commandString);
            break;
        }   
        case QUIT:
            printf("Exiting SCARA Control...\n");
            break;
    }
}