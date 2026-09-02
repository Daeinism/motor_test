/*|ROBT 1270: SCARA |----------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara.cpp
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-04-19
# Last Modified: 2026-04-21
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scara.h"
#include <stdio.h>

/*|Global Variables|-----------------------------------------------------------*/
CRobot robot;     // the global robot Class instance. Can be used everywhere
                  // robot.Initialize()
                  // robot.Send()
                  // robot.Close()

/*|Function Definitions|-------------------------------------------------------*/

// Joint and Linear Interpolation Functions
/*|moveScaraJ|------------------------------------------------------------------
#
# Description:
#   - Moves the robot to a specific coordinate using joint movement.
#   - Calculates required angles and updates the simulator if the target is reachable.
#
# Inputs:
#   - SCARA_ROBOT* scaraState (Pointer to the structure containing target position)
#
# Returns: 
#   - 0 (Success)
#   - -1 (Error: Target location is unreachable)
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int moveScaraJ(SCARA_ROBOT* scaraState){
   // 1. set up empty buffer for 2 angles
   double tempTheta1 = 0.0;
   double tempTheta2 = 0.0;

   // 2. test the validity of the location by scaraIK
   int check = scaraIK(scaraState->armPos.x, 
                           scaraState->armPos.y, 
                           &tempTheta1, &tempTheta2, 
                           scaraState->armPos.armSol);

   // 3. if failed validating, return -1                  
   if (check == -1){
      return -1;
   }

   // 4. otherwise, update the angles to newly calculated angles
   scaraState->armPos.theta1 = tempTheta1;
   scaraState->armPos.theta2 = tempTheta2;
   
   // 5. sending the updated info to be sent to simulator
   scaraSetState(*scaraState); //the scaraState pointer needs to be DEREFERENCED
   // that's why we're sending *scaraState rather than scaraState 

   return 0;


}
/*|moveScaraL|------------------------------------------------------------------
#
# Description:
#   - Moves the robot in a straight line between two points.
#   - Calculates intermediate points, validates reachability, and handles arm 
#     solution swaps if necessary to complete the path.
#
# Inputs:
#   - SCARA_ROBOT* scaraState (Pointer to the current robot state)
#   - LINE_DATA line (Structure containing the start, end, and point density)
#
# Returns: 
#   - 0 (Success)
#   - -1 (Error: Any part of the line is unreachable)
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA line){ // took scaraState Pointer & line data
   
   // 0. Memory allocation for variable sized array of SCARA_ROBOT ------------
   SCARA_ROBOT *robline = (SCARA_ROBOT*) malloc(sizeof(SCARA_ROBOT)*line.numPts);
   // ----------------------------|BREAK DOWN|--------------------------------- 
   // - created a pointer *robline to hold the starting address of new memory block
   // - (SCARA_ROBOT*) lets malloc to return generic pointer
   //    this allow compiler to treat this memory as an array of SCARA_ROBOT structures
   // - malloc requests OS for memory
   // - sizeof(SCARA_ROBOT)*line.numPts calculates the size of memory * muliplier 
   // summary "give me memory for X number of SCARA_ROBOT data and allocate them to X address"
   //----------------------------------------------------------------------------
   
   int currentArm = scaraState->armPos.armSol; // previous arm solution as default

   // 1. initialize to default condition of moveScaraL ------------------------
   scaraState->toolPos.penColor = line.color; //default color

   // 2. pre-calculate all points ---------------------------------------------
   for (int i = 0; i < line.numPts; i++) {

      double moveX, moveY;
      double tempTheta1 = 0.0;
      double tempTheta2 = 0.0;
      int check;

      // a. apply the base state to all of the robline instances as a starting point
      robline[i] = *scaraState; //first, cloning the default state

      // b. Parametric equations for obtaining coordinates for intermediate x and y position
      moveX = line.xA + ((line.xB - line.xA) * i) / (line.numPts - 1);
      moveY = line.yA + ((line.yB - line.yA) * i) / (line.numPts - 1);

      // c. updating each intermediate points (robline) with the points achieved above
      robline[i].armPos.x = moveX;
      robline[i].armPos.y = moveY;

      // d. testing the validity with scaraIK
      check = scaraIK(moveX, moveY, &tempTheta1, &tempTheta2, currentArm);

      // e. testing the validity again using opposite arm solution
      if (check == -1) { //if the location is unreachable by current arm solution
         if (currentArm == LEFT_ARM_SOLUTION)
         {
            currentArm =  RIGHT_ARM_SOLUTION;
         }
         else {
            currentArm = LEFT_ARM_SOLUTION;
         }

         check = scaraIK(moveX, moveY, &tempTheta1, &tempTheta2, currentArm);
      }

      // f. If STILL unreachable, return FAIL (-1)
      if (check == -1) {
         free(robline); //also clean up the memory to prevent memory Leak
         return -1; // get out of the loop, get out of the function, and return FAIL
      }

      // g. otherwise, save successful parameters to the array
      robline[i].armPos.theta1 = tempTheta1; // save theta1
      robline[i].armPos.theta2 = tempTheta2; // save theta2
      robline[i].armPos.armSol = currentArm; // save arm solution
   }

   // 3. Execute validated movements ------------------------------------------
   for (int i = 0; i < line.numPts; i++) {
      
      if (i > 0) {// if the orientation was switched inbetween, 

         // printf("[Debug] current i = %d\n", i);

         if (robline[i].armPos.armSol != robline[i-1].armPos.armSol) {

            // printf("[Debug] DING!!\n");

            //===============|MISSING WAYPOINT BUG FIX|===============
            SCARA_ROBOT flipState = robline [i-1]; // make supplementary waypoint
            flipState.armPos.armSol = robline[i].armPos.armSol;
            
            double newTheta1, newTheta2;
            scaraIK(flipState.armPos.x,
                     flipState.armPos.y,
                     &newTheta1, &newTheta2,
                     flipState.armPos.armSol
                  );
            flipState.armPos.theta1 = newTheta1;
            flipState.armPos.theta2 = newTheta2;
            
            flipState.toolPos.penPos = 'u'; // pen UP
            scaraSetState(flipState); // move to the flipped waypoint without drawing

            flipState.toolPos.penPos = 'd';
            scaraSetState(flipState); // start drawing from the flipped waypoint
            robline[i].toolPos.penPos = 'd'; // pen DOWN
         }
         else { //if there was no change in orientation -> keep going
            robline[i].toolPos.penPos = 'd';
         }
      } 

      scaraSetState(robline[i]); // send all movements to the simulator
   }

   // 4. Updating the initial state to the lastest state ----------------------
   *scaraState = robline[line.numPts - 1];
   // at this point, robline[line.numPts-1] is NOT a pointer, it is an actual data
   // indexing pointer like robline[index] will automatically dereference the pointer!!!
   scaraState->toolPos.penPos = 'u';
   scaraSetState(*scaraState);


   // 5. unlocking the once allocated memory before exiting so the system can recycle that spot
   free(robline);
   return 0;
}

/*|initLine|-------------------------------------------------------------------
#
# Description:
#   - Initializes a line structure with start and end points.
#   - Automatically assigns a color based on the slope of the line.
#
# Inputs:
#   - double xA, yA (Starting coordinates)
#   - double xB, yB (Ending coordinates)
#   - int numPts (Total number of points to calculate for the line)
#
# Returns: 
#   - LINE_DATA (The completed line information)
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts){
	LINE_DATA line; // make the instance

   double dx;
   double dy;
   double slope;

   // 1. updating the line data
   line.xA = xA; // starting x
   line.xB = xB; // ending x
   line.yA = yA; // starting y
   line.yB = yB; // endgin y
   line.numPts = numPts;

   // 2. calculate the difference of x and y to see if it's 0 or above
   dx = xB - xA;
   dy = yB - yA;
   

   // 3. conditional processing to update colors
   if (fabs(dy) <= SLOPE_TOL) {
      line.color.r = 0;
      line.color.g = 255;
      line.color.b = 0;
   }
   else if (fabs(dx) <= SLOPE_TOL) {
      line.color.r = 0;
      line.color.g = 0;
      line.color.b = 0;
   }
   else {
      // if none of dy or dx is 0, then it's safe to calculate for the slope
      // otherwise, it may cause a runtime error since it's tyring to divide by 0. 
      slope = dy / dx;

      if (slope > 0) { // POSITIVE slope
         line.color.r = 0;
         line.color.g = 0;
         line.color.b = 255;
      }
      else if (slope < 0) { // NEGATIVE slope
         line.color.r = 255;
         line.color.g = 0;
         line.color.b = 0;
      }
   }

	return line;
}

// SCARA State Functions
/*|initScaraState|--------------------------------------------------------------
#
# Description:
#   - Sets the robot's starting coordinates, arm orientation, and pen settings.
#   - Calculates initial joint angles based on the target position and arm solution.
#
# Inputs:
#   - double x, y (Starting tool coordinates)
#   - int armSol (LEFT_ARM_SOLUTION or RIGHT_ARM_SOLUTION)
#   - SCARA_TOOL penState (Initial pen position and color)
#   - char mtrSpeed (Initial speed setting)
#
# Returns: 
#   - SCARA_ROBOT (The initialized robot state structure)
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed){
	SCARA_ROBOT robot_state;

   robot_state.armPos.x = x;
   robot_state.armPos.y = y;
   robot_state.armPos.armSol = armSol;

   double t1 = 0.0;
   double t2 = 0.0;
   scaraIK(x, y, &t1, &t2, armSol);

   robot_state.armPos.theta1 = t1; //just initialzing at home position
   robot_state.armPos.theta2 = t2; //just initialzing at home position

   robot_state.toolPos = penState;
   robot_state.motorSpeed = mtrSpeed;

	return robot_state;
}
/*|scaraSetState|--------------------------------------------------------------
#
# Description:
#   - Updates the robot's speed, color, pen, and angles all at once.
#   - It remembers the previous settings and only sends a new command if
#     something has actually changed to save time.
#
# Inputs:
#   - SCARA_ROBOT scaraState (The structure containing all robot settings)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void scaraSetState(SCARA_ROBOT scaraState){
   // setting the starting default values
   static double prevTheta1 = 999.0; 
   static double prevTheta2 = 999.0;
   static char prevPenPos = '9';
   static char prevSpeed = '9';
   static RGB_COLOR prevColor = {-1, -1, -1};

   // 1. Check and update Motor Speed
   if (scaraState.motorSpeed != prevSpeed) {
      setScaraSpeed(scaraState.motorSpeed);
      prevSpeed = scaraState.motorSpeed; //updating the last Value
   }
   // 2. Check and update Pen Color
   if (scaraState.toolPos.penColor.r != prevColor.r || 
      scaraState.toolPos.penColor.g != prevColor.g || 
      scaraState.toolPos.penColor.b != prevColor.b) {
      
      setScaraColor(scaraState.toolPos.penColor.r, 
                     scaraState.toolPos.penColor.g, 
                     scaraState.toolPos.penColor.b);
      prevColor = scaraState.toolPos.penColor;
    }

   // 3. Check and update Pen Position
   if (scaraState.toolPos.penPos != prevPenPos) {
      setScaraPen(scaraState.toolPos.penPos);
      prevPenPos = scaraState.toolPos.penPos;
   }

   // 4. Check and update Joint Angles
   if (scaraState.armPos.theta1 != prevTheta1 || scaraState.armPos.theta2 != prevTheta2) {
      setScaraAngles(scaraState.armPos.theta1, scaraState.armPos.theta2);
      prevTheta1 = scaraState.armPos.theta1;
      prevTheta2 = scaraState.armPos.theta2;
   }
}
/*|scaraDisplayState|-----------------------------------------------------------
#
# Description:
#   - Prints the robot's current status.
#   - Displays the arm angles, coordinates, pen color, and pen position 
#
# Inputs:
#   - SCARA_ROBOT scaraState (The current data of the robot to be shown)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void scaraDisplayState(SCARA_ROBOT scaraState) {
	SCARA_POS arm = scaraState.armPos;
	SCARA_TOOL tool = scaraState.toolPos;

	printf("|SCARA STATE|\n");

	// Display Position
	printf("| Theta 1 | Theta 2 |    X    |    Y    |   Arm   |\n");
	printf("|%9.2lf|%9.2lf|%9.2lf|%9.2lf|    %d    |\n", arm.theta1, arm.theta2, arm.x, arm.y, arm.armSol);

	// Display Tool
	printf("|Position |   RED   |  GREEN  |   BLUE  |\n");
	printf("|    %c    |   %3d   |   %3d   |   %3d   |\n", tool.penPos, tool.penColor.r, tool.penColor.g, tool.penColor.b);
}

// Kinematics Functions
/*|degreeToRadian|-------------------------------------------------------------
#
# Description:
#   - converts a given angle from degrees to radians
#
# Inputs:
#   - double angle (in degrees)
#
# Returns: 
#   - double (angle in radians)
#
# Last Modified: Mar 31st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
double degreeToRadian(double angle) {
   angle = angle * ( PI / 180.0 );
   return angle;
}
/*|radianToDegree|-------------------------------------------------------------
#
# Description:
#   - converts a given angle from radians to degrees
#
# Inputs:
#   - double angle (in radians)
#
# Returns: 
#   - double (angle in degrees)
#
# Last Modified: Mar 31st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
double radianToDegree(double angle) {
   angle = angle * ( 180.0 / PI);
   return angle;
}
/*|scaraFK|--------------------------------------------------------------------
#
# Description:
#   - performs Forward Kinematics (FK) for the SCARA robot
#   - validates if the input joint angles are within mechanical limits
#   - calculates the resulting (x, y) tool coordinates based on arm lengths and angles
#
# Inputs:
#   - double ang1 (joint 1 angle in degrees)
#   - double ang2 (joint 2 angle in degrees)
#   - double *toolX (pointer to store calculated X coordinate)
#   - double *toolY (pointer to store calculated Y coordinate)
#
# Returns: 
#   - 0 (success)
#   - -1 (error: input angles out of range)
#
# Last Modified: April 19th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int scaraFK(double ang1, double ang2, double* toolX, double* toolY){
	// 1. check if angles are out of range
   if (ang1 < -MAX_ABS_THETA1_DEG || ang1 > MAX_ABS_THETA1_DEG || 
       ang2 < -MAX_ABS_THETA2_DEG || ang2 > MAX_ABS_THETA2_DEG) {
      return -1; // it is out of range.
   }
   
   // 2. convert degree to rad
   ang1 = degreeToRadian(ang1);
   ang2 = degreeToRadian(ang2);

   // 3. convert them to x y coordinates & update
   *toolX = L1 * cos(ang1) + L2 * cos(ang1 + ang2);
   *toolY = L1 * sin(ang1) + L2 * sin(ang1 + ang2);

   // 0. debug prints
   // printf("-----scaraFK------\n");
   // printf("ang1: %.2lf\n", radianToDegree(ang1));
   // printf("ang2: %.2lf\n", radianToDegree(ang2));
   // printf("x: %.2lf\n", *x);
   // printf("y: %.2lf\n", *y);

   return 0;
}
/*|scaraIK|--------------------------------------------------------------------
#
# Description:
#   - performs Inverse Kinematics (IK) for the SCARA robot
#   - checks if the target (x, y) coordinates are physically reachable
#   - calculates the required joint angles for either a Left or Right arm orientation
#   - restricts the resulting angles to the mechanical limits
#
# Inputs:
#   - double toolX (target X coordinate)
#   - double toolY (target Y coordinate)
#   - double *ang1 (pointer to store calculated joint 1 angle)
#   - double *ang2 (pointer to store calculated joint 2 angle)
#   - int arm (LEFT_ARM_SOLUTION or RIGHT_ARM_SOLUTION)
#
# Returns: 
#   - 0 (success)
#   - -1 (error: target unreachable or resulting angles out of range)
#
# Last Modified: April 19th 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
int scaraIK(double toolX, double toolY, double* ang1, double* ang2, int arm){
	
   // D = distance from start to the end point
   // L1 = first arm length
   // L2 = second arm lengh

   // 1. Check if the target is reachable -------------------------------------
   double LSquared = toolX*toolX + toolY*toolY;
   double L = sqrt(LSquared);

   if (L > (L1 + L2) || L < fabs(L1 - L2)) {
      return -1; // out of range -> EXIT
   }

   // 2. Calculating the overall angle (beta) ---------------------------------
   double beta = atan2(toolY, toolX);

   // 3. Calculating the alpha ------------------------------------------------
   double cosAlpha = (L2 * L2 - LSquared - L1 * L1) / (-2.0 * L * L1);
   
   // prevent numbers like 1.00000000000002 (clamping)
   if (cosAlpha > 1.0) {
      cosAlpha = 1.0;
   } 
   else if (cosAlpha < -1.0) {
      cosAlpha = -1.0;
   }
   double alpha = acos(cosAlpha);

   // 4. Calculating Theta1 ---------------------------------------------------
   double theta1;
   if (arm == LEFT_ARM_SOLUTION) {
      theta1 = beta + alpha;
   }
   else if (arm == RIGHT_ARM_SOLUTION){
      theta1 = beta - alpha;
   }

   while (theta1 > PI) { // reformatting the angles that are over 180
    theta1 -= 2.0 * PI; // subtracting 360 degrees
   }
   while (theta1 < -PI) { // reformatting the angles that are over -180
      theta1 += 2.0 * PI; // adding 360 degrees
   }

   // 5. Calculating theta2 ---------------------------------------------------
   double theta2 = atan2(toolY - L1 * sin(theta1), toolX - L1 * cos(theta1)) - theta1;

   while (theta2 > PI) { // reformatting the angles that are over 180
    theta2 -= 2.0 * PI; // subtracting 360 degrees
   }
   while (theta2 < -PI) { // reformatting the angles that are over -180
      theta2 += 2.0 * PI; // adding 360 degrees
   }

   // 6. Converting radian to degree ------------------------------------------
   double a1 = radianToDegree(theta1);
   double a2 = radianToDegree(theta2);

   // 7. Checking the range of motion
   if (a1 < -MAX_ABS_THETA1_DEG || a1 > MAX_ABS_THETA1_DEG || 
       a2 < -MAX_ABS_THETA2_DEG || a2 > MAX_ABS_THETA2_DEG) {
      return -1; // it is out of range.
   }

   // 8. allocating the angle data into pointer -------------------------------
   *ang1 = a1;
   *ang2 = a2;

   return 0;
}

// Scara Control Functions
/*|setScaraAngles|-------------------------------------------------------------
#
# Description:
#   - Formats and sends a command to the robot simulator to rotate joints.
#   - Uses sprintf to package the target angles into a protocol-compliant string.
#
# Inputs:
#   - double ang1 (target angle for joint 1 in degrees)
#   - double ang2 (target angle for joint 2 in degrees)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void setScaraAngles(double ang1, double ang2){
	char commandString[MAX_STRING];

   sprintf(commandString, "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", ang1, ang2);
   
   robot.Send(commandString);
}
/*|setScaraColor|--------------------------------------------------------------
#
# Description:
#   - Updates the current drawing color of the SCARA pen in the simulator.
#   - Formats the RGB values into a command string and transmits it via robot.Send.
#
# Inputs:
#   - int r (Red intensity, 0-255)
#   - int g (Green intensity, 0-255)
#   - int b (Blue intensity, 0-255)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void setScaraColor(int r, int g, int b){
   char commandString[MAX_STRING];

   sprintf(commandString, "PEN_COLOR %d %d %d\n", r, g, b); 

   robot.Send(commandString);
}
/*|setScaraPen|----------------------------------------------------------------
#
# Description:
#   - Controls whether the robot's pen is touching the paper or lifted up.
#   - If given the letter 'd', the pen goes down to draw; otherwise, it stays up.
#
# Inputs:
#   - char pen (Use 'd' for pen down, any other character for pen up)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void setScaraPen(char pen){
   
   // only pen down when condition is met. Otherwise, always up
   if (pen == 'd'){
      robot.Send("PEN_DOWN\n");
   }
   else {
      robot.Send("PEN_UP\n");
   }
   
}
/*|setScaraSpeed|--------------------------------------------------------------
#
# Description:
#   - Sets how fast the robot moves by choosing High, Medium, or Low speed.
#   - It checks the input letter and tells the simulator which speed to use.
#
# Inputs:
#   - char speed (Use 'H' for High, 'M' for Medium, or any other for Low)
#
# Returns: 
#   - void
#
# Last Modified: April 21st 2026 by Dain Kim
# -----------------------------------------------------------------------------*/
void setScaraSpeed(char speed){
   char commandString[MAX_STRING];

   if (speed == 'H') {
      sprintf(commandString, "MOTOR_SPEED HIGH\n");
   } 
   else if (speed == 'M') {
      sprintf(commandString, "MOTOR_SPEED MEDIUM\n");
   } 
   else {
      sprintf(commandString, "MOTOR_SPEED LOW\n");
   }

   robot.Send(commandString);
}

/*|Dain's Notes|---------------------------------------------------------------
- sprintf
	- 's' stands for string
	- this saves formatted text into a string variable
	- it allows messages with variables to be stored in memory, transmitted over a network,
		or processed by another function
	ex) sprintf(command, "Move to %.2f", ang1); will save the text in to the command array
		this then will be sent to simulator using robot.Send(command)
	- if I want to send a formatted texts like above example through robot.Send(),
		first creating that sprintf is absolute must

- -> symbol
   - this is a "structure pointer operator"
   - ex) scaraState->armPos.x 
      this means we're trying to access what's inside the scaraState's armpos.x address

- Dereferencing mistakes
   - when a function took a pointer as a input parameter, 
      it's crucial to dereference the variable when sending the values to another
      functions by dereferencing it.
   ex) moveScaraJ takes *scaraState as input parameter, so it needs to send the
      actual value of scaraState to setScaraState by (*scaraState) which is dereferenced.

- dividing by zero
   - this can cause a runtime error
   - therefore it's important to get rid of all possibility of division by zero through
      filtering the numbers out by conditional formats just like in initLine().

- STRUCTS declaration rule
   - when delaring STRUCTS that are based on one another, the order of declaration matters
      the most foundational STRUCT needs to be mentioned first on the very top
      the secondary STRUCT that is dependant on the primary STRUCT needs to at the bottom
      just like the hierarchical structure
*/
/*|Example Codes|--------------------------------------------------------------
int main(){
   // Variables
   char commandString[MAX_STRING];   // string for simulator commands
   double thetaDeg1,thetaDeg2;

   // Open a connection with the simulator
   if(!robot.Initialize()) return 0;
   
   // here are examples of how to send the robot commands. must use robot.
   robot.Send("CYCLE_PEN_COLORS OFF\n");  // DON'T FORGET \n AT THE END OF _EVERY_ COMMAND
   robot.Send("PEN_COLOR 0 0 255\n");
   robot.Send("ROTATE_JOINT ANG1 150.00 ANG2 90.00\n"); // here is an explicit way to move the robot when you know absolute values of angles
   robot.Send("PEN_COLOR 255 0 0\n");
   
   // And here is how you will move the robot more generally by calculating thetaDeg1 and thetaDeg2 variables. 
   // Note those are your variable names ...   scaraIK would be called first to calculate thetaDeg1 and thetaDeg2
   thetaDeg1=-45.0; // define some angles. Arbitrary.  
   thetaDeg2=-155.0;
   sprintf(commandString, "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", thetaDeg1, thetaDeg2);
   robot.Send(commandString);

   robot.Send("PEN_UP\n");
   robot.Send("ROTATE_JOINT ANG1 150.00 ANG2 90.00\n\n");
   robot.Send("PEN_DOWN\n");
   robot.Send(commandString);
   robot.Send("PEN_COLOR 10 64 109\n");
   robot.Send("ROTATE_JOINT ANG1 -150.00 ANG2 90.00\n");
   robot.Send("MESSAGE Erasing Traces\n");
   robot.Send("CLEAR_TRACE\n");

   robot.Send("HOME\n");

   printf("\n\nWhat do the following commands do?\n");
   getchar();

   robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");
   robot.Send("CLEAR_POSITION_LOG\n");
   robot.Send("MESSAGE Bye-Bye\n");
   //robot.Send("SHUTDOWN_SIMULATION\n");
   robot.Send("END\n");

   printf("\n\nPress ENTER to end the program...\n");
   getchar();

   
   robot.Close(); // close remote connection
   return 0;
}
*/