/*|SCARA-LP MK II|----------------------------------------------------------
#
# Project: Summer Project 2026
# Program: scara.cpp
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-02
# Last Modified: 2026-09-02
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scara.h"
#include <stdio.h>

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
   double theta1 = 0.0;
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
