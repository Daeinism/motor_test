/*|SCARA-LP MK II|----------------------------------------------------------
#
# Project: Summer Project 2026
# Program: scara.h
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-02
# Last Modified: 2026-09-02
# -----------------------------------------------------------------------------*/

#ifndef SCARA_H_
#define SCARA_H_

/*|Includes|-------------------------------------------------------------------*/
#include <math.h>

/*|CONSTANTS|------------------------------------------------------------------*/
#define PI                    3.14159265358979323846
#define LEFT_ARM_SOLUTION     1           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION    0           // index that can be used to indicate right arm
#define L1                    142.0       // inner arm length
#define L2                    130.0       // outer arm length
#define MAX_ABS_THETA1_DEG    80.0        // max angle of inner arm
#define MAX_ABS_THETA2_DEG    120.0       // max angle of outer arm relative to inner arm

/*|Function Declarations|------------------------------------------------------*/
// Kinematics Functions
int scaraFK(double ang1, double ang2, double* toolX, double* toolY);
int scaraIK(double toolX, double toolY, double* ang1, double* ang2, int arm);

#endif /* SCARA_H_ */
