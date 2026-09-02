/*|ROBT 1270: SCARA |----------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara.h
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-04-19
# Last Modified: 2026-04-19
# -----------------------------------------------------------------------------*/

#ifndef SCARA_H_
#define SCARA_H_

/*|Includes|-------------------------------------------------------------------*/
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.
#include <math.h>

/*|CONSTANTS|------------------------------------------------------------------*/
#define PI                    3.14159265358979323846
#define NUM_LINES             8           // number of lines for the main loop
#define MAX_POINTS            50          // maximum number of points in a line
#define MAX_STRING            256         // for the commandString array size
#define LEFT_ARM_SOLUTION     1           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION    0           // index that can be used to indicate right arm
#define L1                    350.0       // inner arm length
#define L2                    250.0       // outer arm length
#define MAX_ABS_THETA1_DEG    150.0       // max angle of inner arm
#define MAX_ABS_THETA2_DEG    170.0       // max angle of outer arm relative to inner arm
#define SLOPE_TOL             (1.0e-5)    // for pen color based on slope
#define POINT_TOL             (1.0e-8)    // use to check if previous line end point is
                                          // the same as current line start point

/*|Structures|-----------------------------------------------------------------*/
// RGB Color
typedef struct RGB_COLOR {
	int r, g, b;            		// red, green, blue components (0-255 each)
} RGB_COLOR;

// Line Data
typedef struct LINE_DATA {
	double xA, yA, xB, yB;  		// start point(A), end point(B)
	int numPts;             		// number of points (includes endpoints, >=2)
	RGB_COLOR color;        		// the line color RGB components
} LINE_DATA;

// Forward and Inverse Kinematics Variables
typedef struct SCARA_POS {
	double x, y, theta1, theta2;    // x, y defined by FK and theta1, theta2 defined by IK.
	int armSol;                     // right or left arm solution for IK
} SCARA_POS;

// Tool Data        
typedef struct SCARA_TOOL {
	char penPos;                    // Pen up ('u') or Down ('d').
	RGB_COLOR penColor;             // Pen RGB Color.
}SCARA_TOOL;

// SCARA Settings
typedef struct SCARA_ROBOT {
	SCARA_POS armPos;               // Position Kinematics Data
	SCARA_TOOL toolPos;             // Tool Data
	char motorSpeed = 'H';          // Speed Settting: H, M or L
}SCARA_ROBOT;

/*|Function Declarations|------------------------------------------------------*/
// Joint and Linear Interpolation Functions
int moveScaraJ(SCARA_ROBOT* scaraState);
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA line);
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts);

// SCARA State Functions
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed);
void scaraSetState(SCARA_ROBOT scaraState);
void scaraDisplayState(SCARA_ROBOT scaraState);

// Kinematics Functions
int scaraFK(double ang1, double ang2, double* toolX, double* toolY);
int scaraIK(double toolX, double toolY, double* ang1, double* ang2, int arm);

// Scara Control Functions
void setScaraAngles(double ang1, double ang2);
void setScaraColor(int r, int g, int b);
void setScaraPen(char pen);
void setScaraSpeed(char speed);

#endif /* SCARA_H_ */
