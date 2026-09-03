#ifndef SCARA_MOTION_H
#define SCARA_MOTION_H

#include <stdbool.h>

#define SCARA_LINK1_MIN_TARGET_DEGREES -80.0
#define SCARA_LINK1_MAX_TARGET_DEGREES 80.0
#define SCARA_LINK2_MIN_TARGET_DEGREES -120.0
#define SCARA_LINK2_MAX_TARGET_DEGREES 120.0

bool setScaraAngles(double theta1Degrees, double theta2Degrees);

#endif
