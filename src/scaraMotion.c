#include "scaraMotion.h"

#include <stdint.h>
#include <stdio.h>

#include "encoder.h"
#include "motor.h"

#define MOTOR_MOVE_TIMEOUT_MS 3000 // 3 seconds

static int32_t degreesToEncoderCount(float inputDegrees);

bool setScaraAngles(double theta1Degrees, double theta2Degrees)
{
    if (theta1Degrees < SCARA_LINK1_MIN_TARGET_DEGREES ||
        theta1Degrees > SCARA_LINK1_MAX_TARGET_DEGREES ||
        theta2Degrees < SCARA_LINK2_MIN_TARGET_DEGREES ||
        theta2Degrees > SCARA_LINK2_MAX_TARGET_DEGREES) {
        return false;
    }

    float link1PhysicalDegrees = (float)theta1Degrees;
    float link2PhysicalDegrees = (float)(theta1Degrees + theta2Degrees);

    int32_t link1TargetCounts = degreesToEncoderCount(link1PhysicalDegrees);
    int32_t link2TargetCounts = degreesToEncoderCount(link2PhysicalDegrees);

    // Setting the targetcount based on home
    motorSetLink1TargetCount(link1TargetCounts);
    motorSetLink2TargetCount(link2TargetCounts);

    printf("SCARA target: theta1 %.2f degrees, theta2 %.2f degrees | Physical Link 1: %.2f degrees (%ld counts) | Physical Link 2: %.2f degrees (%ld counts)\n",
           theta1Degrees,
           theta2Degrees,
           link1PhysicalDegrees,
           (long)link1TargetCounts,
           link2PhysicalDegrees,
           (long)link2TargetCounts);

    return motorWaitUntilTargetReached(MOTOR_MOVE_TIMEOUT_MS);
}

static int32_t degreesToEncoderCount(float inputDegrees)
{
    /*-------------------------|Angle to Target Count|----------------------------*/
    // Getting the targetCounts(float) value from the input degree
    float targetCountsFloat = inputDegrees * ENCODER_COUNTS_PER_REVOLUTION / 360.0f;

    // Round the float value to the nearest integer.
    int32_t targetCounts;

    // adding 0.5 or -0.5 before truncating into int for rounding
    if (targetCountsFloat >= 0.0f) {
        targetCounts = (int32_t)(targetCountsFloat + 0.5f);
    }
    else {
        targetCounts = (int32_t)(targetCountsFloat - 0.5f);
    }

    return targetCounts;
}
