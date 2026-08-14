#include "pidCalculator.h"

#include <stdbool.h>

#define POSITION_KP 1.0f // PID-P: Proportional Gain per error
#define POSITION_KI 12.0f // PID-I: Integral Gain per error
#define POSITION_KD 0.2f // PID-D: Derivative
#define POSITION_INTEGRAL_ZONE 80 // starts integrating when the error is within this range (in counts)
#define POSITION_INTEGRAL_MAX_OUTPUT 1000.0f // maximum output from the integral term (in counts)
#define POSITION_MIN_DUTY 600 // recommended minimum (lower than 450 may result in weak output)
#define POSITION_MAX_DUTY 1000 
#define POSITION_TOLERANCE 3 // Ex) Tolerance 3 × 360 / 1320 ≈ ±0.82° permitted

static bool calculationInitialized = false;
static int32_t previousCountForDerivative = 0;
static int32_t previousTargetCount = 0;
static int32_t previousPositionError = 0;
static float integralError = 0.0f;

int pidCalculatorUpdate(int32_t targetCount, int32_t currentCount, float deltaTime)
{
    // 1. Initializing the variables for the first time
    if (!calculationInitialized) {
        previousCountForDerivative = currentCount;
        previousTargetCount = targetCount;
        previousPositionError = 0;
        integralError = 0.0f;
        calculationInitialized = true;
    }

    int32_t positionError = targetCount - currentCount; // Get the positionError for later calculation
    float encoderVelocity = 0.0f;
    int requestedDuty = 0; // Initializing the request value to 0 first.

    // 1. Calculating the encoder velocity (derivative)
    if (deltaTime > 0.0f) { //deltaTime = 0.02f (20ms) from motorTask
        encoderVelocity = (currentCount - previousCountForDerivative) / deltaTime;
    }

    previousCountForDerivative = currentCount;

    if (targetCount != previousTargetCount ||
        (positionError > 0 && previousPositionError < 0) ||
        (positionError < 0 && previousPositionError > 0)) {
        integralError = 0.0f;
    }

    previousTargetCount = targetCount;
    previousPositionError = positionError;

    // 2. Determining the move direction & PID Control Logics
    if (positionError > POSITION_TOLERANCE || positionError < -POSITION_TOLERANCE)
    {
        if (positionError <= POSITION_INTEGRAL_ZONE && positionError >= -POSITION_INTEGRAL_ZONE) {
            integralError += positionError * deltaTime;

            float integralOutput = POSITION_KI * integralError;
            if (integralOutput > POSITION_INTEGRAL_MAX_OUTPUT) {
                integralError = POSITION_INTEGRAL_MAX_OUTPUT / POSITION_KI;
            } else if (integralOutput < -POSITION_INTEGRAL_MAX_OUTPUT) {
                integralError = -POSITION_INTEGRAL_MAX_OUTPUT / POSITION_KI;
            }
        } else {
            integralError = 0.0f;
        }

        // P D calculations
        float controlOutput =
            (POSITION_KP * positionError) +
            (POSITION_KI * integralError) -
            (POSITION_KD * encoderVelocity);

        bool outputMovesTowardTarget = // Deciding whether or not need to move the motor
            (positionError > 0 && controlOutput > 0.0f) ||
            (positionError < 0 && controlOutput < 0.0f);

        if (outputMovesTowardTarget) {
            int dutyMagnitude;

            // Turning float into int value with direction
            if (controlOutput > 0.0f) {
                dutyMagnitude = (int)controlOutput;
            } else {
                dutyMagnitude = (int)(-controlOutput);
            }

            // Adjusting the value within the MIN & MAX duty value range
            if (dutyMagnitude < POSITION_MIN_DUTY) { 
                dutyMagnitude = POSITION_MIN_DUTY;
            } else if (dutyMagnitude > POSITION_MAX_DUTY) {
                dutyMagnitude = POSITION_MAX_DUTY;
            }

            // applying the direction of requestedDuty based on position Error
            if (controlOutput > 0.0f) {
                requestedDuty = dutyMagnitude;
            } else {
                requestedDuty = -dutyMagnitude;
            }
        }
    } else {
        integralError = 0.0f;
    }

    return requestedDuty;
}

void pidCalculatorReset(void)
{
    calculationInitialized = false;
    previousCountForDerivative = 0;
    previousTargetCount = 0;
    previousPositionError = 0;
    integralError = 0.0f;
}
