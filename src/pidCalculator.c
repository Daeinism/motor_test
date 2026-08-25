#include "pidCalculator.h"

#define POSITION_INTEGRAL_ZONE 80 // starts integrating when the error is within this range (in counts)
#define POSITION_INTEGRAL_MAX_OUTPUT 1000.0f // maximum output from the integral term (in counts)
#define POSITION_MIN_DUTY 600 // recommended minimum (lower than 450 may result in weak output)
#define POSITION_MAX_DUTY 1000 // 1023 is the max
#define POSITION_TOLERANCE 3 // Ex) Tolerance 3 × 360 / 1320 ≈ ±0.82° permitted

int pidCalculatorUpdate(PidCalculatorState *state, const PidCalculatorGains *gains, int32_t targetCount, int32_t currentCount, float deltaTime)
{
    // Initializing the variables for the first time
    if (!state->calculationInitialized) {
        state->previousCountForDerivative = currentCount;
        state->previousTargetCount = targetCount;
        state->previousPositionError = 0;
        state->integralError = 0.0f;
        state->calculationInitialized = true;
    }

    int32_t positionError = targetCount - currentCount; // Get the positionError for later calculation
    float encoderVelocity = 0.0f;
    int requestedDuty = 0; // Initializing the request value to 0 first.

    // 1. Calculating the encoder velocity (derivative)
    if (deltaTime > 0.0f) { //deltaTime = 0.02f (20ms) from motorTask
        encoderVelocity = (currentCount - state->previousCountForDerivative) / deltaTime;
    }

    state->previousCountForDerivative = currentCount;

    if (targetCount != state->previousTargetCount ||
        (positionError > 0 && state->previousPositionError < 0) ||
        (positionError < 0 && state->previousPositionError > 0)) {
        state->integralError = 0.0f;
    }

    state->previousTargetCount = targetCount;
    state->previousPositionError = positionError;

    // 2. Determining the move direction & PID Control Logics
    if (positionError > POSITION_TOLERANCE || positionError < -POSITION_TOLERANCE)
    {
        if (positionError <= POSITION_INTEGRAL_ZONE && positionError >= -POSITION_INTEGRAL_ZONE) {
            state->integralError += positionError * deltaTime;

            float integralOutput = gains->ki * state->integralError;
            if (integralOutput > POSITION_INTEGRAL_MAX_OUTPUT) {
                state->integralError = POSITION_INTEGRAL_MAX_OUTPUT / gains->ki;
            } else if (integralOutput < -POSITION_INTEGRAL_MAX_OUTPUT) {
                state->integralError = -POSITION_INTEGRAL_MAX_OUTPUT / gains->ki;
            }
        } else {
            state->integralError = 0.0f;
        }

        // P D calculations
        float controlOutput =
            (gains->kp * positionError) +
            (gains->ki * state->integralError) -
            (gains->kd * encoderVelocity);

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
        state->integralError = 0.0f;
    }

    return requestedDuty;
}

void pidCalculatorReset(PidCalculatorState *state) // clears the PID calculator's internal memory
{
    // this reset is necessary because the PID-I term can accumulate over time and cause overshoot or oscillation
    state->calculationInitialized = false;
    state->previousCountForDerivative = 0;
    state->previousTargetCount = 0;
    state->previousPositionError = 0;
    state->integralError = 0.0f;
}
