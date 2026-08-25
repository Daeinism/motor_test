#ifndef PID_CALCULATOR_H
#define PID_CALCULATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool calculationInitialized;
    int32_t previousCountForDerivative;
    int32_t previousTargetCount;
    int32_t previousPositionError;
    float integralError;
} PidCalculatorState;

int pidCalculatorUpdate(
    PidCalculatorState *state,
    int32_t targetCount,
    int32_t currentCount,
    float deltaTime
);
void pidCalculatorReset(PidCalculatorState *state);

#endif
