#ifndef PID_CALCULATOR_H
#define PID_CALCULATOR_H

#include <stdint.h>

int pidCalculatorUpdate(int32_t targetCount, int32_t currentCount, float deltaTime);
void pidCalculatorReset(void);

#endif
