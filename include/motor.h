#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_attr.h"

typedef int32_t (*MotorEncoderCountReader)(void);

void motorInit(MotorEncoderCountReader encoderCountReader);
void motorSetTargetCount(int32_t targetCount);
void motorHold(void);
void motorRelease(void);
void motorEmergencyStop(void);
void IRAM_ATTR motorDisableControlFromISR(void);
bool motorIsControlEnabled(void);

#endif
