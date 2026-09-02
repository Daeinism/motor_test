#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef int32_t (*MotorEncoderCountReader)(void);

void motorInit(
    MotorEncoderCountReader link1EncoderCountReader,
    MotorEncoderCountReader link2EncoderCountReader
);
void motorSetLink1TargetCount(int32_t targetCount);
void motorSetLink2TargetCount(int32_t targetCount);
bool motorWaitUntilTargetReached(uint32_t timeoutMs);
void motorHold(void);
void motorRelease(void);
void motorEmergencyStop(void);
bool motorIsControlEnabled(void);

#endif
