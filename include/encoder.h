#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_COUNTS_PER_REVOLUTION 7360 //Full Quadrature  Reading (Bottom Motor: 7360 )
    // 60RPM motor is 169:1 gear ratio
    // 333RPM motor is 30:1 gear ratio

void encoderInit(void);
int32_t encoderGetLink1Count(void);
int32_t encoderGetLink2Count(void);
void encoderResetLink1Count(void);
void encoderResetLink2Count(void);

#endif
