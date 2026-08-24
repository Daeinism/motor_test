#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoderInit(void);
int32_t encoderGetLink1Count(void);
int32_t encoderGetLink2Count(void);
void encoderResetLink1Count(void);
void encoderResetLink2Count(void);

#endif
