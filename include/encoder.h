#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoderInit(void);
int32_t encoderGetCount(void);
void encoderResetCount(void);

#endif
