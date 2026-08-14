#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdbool.h>

typedef void (*LimitSwitchPressedHandler)(void);

void limitSwitchInit(LimitSwitchPressedHandler pressedHandler);
bool limitSwitchIsAnyPressed(void);
bool limitSwitchIsLeftPressed(void);
bool limitSwitchIsRightPressed(void);

#endif
