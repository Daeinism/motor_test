#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdbool.h>

typedef void (*LimitSwitchPressedHandler)(void);

void limitSwitchInit(LimitSwitchPressedHandler pressedHandler);
bool limitSwitchAnyIsPressed(void);
bool limitSwitchLink1LeftIsPressed(void);
bool limitSwitchLink1RightIsPressed(void);
bool limitSwitchLink2LeftIsPressed(void);
bool limitSwitchLink2RightIsPressed(void);

#endif
