#ifndef __COMMANDS_H__
#define __COMMANDS_H__
#include <Arduino.h>
#include "stepper_controller.h"

extern StepperCtrl stepperCtrl;

void commandsInit(void);
int commandsProcess(void);

#endif //__COMMANDS_H__
