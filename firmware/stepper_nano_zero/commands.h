#ifndef __COMMANDS_H__
#define __COMMANDS_H__
#include <Arduino.h>
#include "stepper_controller.h"
#include "nzs.h"
#include "I2C_com.h"

extern StepperCtrl stepperCtrl;
extern eepromData_t PowerupEEPROM;
extern i2c_com i2c_com;

void commandsInit(void);
int commandsProcess(void);

#endif //__COMMANDS_H__
