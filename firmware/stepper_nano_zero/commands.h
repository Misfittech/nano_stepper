#ifndef __COMMANDS_H__
#define __COMMANDS_H__
#include <Arduino.h>
#include "stepper_controller.h"
<<<<<<< HEAD
#include "USB_nzs_datastream.h"

extern StepperCtrl stepperCtrl;
extern USB_stream USB_stream;
=======
#include "nzs.h"

extern StepperCtrl stepperCtrl;
extern eepromData_t PowerupEEPROM;
>>>>>>> refs/remotes/Misfittech/master

void commandsInit(void);
int commandsProcess(void);

#endif //__COMMANDS_H__
