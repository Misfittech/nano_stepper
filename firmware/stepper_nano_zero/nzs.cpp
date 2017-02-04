/*
 * nzs.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

#include "nzs.h"
#include "commands.h"
#include "nonvolatile.h"
#include "angle.h"

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")


volatile bool enableState=true;

StepperCtrl stepperCtrl;
NZS_LCD Lcd;

int menuCalibrate(int argc, char *argv[])
{
	stepperCtrl.calibrateEncoder();
}

int menuTestCal(int argc, char *argv[])
{
	Angle error;
	int32_t x,y;
	char str[25];
	error=stepperCtrl.maxCalibrationError();

	x=(36000*(int32_t)error)/ANGLE_STEPS;
	LOG("Error %d %d", (int32_t)error, x);
	y=x/100;
	x=x-(y*100);
	x=abs(x);
	sprintf(str, "%d.%02d deg",y,x);
	Lcd.lcdShow("Cal Error", str,"");
  LOG("Calibration error %s",str);
#ifndef MECHADUINO_HARDWARE
	while(digitalRead(PIN_SW3)==1)
	{
		//wait for button press
	}
	while(digitalRead(PIN_SW3)==0)
	{
		//wait for button release
	}
#endif
}

static  options_t stepOptions[] {
		{"200"},
		{"400"},
		{""},
};

//returns the index of the stepOptions when called
// with no arguments.
int motorSteps(int argc, char *argv[])
{
	if (argc==0)
	{
		int i;
		i=NVM->motorParams.fullStepsPerRotation;
		if (i==400)
		{
			return 1;
		}
		return 0; //default to 200
	}
	if (argc>0)
	{
		int32_t i;
		MotorParams_t params;
		memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params));
		i=atol(argv[0]);
		if (i!=params.fullStepsPerRotation)
		{
			params.fullStepsPerRotation=i;
			nvmWriteMotorParms(params);
		}
	}

	return 0;
}

static  options_t currentOptions[] {
		{"0"},
		{"100"},
		{"200"},
		{"300"},
		{"400"},
		{"500"},
		{"600"},
		{"700"},
		{"800"},
		{"900"},
		{"1000"},
		{"1100"},
		{"1200"},
		{"1300"},
		{"1400"},
		{"1500"},
		{"1600"},
		{"1700"},
		{"1800"},
		{"1900"},
		{"2000"},
		{"2100"},
		{"2200"},
		{"2300"},
		{"2400"},
		{"2500"},
		{"2600"},
		{"2700"},
		{"2800"},
		{"2900"},
		{"3000"},
		{"3100"},
		{"3200"},
		{"3300"},
		{""},
};

int motorCurrent(int argc, char *argv[])
{
	LOG("called motorCurrent %d",argc);
	if (argc==1)
	{
		int i;
		LOG("called %s",argv[0]);
		i=atol(argv[0]);
		i=i*100;
		MotorParams_t params;
		memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params));
		if (i!=params.currentMa)
		{
			params.currentMa=i;
			nvmWriteMotorParms(params);
		}
		return i/100;
	}
	int i;
	i=NVM->motorParams.currentMa/100;
	LOG(" motorCurrent return %d",i);
	return i;

}

int motorHoldCurrent(int argc, char *argv[])
{
	if (argc==1)
	{
		int i;
		i=atol(argv[0]);
		i=i*100;
		MotorParams_t params;
		memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params));
		if (i!=params.currentHoldMa)
		{
			params.currentHoldMa=i;
			nvmWriteMotorParms(params);
		}
		return i/100;
	}else
	{
		int i;
		i=NVM->motorParams.currentHoldMa/100;
		return i;
	}
}

static  options_t microstepOptions[] {
		{"1"},
		{"2"},
		{"4"},
		{"8"},
		{"16"},
		{"32"},
		{"64"},
		{"128"},
		{"256"},
		{""}
};

int microsteps(int argc, char *argv[])
{
	if (argc==1)
	{
		int i,steps;
		i=atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		steps=0x01<<i;
		if (steps!=params.microsteps)
		{
			params.microsteps=steps;
			nvmWriteSystemParms(params);
		}
		return i;
	}
	int i,j;
	i=NVM->SystemParams.microsteps;
	for (j=0; j<9; j++)
	{

		if ((0x01<<j) == i)
		{
			return j;
		}
	}
	return 0;
}

static  options_t controlLoopOptions[] {
		{"Off"},
		{"Open"},
		{"Simple"},
		{"Pos PID"},
		{"Vel PID"},
		{""}
};



int controlLoop(int argc, char *argv[])
{
	if (argc==1)
	{
		int i;
		i=atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i!=params.controllerMode)
		{
			params.controllerMode=(feedbackCtrl_t)i;
			nvmWriteSystemParms(params);
		}
		return i;
	}
	return NVM->SystemParams.controllerMode;
}


static  options_t errorPinOptions[] {
		{"Enable"},
		{"!Enable"}, //error pin works like enable on step sticks
		{"Error"},
		//	{"BiDir"}, //12/12/2016 not implemented yet
		{""}
};


int errorPin(int argc, char *argv[])
{
	if (argc==1)
	{
		int i;
		i=atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i!=params.errorPinMode)
		{
			params.errorPinMode=(ErrorPinMode_t)i;
			nvmWriteSystemParms(params);
		}
		return i;
	}
	return NVM->SystemParams.errorPinMode;
}

static  options_t dirPinOptions[] {
		{"High CW"},
		{"High CCW"},
		{""}
};

int dirPin(int argc, char *argv[])
{
	if (argc==1)
	{
		int i;
		i=atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i!=params.dirPinRotation)
		{
			params.dirPinRotation=(RotationDir_t)i;
			nvmWriteSystemParms(params);
		}
		return i;
	}
	return NVM->SystemParams.dirPinRotation;
}


static  menuItem_t MenuMain[] {
		{"Calibrate", menuCalibrate,NULL},
		{"Test Cal", menuTestCal,NULL},
		//		{"Mtr steps", motorSteps,stepOptions}, NOT GOOD for user to call this
		{"Motor mA", motorCurrent,currentOptions},
		{"Hold mA", motorHoldCurrent,currentOptions},
		{"Microstep", microsteps,microstepOptions},
		//		{"Ctlr Mode", controlLoop,controlLoopOptions}, //this may not be good for user to call
		{"Error Pin", errorPin,errorPinOptions},
		{"Dir Pin", dirPin,dirPinOptions},


		{ "", NULL}
};

static  menuItem_t MenuCal[] {
		{"Calibrate", menuCalibrate,NULL},
		//{"Test Cal", menuTestCal,NULL},
		{ "", NULL}
};



//this function is called on the rising edge of a step from external device
static void stepInput(void)
{
	static int dir;
	//read our direction pin
	dir = digitalRead(PIN_DIR_INPUT);

	if (CW_ROTATION == NVM->SystemParams.dirPinRotation)
	{
		dir=!dir; //reverse the rotation
	}
	stepperCtrl.requestStep(dir,1);
}


//this function is called when error pin changes as enable signal
static void enableInput(void)
{
	if (NVM->SystemParams.errorPinMode == ERROR_PIN_MODE_ENABLE)
	{
		static int enable;
		//read our enable pin
		enable = digitalRead(PIN_ERROR);
		enableState=enable;
		//stepperCtrl.enable(enable);
	}
	if (NVM->SystemParams.errorPinMode == ERROR_PIN_MODE_ACTIVE_LOW_ENABLE)
	{
		static int enable;
		//read our enable pin
		enable = !digitalRead(PIN_ERROR);
		enableState=enable;
		//stepperCtrl.enable(enable);
	}
}





void TC5_Handler()
{
	if (TC5->COUNT16.INTFLAG.bit.OVF == 1)
	{
		int error=0;

		error=(stepperCtrl.processFeedback()); //handle the control loop
		YELLOW_LED(error);
		if (NVM->SystemParams.errorPinMode == ERROR_PIN_MODE_ERROR)
		{
			GPIO_OUTPUT(PIN_ERROR);
			if (error)
			{	//assume high is inactive and low is active on error pin
				digitalWrite(PIN_ERROR,LOW);
			}else
			{
				digitalWrite(PIN_ERROR,HIGH);
			}
		}

		TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	}

}

//check the NVM and set to defaults if there is any
void validateAndInitNVMParams(void)
{

	if (false == NVM->sPID.parametersVaild)
	{
		nvmWrite_sPID(1.0,0.001, 0.01);
	}

	if (false == NVM->pPID.parametersVaild)
	{
		nvmWrite_pPID(1.0, 0, 0);
	}

	if (false == NVM->vPID.parametersVaild)
	{
		nvmWrite_vPID(2.0, 1.0, 1.0);
	}

	if (false == NVM->SystemParams.parametersVaild)
	{
		SystemParams_t params;
		params.microsteps=16;
		params.controllerMode=CTRL_SIMPLE;
		params.dirPinRotation=CW_ROTATION; //default to clockwise rotation when dir is high
		params.errorLimit=(int32_t)ANGLE_FROM_DEGREES(1.8);
		params.errorPinMode=ERROR_PIN_MODE_ENABLE;  //default to enable pin
		nvmWriteSystemParms(params);
	}

	//the motor parameters are check in the stepper_controller code
	// as that there we can auto set much of them.


}


void NZS::begin(void)
{
	int to=20;
	stepCtrlError_t stepCtrlError;

	//set up the pins correctly on the board.
	boardSetupPins();

	//start up the USB serial interface
	//TODO check for power on USB before doing this...
	SerialUSB.begin(SERIAL_BAUD);

	//setup the serial port for syslog
	Serial5.begin(SERIAL_BAUD);
	SysLogInit(&Serial5,LOG_DEBUG); //use SWO for the sysloging

	LOG("Power up!");

	//wait for USB serial port to come alive
	while (!SerialUSB)
	{
		to--;
		if (to == 0)
		{
			break;
		}
		delay(500);
	};     //wait for serial


	validateAndInitNVMParams();


#ifndef DISABLE_USBstream //mod OE
	USB_stream.setup(&stepperCtrl);
#endif
	Lcd.begin(&stepperCtrl);
	Lcd.lcdShow("Misfit"," Tech", VERSION);


	LOG("command init!");
	commandsInit(); //setup command handler system

	stepCtrlError=STEPCTRL_NO_CAL;


	while (STEPCTRL_NO_ERROR != stepCtrlError)
	{
		LOG("init the stepper controller");
		stepCtrlError=stepperCtrl.begin(); //start controller before accepting step inputs

		//todo we need to handle error on LCD and through command line
		if (STEPCTRL_NO_POWER == stepCtrlError)
		{

			SerialUSB.println("Appears that there is no Motor Power");
			SerialUSB.println("Connect motor power!");

			Lcd.lcdShow("Waiting", "MOTOR", "POWER");

			while (STEPCTRL_NO_POWER == stepCtrlError)
			{
				stepCtrlError=stepperCtrl.begin(); //start controller before accepting step inputs
			}

		}

		if (STEPCTRL_NO_CAL == stepCtrlError)
		{
			SerialUSB.println("You need to Calibrate");
			Lcd.lcdShow("   NOT ", "Calibrated", " ");
			delay(1000);
			Lcd.setMenu(MenuCal);
			Lcd.forceMenuActive();

			//TODO add code here for LCD and command line loop
			while(false == stepperCtrl.calibrationValid())
			{
				commandsProcess(); //handle commands

				Lcd.process();

			}

			Lcd.setMenu(NULL);

		}

		if (STEPCTRL_NO_ENCODER == stepCtrlError)
		{
			SerialUSB.println("AS5047D not working");
			SerialUSB.println(" try disconnecting power from board for 15+mins");
			SerialUSB.println(" you might have to short out power pins to ground");

			Lcd.lcdShow("Encoder", " Error!", " REBOOT");

			while(1)
			{

			}
		}

	}

	Lcd.setMenu(MenuMain);


	attachInterrupt(digitalPinToInterrupt(PIN_STEP_INPUT), stepInput, RISING);

	attachInterrupt(digitalPinToInterrupt(PIN_ERROR), enableInput, CHANGE);


	SmartPlanner.begin(&stepperCtrl);
	LOG("SETUP DONE!");
}

void NZS::loop(void)
{

	//LOG("loop time is %dus",stepperCtrl.getLoopTime());

	if (enableState != stepperCtrl.getEnable())
	{
		stepperCtrl.enable(enableState);
	}

	commandsProcess(); //handle commands

	Lcd.process();

#ifndef DISABLE_USBstream //mod OE
	if(USB_stream.on&&SerialUSB){
		USB_stream.process();
	}
#endif

	return;
}

#pragma GCC pop_options
