#include "commands.h"
#include "command.h"
#include "calibration.h"
#include "stepper_controller.h"
#include <stdlib.h>
#include "nonvolatile.h"
#include "Reset.h"
#include "nzs.h"
#include "ftoa.h"


#define COMMANDS_PROMPT (":>")
sCmdUart UsbUart;


static int isPowerOfTwo (unsigned int x)
{
	while (((x % 2) == 0) && x > 1) /* While x is even and > 1 */
		x /= 2;
	return (x == 1);
}


CMD_STR(help,"Displays this message");
CMD_STR(getcal,"Prints the calibration table");
CMD_STR(calibrate,"Calbirates the encoder, should be done with motor disconnected from machine");
CMD_STR(testcal,"tests the calibaration of the encoder");
CMD_STR(microsteps,"gets/sets the microstep size, example 'microsteps 16'");
CMD_STR(step, "Steps motor one step, optionally direction can be set is 'step 1' for reverse");
CMD_STR(feedback, "enable or disable feedback controller, 'feedback 0' - disables, 'feedback 1' - enables");
CMD_STR(readpos, "reads the current angle as 16bit number, applies calibration if valid");
CMD_STR(encoderdiag, "Prints encoder diagnostic")
CMD_STR(spid, "with no arguments prints SIMPLE PID parameters, with arguments sets PID 'sPID Kp Ki Kd' "
		"Where Kp,Ki,Kd are floating point numbers");
CMD_STR(vpid, "with no arguments prints VELOCITY PID parameters, with arguments sets PID 'sPID Kp Ki Kd' "
		"Where Kp,Ki,Kd are floating point numbers");
CMD_STR(ppid, "with no arguments prints POSITIONAL PID parameters, with arguments sets PID 'sPID Kp Ki Kd' "
		"Where Kp,Ki,Kd are floating point numbers");
//CMD_STR(testringing ,"Steps motor at various currents and measures encoder");
//CMD_STR(microsteperror ,"test error on microstepping")
CMD_STR(dirpin, "with no arguments read dirpin setting, with argument sets direction pin rotation");
CMD_STR(errorpinmode,"gets/sets the functionality of the error/enable pin");

CMD_STR(errorlimit, "gets/set the error limit which will assert error pin (when error pin is set for error output)");
CMD_STR(ctrlmode, "gets/set the feedback controller mode of operation");
CMD_STR(maxcurrent, "gets/set the maximum motor current allowed in milliAmps");
CMD_STR(holdcurrent, "gets/set the motor holding current in milliAmps, only used in the simple positional PID mode");
CMD_STR(motorwiring, "gets/set the motor wiring direction, should only be used by experts");
CMD_STR(stepsperrotation, "gets/set the motor steps per rotation, should only be used by experts");

//CMD_STR(sysparams, "with no arguments read parameters, will set with arguments");
//CMD_STR(motorparams, "with no arguments read parameters, will set with arguments");
CMD_STR(boot, "Enters the bootloader");
CMD_STR(move, "moves encoder to absolute angle in degrees 'move 400.1'");
//CMD_STR(printdata, "prints last n error terms");
CMD_STR(velocity, "gets/set velocity in RPMs");
CMD_STR(factoryreset, "resets board to factory defaults");
CMD_STR(stopplaner, "stops the motion planner");;//mod OE
CMD_STR(setzero, "set the reference angle to zero");
CMD_STR(usbstream, "streams time, error and angle data through USB");//mod OE
CMD_STR(stop, "stop USB data stream");//mod OE
CMD_STR(reboot, "reboots NZS");//mod OE


//List of supported commands
sCommand Cmds[] =
{
		COMMAND(help),
		COMMAND(calibrate),
		COMMAND(getcal),
		COMMAND(testcal),
		COMMAND(microsteps),
		COMMAND(step),
		COMMAND(feedback),
		COMMAND(readpos),
		COMMAND(encoderdiag),
		COMMAND(spid),
		COMMAND(vpid),
		COMMAND(ppid),
		//COMMAND(testringing),
		//COMMAND(microsteperror),
		COMMAND(dirpin),
		COMMAND(errorpinmode),
		COMMAND(errorlimit),
		COMMAND(ctrlmode),
		COMMAND(maxcurrent),
		COMMAND(holdcurrent),
		COMMAND(motorwiring),
		COMMAND(stepsperrotation),

		//COMMAND(sysparams),
		//COMMAND(motorparams),
		COMMAND(boot),
		COMMAND(move),
		//COMMAND(printdata),
		COMMAND(velocity),
		COMMAND(factoryreset),
		COMMAND(stopplaner),//mod OE
		COMMAND(setzero),
		COMMAND(usbstream),//mod OE
		COMMAND(stop),//mod OE
		COMMAND(reboot),//mod OE

		{"",0,""}, //End of list signal
};

static int reboot_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
  NVIC_SystemReset();
  return 0;
}

static int stop_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
  //CommandPrintf(ptrUart,"Streaming control data stopped");
  USB_stream.on = false;
  return 0;
}
static int usbstream_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
  CommandPrintf(ptrUart,"Begin streaming control data");
  USB_stream.on = true;
  USB_stream.begin();
  return 0;
}

static int setzero_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	stepperCtrl.setZero();
	return 0;
}


static int stopplaner_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	SmartPlanner.stop();
	return 0;
}



static int stepsperrotation_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{

	if (argc == 0)
	{
		uint32_t x;
		x=NVM->motorParams.fullStepsPerRotation;
		CommandPrintf(ptrUart,"full steps per rotation %u\n\r",x);
		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=atol(argv[0]);

		if (x==200 || x==400)
		{
			MotorParams_t motorParams;

			memcpy(&motorParams,&NVM->motorParams, sizeof(motorParams) );
			motorParams.fullStepsPerRotation=x;

			nvmWriteMotorParms(motorParams);
			stepperCtrl.updateParamsFromNVM();


			x=NVM->motorParams.fullStepsPerRotation;
			CommandPrintf(ptrUart,"full steps per rotation %u\n\r",x);
			CommandPrintf(ptrUart,"please power cycle board\n\r");
			return 0;
		}

	}
	CommandPrintf(ptrUart,"usage 'stepsperrotation 200' or 'stepsperrotation 400'\n\r");

	return 1;
}

static int motorwiring_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{

	if (argc == 0)
	{
		uint32_t x;
		x=NVM->motorParams.motorWiring;
		CommandPrintf(ptrUart,"motor wiring %u\n\r",x);
		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=atol(argv[0]);

		if (x<=1)
		{
			MotorParams_t motorParams;

			memcpy(&motorParams,&NVM->motorParams, sizeof(motorParams) );
			motorParams.motorWiring=x;

			nvmWriteMotorParms(motorParams);
			stepperCtrl.updateParamsFromNVM();


			x=NVM->motorParams.motorWiring;
			CommandPrintf(ptrUart,"motor wiring %u\n\r",x);
			CommandPrintf(ptrUart,"please power cycle board\n\r");
			return 0;
		}

	}
	CommandPrintf(ptrUart,"usage 'motorwiring 0' or 'motorwiring 1'\n\r");

	return 1;
}

static int holdcurrent_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{

	if (argc == 0)
	{
		uint32_t x;
		x=NVM->motorParams.currentHoldMa;
		CommandPrintf(ptrUart,"hold current %u mA\n\r",x);
		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=atol(argv[0]);

		MotorParams_t motorParams;

		memcpy(&motorParams,&NVM->motorParams, sizeof(motorParams) );
		motorParams.currentHoldMa=x;

		nvmWriteMotorParms(motorParams);
		stepperCtrl.updateParamsFromNVM();


		x=NVM->motorParams.currentHoldMa;
		CommandPrintf(ptrUart,"hold current %u mA\n\r",x);
		return 0;


	}
	CommandPrintf(ptrUart, "use 'holdcurrent 1000' to set maximum current to 1.0A");

	return 1;
}


static int maxcurrent_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{

	if (argc == 0)
	{
		uint32_t x;
		x=NVM->motorParams.currentMa;
		CommandPrintf(ptrUart,"max current %u mA\n\r",x);
		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=atol(argv[0]);

		MotorParams_t motorParams;

		memcpy(&motorParams,&NVM->motorParams, sizeof(motorParams) );

		motorParams.currentMa=x;
		nvmWriteMotorParms(motorParams);
		stepperCtrl.updateParamsFromNVM();


		x=NVM->motorParams.currentMa;
		CommandPrintf(ptrUart,"max current %u mA\n\r",x);
		return 0;


	}
	CommandPrintf(ptrUart, "use 'maxcurrent 2000' to set maximum current to 2.0A");

	return 1;
}


static int ctrlmode_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	bool ret;
	if (argc == 0)
	{
		switch(NVM->SystemParams.controllerMode)
		{
			case CTRL_OFF:
				CommandPrintf(ptrUart,"controller Off(0)");
				return 0;
			case CTRL_OPEN:
				CommandPrintf(ptrUart,"controller Open-loop(1)");
				return 0;
			case CTRL_SIMPLE:
				CommandPrintf(ptrUart,"controller Simple-Position-PID(2)");
				return 0;
			case CTRL_POS_PID:
				CommandPrintf(ptrUart,"controller Current-Position-PID(3)");
				return 0;
			case CTRL_POS_VELOCITY_PID:
				CommandPrintf(ptrUart,"controller Velocity-PID(4)");
				return 0;

		}
		return 1;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=atol(argv[0]);

		if (x<=4)
		{
			SystemParams_t systemParams;

			memcpy(&systemParams,&NVM->SystemParams, sizeof(systemParams) );

			systemParams.controllerMode=(feedbackCtrl_t)(x);

			nvmWriteSystemParms(systemParams);
			stepperCtrl.updateParamsFromNVM();

			switch(NVM->SystemParams.controllerMode)
			{
				case CTRL_OFF:
					CommandPrintf(ptrUart,"controller Off(0)");
					return 0;
				case CTRL_OPEN:
					CommandPrintf(ptrUart,"controller Open-loop(1)");
					return 0;
				case CTRL_SIMPLE:
					CommandPrintf(ptrUart,"controller Simple-Position-PID(2)");
					return 0;
				case CTRL_POS_PID:
					CommandPrintf(ptrUart,"controller Current-Position-PID(3)");
					return 0;
				case CTRL_POS_VELOCITY_PID:
					CommandPrintf(ptrUart,"controller Velocity-PID(4)");
					return 0;

			}
			return 1;
		}

	}
	CommandPrintf(ptrUart, "use 'ctrlmode [0 .. 4]' to set control mode");

	return 1;
}
static int errorlimit_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	bool ret;
	char str[20];
	if (argc == 0)
	{
		float x;
		x=ANGLE_T0_DEGREES(NVM->SystemParams.errorLimit);
		ftoa(x,str,2,'f');
		CommandPrintf(ptrUart,"errorLimit %s deg\n\r",str);
		return 0;
	}

	if (argc == 1)
	{
		float x;

		x=fabs(atof(argv[0]));

		SystemParams_t systemParams;

		memcpy(&systemParams,&NVM->SystemParams, sizeof(systemParams) );

		systemParams.errorLimit=ANGLE_FROM_DEGREES(x);

		nvmWriteSystemParms(systemParams);
		stepperCtrl.updateParamsFromNVM();

		x=ANGLE_T0_DEGREES(NVM->SystemParams.errorLimit);
		ftoa(x,str,2,'f');
		CommandPrintf(ptrUart,"errorLimit %s deg\n\r",str);
		return 0;


	}
	CommandPrintf(ptrUart, "use 'errorlimit 1.8' to set error limit to 1.8 degrees");

	return 1;
}


static int dirpin_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	bool ret;

	if (argc == 0)
	{
		if (CW_ROTATION == NVM->SystemParams.dirPinRotation)
		{
			CommandPrintf(ptrUart,"dirpin CW(%d)\n\r",(uint32_t)NVM->SystemParams.dirPinRotation);
		}else
		{
			CommandPrintf(ptrUart,"dirpin CCW(%d)\n\r",(uint32_t)NVM->SystemParams.dirPinRotation);
		}
		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=abs(atol(argv[0]));
		if (x<=1)
		{

			SystemParams_t systemParams;

			memcpy(&systemParams,&NVM->SystemParams, sizeof(systemParams) );

			systemParams.dirPinRotation=(RotationDir_t)x;

			nvmWriteSystemParms(systemParams);
			stepperCtrl.updateParamsFromNVM();

			if (CW_ROTATION == NVM->SystemParams.dirPinRotation)
			{
				CommandPrintf(ptrUart,"dirpin CW(%d)\n\r",(uint32_t)NVM->SystemParams.dirPinRotation);
			}else
			{
				CommandPrintf(ptrUart,"dirpin CCW(%d)\n\r",(uint32_t)NVM->SystemParams.dirPinRotation);
			}
			return 0;

		}
	}
	CommandPrintf(ptrUart, "used 'dirpin 0' for CW rotation and 'dirpin 1' for CCW");


	return 1;
}


static int errorpinmode_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	bool ret;

	if (argc == 0)
	{
		if (ERROR_PIN_MODE_ENABLE == NVM->SystemParams.errorPinMode)
		{
			CommandPrintf(ptrUart,"Error pin -  Enable Active High(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
		}else if (ERROR_PIN_MODE_ACTIVE_LOW_ENABLE == NVM->SystemParams.errorPinMode)
		{
			CommandPrintf(ptrUart,"Error pin -  Enable active low(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
		}else if (ERROR_PIN_MODE_ERROR == NVM->SystemParams.errorPinMode)
		{
			CommandPrintf(ptrUart,"Error pin -  Error pin(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
		} else if (ERROR_PIN_MODE_BIDIR == NVM->SystemParams.errorPinMode)
		{
			CommandPrintf(ptrUart,"Error pin -  Bidi error(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
		}

		return 0;
	}

	if (argc == 1)
	{
		uint32_t x;

		x=abs(atol(argv[0]));
		if (x<=3)
		{

			SystemParams_t systemParams;

			memcpy(&systemParams,&NVM->SystemParams, sizeof(systemParams) );

			systemParams.errorPinMode=(ErrorPinMode_t)x;

			nvmWriteSystemParms(systemParams);
			stepperCtrl.updateParamsFromNVM();

			if (ERROR_PIN_MODE_ENABLE == NVM->SystemParams.errorPinMode)
			{
				CommandPrintf(ptrUart,"Error pin -  Enable Active High(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
			}else if (ERROR_PIN_MODE_ACTIVE_LOW_ENABLE == NVM->SystemParams.errorPinMode)
			{
				CommandPrintf(ptrUart,"Error pin -  Enable active low(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
			}else if (ERROR_PIN_MODE_ERROR == NVM->SystemParams.errorPinMode)
			{
				CommandPrintf(ptrUart,"Error pin -  Error pin(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
			} else if (ERROR_PIN_MODE_BIDIR == NVM->SystemParams.errorPinMode)
			{
				CommandPrintf(ptrUart,"Error pin -  Bidi error(%d)\n\r",(uint32_t)NVM->SystemParams.errorPinMode);
			}
			return 0;

		}
	}
	CommandPrintf(ptrUart, "use 'errorpinmode 0' for enable active high, 'errorpinmode 1' for enable active low  and 'errorpinmode 2' for error output"  );


	return 1;
}

static int factoryreset_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	nvmErase(); //erase all of the flash
	NVIC_SystemReset();
}
static int velocity_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	int64_t x;

	if (1 == argc)
	{
		float rpm;
		rpm=atof(argv[0]);
		x=(int64_t)(DIVIDE_WITH_ROUND(rpm*ANGLE_STEPS,60)); //divide with r


		stepperCtrl.setVelocity(x);
	}
	int64_t y;
	x=(stepperCtrl.getVelocity()*100 *60)/(ANGLE_STEPS);
	y=abs(x-((x/100)*100));
	CommandPrintf(ptrUart,"Velocity is %d.%02d - %d\n\r",(int32_t)(x/100),(int32_t)y,(int32_t)stepperCtrl.getVelocity());

	return 0;
}

//
//static int printdata_cmd(sCmdUart *ptrUart,int argc, char * argv[])
//{
//	int32_t x;
//
//	stepperCtrl.printData();
//
//	return 0;
//}


static int move_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	int32_t x,ma;
	//CommandPrintf(ptrUart, "Move %d",argc);

	if (1 == argc)
	{
		float f;

		f=atof(argv[0]);
		//		if (f>1.8)
		//			f=1.8;
		//		if (f<-1.8)
		//			f=-1.8;
		x=ANGLE_FROM_DEGREES(f);
		LOG("moving %d", x);

		stepperCtrl.moveToAbsAngle(x);
	}
	if (2 == argc)
	{
		float f,rpm,a,y;
		float pos,dx;

		f=atof(argv[0]);
		rpm=atof(argv[1]);
		//		if (f>1.8)
		//			f=1.8;
		//		if (f<-1.8)
		//			f=-1.8;

		SmartPlanner.moveConstantVelocity(f,rpm);
		return 0;
		a=360*rpm/60/1000; //rotations/100ms

		pos=ANGLE_T0_DEGREES(stepperCtrl.getCurrentAngle());
		y=pos;
		if (y>f) a=-a;

		SerialUSB.println(f);
		SerialUSB.println(y);
		SerialUSB.println(a);

		while (abs(y-f)>(2*abs(a)))
		{
			//			SerialUSB.println();
			//			SerialUSB.println(f);
			//		SerialUSB.println(y);
			//		SerialUSB.println(a);
			y=y+a;

			x=ANGLE_FROM_DEGREES(y);
			//LOG("moving %d", x);
			stepperCtrl.moveToAbsAngle(x);
			delay(1);
			//y=stepperCtrl.getCurrentAngle();
		}
		x=ANGLE_FROM_DEGREES(f);
		LOG("moving %d", x);
		stepperCtrl.moveToAbsAngle(x);
	}

	return 0;
}

static int boot_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	initiateReset(250);
}

/*
static int microsteperror_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	int i,n,j;
	bool feedback=stepperCtrl.getFeedback();
	n=200*stepperCtrl.getMicroSteps();

	CommandPrintf(ptrUart, "Function needs fixed");
	return 0;
	stepperCtrl.feedback(false);
	for (j=0; j<2; j++)
	{
		for (i=0; i<n; i++)
		{
			int32_t e;
			stepperCtrl.requestStep(1,1);
			//stepperCtrl.step(1, 2000);
			stepperCtrl.pidFeedback();

			//average 1readings
			int32_t sum=0,ii;
			for (ii=0; ii<1; ii++)
			{
				sum+=stepperCtrl.measureError();
				stepperCtrl.pidFeedback();
			}
			e=sum/ii;
			CommandPrintf(ptrUart,"%d %d\n\r",i,e);
		}
	}
	stepperCtrl.feedback(feedback); //restore feedback
	return 0;
} */

/*
static int testringing_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	stepperCtrl.testRinging();
	return 0;
}
 */

//static int sysparams_cmd(sCmdUart *ptrUart,int argc, char * argv[])
//{
//	if (0 == argc)
//	{
//		CommandPrintf(ptrUart,"microsteps %d\n\r",NVM->SystemParams.microsteps);
//		CommandPrintf(ptrUart,"dirPinRotation %d\n\r",NVM->SystemParams.dirPinRotation);
//		CommandPrintf(ptrUart,"errorLimit %d\n\r",NVM->SystemParams.errorLimit);
//		CommandPrintf(ptrUart,"errorPinMode %d\n\r",NVM->SystemParams.errorPinMode);
//		CommandPrintf(ptrUart,"controllerMode %d\n\r",NVM->SystemParams.controllerMode);
//
//	} else	if (5 == argc)
//	{
//		int32_t x;
//		SystemParams_t systemParams;
//
//		systemParams.microsteps=atol(argv[0]);
//		x=atol(argv[1]);
//		systemParams.dirPinRotation=CCW_ROTATION;
//		if (x==0)
//		{
//			systemParams.dirPinRotation=CW_ROTATION;
//		}
//		systemParams.errorLimit=atol(argv[2]);
//		systemParams.errorPinMode=(ErrorPinMode_t)atol(argv[3]);
//		systemParams.controllerMode=(feedbackCtrl_t)atol(argv[4]);
//
//		nvmWriteSystemParms(systemParams);
//		stepperCtrl.updateParamsFromNVM();
//
//		CommandPrintf(ptrUart,"microsteps %d\n\r",NVM->SystemParams.microsteps);
//		CommandPrintf(ptrUart,"dirPinRotation %d\n\r",NVM->SystemParams.dirPinRotation);
//		CommandPrintf(ptrUart,"errorLimit %d\n\r",NVM->SystemParams.errorLimit);
//		CommandPrintf(ptrUart,"errorPinMode %d\n\r",NVM->SystemParams.errorPinMode);
//		CommandPrintf(ptrUart,"controllerMode %d\n\r",NVM->SystemParams.controllerMode);
//	} else
//	{
//		CommandPrintf(ptrUart, "try 'sysparams microsteps dirPinRotation errorLimit errorPinMode controllerMode'\n\r\tlike 'sysparams 16 0 327 0 2'\n\e");
//	}
//	return 0;
//}

/*
static int motorparams_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	if (0 == argc)
	{
		CommandPrintf(ptrUart,"currentMa %d\n\r",NVM->motorParams.currentMa);
		CommandPrintf(ptrUart,"currentHoldMa %d\n\r",NVM->motorParams.currentHoldMa);
		CommandPrintf(ptrUart,"motorWiring %d\n\r",NVM->motorParams.motorWiring);
		CommandPrintf(ptrUart,"fullStepsPerRotation %d\n\r",NVM->motorParams.fullStepsPerRotation);

	} else	if (4 == argc)
	{
		int32_t x;
		MotorParams_t motorParams;

		motorParams.currentMa=atol(argv[0]);
		motorParams.currentHoldMa=atol(argv[1]);
		motorParams.motorWiring=atol(argv[2]);
		motorParams.fullStepsPerRotation=atol(argv[3]);

		nvmWriteMotorParms(motorParams);
		stepperCtrl.updateParamsFromNVM();

		CommandPrintf(ptrUart,"currentMa %d\n\r",NVM->motorParams.currentMa);
		CommandPrintf(ptrUart,"currentHoldMa %d\n\r",NVM->motorParams.currentHoldMa);
		CommandPrintf(ptrUart,"motorWiring %d\n\r",NVM->motorParams.motorWiring);
		CommandPrintf(ptrUart,"fullStepsPerRotation %d\n\r",NVM->motorParams.fullStepsPerRotation);
	} else
	{
		CommandPrintf(ptrUart, "try 'motorparams currentMa currentHoldMa motorWiring fullStepsPerRotation'\n\r\tlike 'motroparams 2200 1500 0 200'\n\e");
	}
	return 0;
}
*/
static int vpid_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	CommandPrintf(ptrUart, "args %d\n\r",argc);
	if (0 == argc)
	{
		int32_t x,y;
		x=(int32_t)NVM->vPID.Kp;
		y=abs(1000*NVM->vPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->vPID.Ki;
		y=abs(1000*NVM->vPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->vPID.Kd;
		y=abs(1000*NVM->vPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	if (3 == argc)
	{
		float Kp,Ki,Kd;
		int32_t x,y;

		Kp=atof(argv[0]);
		Ki=atof(argv[1]);
		Kd=atof(argv[2]);

		nvmWrite_vPID(Kp,Ki,Kd);
		stepperCtrl.updateParamsFromNVM(); //force the controller to use the new parameters

		x=(int32_t)NVM->vPID.Kp;
		y=abs(1000*NVM->vPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->vPID.Ki;
		y=abs(1000*NVM->vPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->vPID.Kd;
		y=abs(1000*NVM->vPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	return 0;
}

static int ppid_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	if (0 == argc)
	{
		int32_t x,y;
		x=(int32_t)NVM->pPID.Kp;
		y=abs(1000*NVM->pPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->pPID.Ki;
		y=abs(1000*NVM->pPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->pPID.Kd;
		y=abs(1000*NVM->pPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	if (3 == argc)
	{
		float Kp,Ki,Kd;
		int32_t x,y;

		Kp=atof(argv[0]);
		Ki=atof(argv[1]);
		Kd=atof(argv[2]);

		nvmWrite_pPID(Kp,Ki,Kd);
		stepperCtrl.updateParamsFromNVM(); //force the controller to use the new parameters

		x=(int32_t)NVM->pPID.Kp;
		y=abs(1000*NVM->pPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->pPID.Ki;
		y=abs(1000*NVM->pPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->pPID.Kd;
		y=abs(1000*NVM->pPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	return 0;
}

static int spid_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	if (0 == argc)
	{
		int32_t x,y;
		x=(int32_t)NVM->sPID.Kp;
		y=abs(1000*NVM->sPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->sPID.Ki;
		y=abs(1000*NVM->sPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->sPID.Kd;
		y=abs(1000*NVM->sPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	if (3 == argc)
	{
		float Kp,Ki,Kd;
		int32_t x,y;

		Kp=atof(argv[0]);
		Ki=atof(argv[1]);
		Kd=atof(argv[2]);

		nvmWrite_sPID(Kp,Ki,Kd);
		stepperCtrl.updateParamsFromNVM(); //force the controller to use the new parameters

		x=(int32_t)NVM->sPID.Kp;
		y=abs(1000*NVM->sPID.Kp-(x*1000));
		CommandPrintf(ptrUart,"Kp %d.%03d\n\r",x,y);

		x=(int32_t)NVM->sPID.Ki;
		y=abs(1000*NVM->sPID.Ki-(x*1000));
		CommandPrintf(ptrUart,"Ki %d.%03d\n\r",x,y);

		x=(int32_t)NVM->sPID.Kd;
		y=abs(1000*NVM->sPID.Kd-(x*1000));
		CommandPrintf(ptrUart,"Kd %d.%03d\n\r",x,y);
	}
	return 0;
}
static int encoderdiag_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	char str[512];
	stepperCtrl.encoderDiagnostics(str);
	CommandPrintf(ptrUart,"%s",str);
	return 0;
}

static int readpos_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	float pos;
	int32_t x,y;

	pos=ANGLE_T0_DEGREES(stepperCtrl.getCurrentAngle());
	x=int(pos);
	y=abs((pos-x)*100);
	CommandPrintf(ptrUart,"encoder %d.%02d",x,y);
	return 0;
}
static int feedback_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	if (0 == argc)
	{
		CommandPrintf(ptrUart,"must pass argument, 'feedback 0' - disables, 'feedback 1' - enables");
		return 1;
	}
	stepperCtrl.feedback(atoi(argv[0]));
	return 0;
}

static int step_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	if (argc == 0 )
	{
		stepperCtrl.move(0, 1);
		//stepperCtrl.step(STEPPER_FORWARD);
	}else
	{
		int d, steps=1;
		d=atoi(argv[0]);
		if (argc >1)
		{
			steps=atoi(argv[1]);
		}
		if (1 == d)
		{
			stepperCtrl.move(1, steps);
		} else
		{
			stepperCtrl.move(0, steps);
		}
	}
	return 0;
}


static int microsteps_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	bool ret;

	if (argc != 1)
	{
		CommandPrintf(ptrUart,"microsteps %d\n\r",NVM->SystemParams.microsteps);
		return 0;
	}

	int32_t x;

	x=atol(argv[0]);
	if (isPowerOfTwo(x) && x>0 && x<=256)
	{
		SystemParams_t systemParams;

		memcpy(&systemParams,&NVM->SystemParams, sizeof(systemParams) );

		systemParams.microsteps=atol(argv[0]);

		nvmWriteSystemParms(systemParams);
		stepperCtrl.updateParamsFromNVM();

		CommandPrintf(ptrUart,"microsteps %d\n\r",NVM->SystemParams.microsteps);

	}else
	{
		CommandPrintf(ptrUart,"number of microsteps must be a power of 2 between 1 and 256");
		return 1; //return error
	}

	return 0;
}


// print out the help strings for the commands
static int help_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	sCommand cmd_list;
	int i;

	//now let's parse the command
	i=0;
	memcpy(&cmd_list, &Cmds[i], sizeof(sCommand));
	while(cmd_list.function!=0)
	{

		CommandPrintf(ptrUart,(cmd_list.name));
		CommandPrintf(ptrUart,(" - "));
		CommandPrintf(ptrUart,(cmd_list.help));
		CommandPrintf(ptrUart,("\n\r"));
		i=i+1;
		memcpy(&cmd_list, &Cmds[i], sizeof(sCommand));
	}
	return 0;
}



static int getcal_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	stepperCtrl.calTable.printCalTable();
	return 0;
}

static int calibrate_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	stepperCtrl.calibrateEncoder();
	CommandPrintf(ptrUart,"Calibration DONE!\n\r");
	return 0;
}

static int testcal_cmd(sCmdUart *ptrUart,int argc, char * argv[])
{
	Angle a;
	int32_t x;

	a=stepperCtrl.maxCalibrationError();
	x=(uint16_t)a*(int32_t)360000L/(int32_t)ANGLE_MAX;

	CommandPrintf(ptrUart,"Max error is %d.%03d degrees\n\r", x/1000,abs(x)%1000);
	return 0;
}




uint8_t kbhit(void)
{
	return SerialUSB.available();
	//return SerialUSB.peek() != -1;
}
uint8_t getChar(void)
{
	return SerialUSB.read();
}
uint8_t putch(char data)
{
	return SerialUSB.write((uint8_t)data);
}


void commandsInit(void)
{
	CommandInit(&UsbUart, kbhit, getChar, putch ,NULL); //set up the UART structure
	SerialUSB.print("\n\rPower Up\n\r");
	SerialUSB.print(COMMANDS_PROMPT);
}

int commandsProcess(void)
{
	return CommandProcess(&UsbUart,Cmds,' ',COMMANDS_PROMPT);
}
