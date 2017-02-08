/**********************************************************************
 *      Author: tstern
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/
#include "stepper_controller.h"

#include "nonvolatile.h" //for programmable parameters
#include <Wire.h>

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

volatile bool TC5_ISR_Enabled=false;

void setupTCInterrupts() {



	// Enable GCLK for TC4 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
	WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
	WAIT_TC16_REGS_SYNC(TC5)

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
	WAIT_TC16_REGS_SYNC(TC5)

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
	WAIT_TC16_REGS_SYNC(TC5)


	TC5->COUNT16.CC[0].reg = F_CPU/NZS_CONTROL_LOOP_HZ;
	WAIT_TC16_REGS_SYNC(TC5)


	TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
	TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
	//  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0


	NVIC_SetPriority(TC5_IRQn, 1);


	// Enable InterruptVector
	NVIC_EnableIRQ(TC5_IRQn);


	// Enable TC
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	WAIT_TC16_REGS_SYNC(TC5)

}

static void enableTCInterrupts() {

	TC5_ISR_Enabled=true;
	NVIC_EnableIRQ(TC5_IRQn);
	TC5->COUNT16.INTENSET.bit.OVF = 1;
	//  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
	//  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}

static void disableTCInterrupts() {

	TC5_ISR_Enabled=false;
	//NVIC_DisableIRQ(TC5_IRQn);
	TC5->COUNT16.INTENCLR.bit.OVF = 1;
}

static bool enterCriticalSection()
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	return state;
}

static void exitCriticalSection(bool prevState)
{
	if (prevState)
	{
		enableTCInterrupts();
	} //else do nothing
}


void StepperCtrl::updateParamsFromNVM(void)
{
	bool state=enterCriticalSection();

	pPID.Kd=NVM->pPID.Kd*CTRL_PID_SCALING;
	pPID.Ki=NVM->pPID.Ki*CTRL_PID_SCALING;
	pPID.Kp=NVM->pPID.Kp*CTRL_PID_SCALING;

	vPID.Kd=NVM->vPID.Kd*CTRL_PID_SCALING;
	vPID.Ki=NVM->vPID.Ki*CTRL_PID_SCALING;
	vPID.Kp=NVM->vPID.Kp*CTRL_PID_SCALING;

	sPID.Kd=NVM->sPID.Kd*CTRL_PID_SCALING;
	sPID.Ki=NVM->sPID.Ki*CTRL_PID_SCALING;
	sPID.Kp=NVM->sPID.Kp*CTRL_PID_SCALING;

	if (NVM->SystemParams.parametersVaild)
	{
		memcpy((void *)&systemParams, (void *)&NVM->SystemParams, sizeof(systemParams));
	}else
	{
		ERROR("This should never happen but just in case");
		systemParams.microsteps=16;
		systemParams.controllerMode=CTRL_SIMPLE;
		systemParams.dirPinRotation=CW_ROTATION; //default to clockwise rotation when dir is high
		systemParams.errorLimit=(int32_t)ANGLE_FROM_DEGREES(1.8);
		systemParams.errorPinMode=ERROR_PIN_MODE_ENABLE;  //default to enable pin
	}

	//default the error pin to input, if it is an error pin the
	// handler for this will change the pin to be an output.
	// for bidirection error it has to handle input/output it's self as well.
	// This is not the cleanest way to handle this...
	// TODO implement this cleaner?
	pinMode(PIN_ERROR, INPUT_PULLUP); //we have input pin

	if (NVM->motorParams.parametersVaild)
	{
		memcpy((void *)&motorParams, (void *)&NVM->motorParams, sizeof(motorParams));
	} else
	{
		//MotorParams_t Params;
		motorParams.fullStepsPerRotation=200;
		motorParams.currentHoldMa=500;
		motorParams.currentMa=1000;
		motorParams.motorWiring=true;
		//memcpy((void *)&Params, (void *)&motorParams, sizeof(motorParams));
		//nvmWriteMotorParms(Params);
	}

	stepperDriver.setRotationDirection(motorParams.motorWiring);

	exitCriticalSection(state);
}


void  StepperCtrl::motorReset(void)
{
	//when we reset the motor we want to also sync the motor
	// phase.  Therefore we move forward a few full steps then back
	// to sync motor phasing, leaving the motor at "phase 0"
	bool state=enterCriticalSection();

	stepperDriver.move(0,motorParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS,motorParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*2,motorParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*3,motorParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*2,motorParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS,motorParams.currentMa);
	delay(100);
	stepperDriver.move(0,motorParams.currentMa);
	delay(500);

	setLocationFromEncoder(); //measure new starting point
	exitCriticalSection(state);
}

void StepperCtrl::setLocationFromEncoder(void)
{
	numSteps=0;
	currentLocation=0;

	if (calTable.calValid())
	{
		int32_t n,x;
		int32_t calIndex;
		Angle a;

		//set our angles based on previous cal data

		x=measureMeanEncoder();
		a=calTable.fastReverseLookup(x);

		//our cal table starts at angle zero, so lets set starting based on this and stepsize
		LOG("start angle %d, encoder %d", (uint16_t)a,x);

		//TODO we need to handle 0.9 degree motor
		if (CALIBRATION_TABLE_SIZE == motorParams.fullStepsPerRotation)
		{
			n=(int32_t)ANGLE_STEPS/CALIBRATION_TABLE_SIZE;

			calIndex=((int32_t)((uint16_t)a+n/2)*CALIBRATION_TABLE_SIZE)/ANGLE_STEPS; //find calibration index
			if (calIndex>CALIBRATION_TABLE_SIZE)
			{
				calIndex-=CALIBRATION_TABLE_SIZE;
			}
			a=(uint16_t)((calIndex*ANGLE_STEPS)/CALIBRATION_TABLE_SIZE);
		}


		x=(int32_t)((((float)(uint16_t)a)*360.0/(float)ANGLE_STEPS)*1000);
		LOG("start angle after rounding %d %d.%03d", (uint16_t)a,x/1000,x%1000);

		//we need to set our numSteps
		numSteps=DIVIDE_WITH_ROUND( ((int32_t)a *motorParams.fullStepsPerRotation*systemParams.microsteps),ANGLE_STEPS);
		currentLocation=(uint16_t)a;
	}
	zeroAngleOffset=getCurrentLocation();
}


void StepperCtrl::setZero(void)
{
	//we want to set the starting angle to zero.
	bool state=enterCriticalSection();

	zeroAngleOffset=getCurrentLocation();

	exitCriticalSection(state);
}

void StepperCtrl::encoderDiagnostics(char *ptrStr)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();

	encoder.diagnostics(ptrStr);

	if (state) enableTCInterrupts();
}


//TODO This function does two things, set rotation direction
//  and measures step size, it should be two functions.
//return is anlge in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl::measureStepSize(void)
{
	int32_t angle1,angle2,x,i;
	bool feedback=enableFeedback;
	int32_t microsteps=systemParams.microsteps;

	systemParams.microsteps=1;
	enableFeedback=false;
	motorParams.motorWiring=true; //assume we are forward wiring to start with
	stepperDriver.setRotationDirection(motorParams.motorWiring);
	/////////////////////////////////////////
	//// Measure the full step size /////
	/// Note we assume machine can take one step without issue///

	LOG("reset motor");
	motorReset(); //this puts stepper motor at stepAngle of zero

	LOG("sample encoder");

	angle1=measureMeanEncoder();

	LOG("move");
	stepperDriver.move(A4954_NUM_MICROSTEPS,motorParams.currentMa); //move one full step 'forward'
	LOG("sample encoder");
	angle2=measureMeanEncoder();

	LOG("Angles %d %d",angle1,angle2);
	if ((abs(angle2-angle1))>(ANGLE_STEPS/2))
	{
		//we crossed the wrap around
		if (angle1>angle2)
		{
			angle1=angle1+(int32_t)ANGLE_STEPS;
		}else
		{
			angle2=angle2+(int32_t)ANGLE_STEPS;
		}
	}
	LOG("Angles %d %d",angle1,angle2);

	//when we are increase the steps in the  stepperDriver.move() command
	// we want the encoder increasing. This ensures motor is moving clock wise
	// when encoder is increasing.
	//	if (angle2>angle1)
	//	{
	//		motorParams.motorWiring=true;
	//		stepperDriver.setRotationDirection(true);
	//		LOG("Forward rotating");
	//	}else
	//	{
	//		//the motor is wired backwards so correct in stepperDriver
	//		motorParams.motorWiring=false;
	//		stepperDriver.setRotationDirection(false);
	//		LOG("Reverse rotating");
	//	}
	x=((int64_t)(angle2-angle1)*36000)/(int32_t)ANGLE_STEPS;
	// if x is ~180 we have a 1.8 degree step motor, if it is ~90 we have 0.9 degree step
	LOG("%angle delta %d %d (%d %d)",x,abs(angle2-angle1),angle1,angle2 );

	systemParams.microsteps=microsteps;
	enableFeedback=feedback;

	return ((float)x)/100.0;
}


int32_t StepperCtrl::measureError(void)
{
	//LOG("current %d desired %d %d",(int32_t) currentLocation, (int32_t)getDesiredLocation(), numSteps);

	return ((int32_t)currentLocation-(int32_t)getDesiredLocation());
}

/*
bool StepperCtrl::changeMicrostep(uint16_t microSteps)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	systemParams.microsteps=microSteps;
	motorReset();
	if (state) enableTCInterrupts();
	return true;
}
 */
Angle StepperCtrl::maxCalibrationError(void)
{
	//Angle startingAngle;
	bool done=false;
	int32_t mean;
	int32_t maxError=0, j;
	int16_t dist;
	uint16_t angle=0;
	bool feedback=enableFeedback;
	uint16_t microSteps=systemParams.microsteps;
	int32_t steps;
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();


	if (false == calTable.calValid())
	{
		return ANGLE_MAX;
	}

	enableFeedback=false;
	j=0;
	LOG("Running calibration test");

	systemParams.microsteps=1;
	motorReset();
	steps=0;

	while(!done)
	{
		Angle cal;
		Angle act, desiredAngle;

		//todo we should measure mean and wait until stable.
		mean=measureMeanEncoder();
		desiredAngle=(uint16_t)(getDesiredLocation() & 0x0FFFFLL);

		cal=calTable.getCal(desiredAngle);
		dist=Angle(mean)-cal;
		act=calTable.fastReverseLookup(cal);

		LOG("actual %d, cal %d",mean,(uint16_t)cal);
		LOG("desired %d",(uint16_t)desiredAngle);
		LOG("numSteps %d", numSteps);

		LOG("cal error for step %d is %d, %d",j,dist,act-desiredAngle);
		LOG("mean %d, cal %d",mean, (uint16_t)cal);

		updateStep(0,1);

		steps+=A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,motorParams.currentMa);
		if (400==motorParams.fullStepsPerRotation)
		{
			updateStep(0,1);
			steps=steps+A4954_NUM_MICROSTEPS;
			stepperDriver.move(steps,motorParams.currentMa);
		}
		//delay(400);

		if (abs(dist)>maxError)
		{
			maxError=abs(dist);
		}

		j++;
		if (j>=(1*CALIBRATION_TABLE_SIZE+3))
		{
			done=true;
		}


	}
	systemParams.microsteps=microSteps;
	motorReset();
	enableFeedback=feedback;
	if (state) enableTCInterrupts();
	return Angle((uint16_t)maxError);
}


//The encoder needs to be calibrated to the motor.
// we will assume full step detents are correct,
// ex 1.8 degree motor will have 200 steps for 360 degrees.
// We also need to calibrate the phasing of the motor
// to the A4954. This requires that the A4954 "step angle" of
// zero is the first entry in the calibration table.
bool StepperCtrl::calibrateEncoder(void)
{
	int32_t x,i,j;
	uint32_t angle=0;
	int32_t steps;
	bool done=false;

	int32_t mean;
	uint16_t microSteps=systemParams.microsteps;
	bool feedback=enableFeedback;
	bool state=TC5_ISR_Enabled;

	disableTCInterrupts();

	enableFeedback=false;
	systemParams.microsteps=1;
	LOG("reset motor");
	motorReset();
	LOG("Starting calibration");
	delay(200);
	steps=0;
	j=0;
	while(!done)
	{
		Angle cal,desiredAngle;
		desiredAngle=(uint16_t)(getDesiredLocation() & 0x0FFFFLL);
		cal=calTable.getCal(desiredAngle);
		//delay(200);
		mean=measureMeanEncoder();

		LOG("Previous cal distance %d, %d, mean %d, cal %d",j, cal-Angle((uint16_t)mean), mean, (uint16_t)cal);

		calTable.updateTableValue(j,mean);

		updateStep(0,1);
		steps=steps+A4954_NUM_MICROSTEPS;
		//LOG("move %d %d",steps,motorParams.currentMa );
		stepperDriver.move(steps,motorParams.currentMa/2);
		if (400==motorParams.fullStepsPerRotation)
		{
			updateStep(0,1);
			steps=steps+A4954_NUM_MICROSTEPS;
			stepperDriver.move(steps,motorParams.currentMa/2);
		}

		j++;
		if (j>=CALIBRATION_TABLE_SIZE)
		{
			done=true;
		}


	}
	//calTable.printCalTable();
	calTable.smoothTable();
	//calTable.printCalTable();
	calTable.saveToFlash(); //saves the calibration to flash
	calTable.printCalTable();

	systemParams.microsteps=microSteps;
	motorReset();
	enableFeedback=feedback;
	if (state) enableTCInterrupts();
	return done;
}





stepCtrlError_t StepperCtrl::begin(void)
{
	int i;
	float x;


	enableFeedback=false;
	velocity=0;
	currentLocation=0;
	numSteps=0;

	//we have to update from NVM before moving motor
	updateParamsFromNVM(); //update the local cache from the NVM

	LOG("start stepper driver");
	stepperDriver.begin();

	LOG("start up encoder");
	if (false == encoder.begin(PIN_AS5047D_CS))
	{
		return STEPCTRL_NO_ENCODER;
	}

	LOG("cal table init");
	calTable.init();



	LOG("measuring step size");
	x=measureStepSize();
	if (abs(x)<0.5)
	{
		ERROR("Motor may not have power");
		return STEPCTRL_NO_POWER;
	}



	LOG("Checking the motor parameters");
	//todo we might want to move this up a level to the NZS
	//  especially since it has default values
	if (false == NVM->motorParams.parametersVaild)
	{
		MotorParams_t params;
		WARNING("NVM motor parameters are not set, we will update");

		//power could have just been applied and step size read wrong
		// if we are more than 200 steps/rotation which is most common
		// lets read again just to be sure.
		if (abs(x)<1.5)
		{
			//run step test a second time to be sure
			x=measureStepSize();
		}

		if (x>0)
		{
			motorParams.motorWiring=true;
		} else
		{
			motorParams.motorWiring=false;
		}
		if (abs(x)<=1.2)
		{
			motorParams.fullStepsPerRotation=400;
		}else
		{
			motorParams.fullStepsPerRotation=200;
		}

		memcpy((void *)&params, (void *)&motorParams,sizeof(motorParams));
		nvmWriteMotorParms(params);
	}

	LOG("Motor params are now good");
	LOG("fullSteps %d", motorParams.fullStepsPerRotation);
	LOG("motorWiring %d", motorParams.motorWiring);
	LOG("currentMa %d", motorParams.currentMa);
	LOG("holdCurrentMa %d", motorParams.currentHoldMa);


	updateParamsFromNVM(); //update the local cache from the NVM


	if (false == calTable.calValid())
	{
		return STEPCTRL_NO_CAL;
	}


	enableFeedback=true;
	setupTCInterrupts();
	enableTCInterrupts();
	return STEPCTRL_NO_ERROR;

}

Angle StepperCtrl::sampleAngle(void)
{
	uint16_t angle;

#ifdef NZS_AS5047_PIPELINE
	//read encoder twice such that we get the latest sample as the pipeline is always once sample behind
	encoder.readEncoderAnglePipeLineRead(); //convert the 14 bit encoder value to a 16 bit number
	angle=((uint32_t)encoder.readEncoderAnglePipeLineRead())<<2; //convert the 14 bit encoder value to a 16 bit number
#else
	angle=((uint32_t)encoder.readEncoderAngle())<<2; //convert the 14 bit encoder value to a 16 bit number
#endif
	return Angle(angle);
}

//when sampling the mean of encoder if we are on roll over
// edge we can have an issue so we have this function
// to do the mean correctly
Angle StepperCtrl::sampleMeanEncoder(uint32_t numSamples)
{
	Angle last, a;
	uint32_t i;
	int32_t mean=0;

	last=(Angle)(((uint32_t)encoder.readEncoderAngle())<<2);
	mean=(int32_t)last;
	for (i=0; i<(numSamples-1); i++)
	{
		int32_t d;
		d=last-(Angle)(((uint32_t)encoder.readEncoderAngle())<<2); //take the difference which handles roll over
		mean=mean+((int32_t)last+d);
	}
	return Angle(mean/numSamples);
}

void StepperCtrl::feedback(bool enable)
{
	disableTCInterrupts();
	motorReset();
	enableFeedback=enable;
	if (enable == true)
	{
		enableTCInterrupts();
	}
}

void StepperCtrl::updateStep(int dir, uint16_t steps)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	if (dir)
	{
		numSteps-=steps;
	}else
	{
		numSteps+=steps;
	}
	if (state) enableTCInterrupts();
}

void StepperCtrl::requestStep(int dir, uint16_t steps)
{
	bool state;
	state=TC5_ISR_Enabled;
	disableTCInterrupts();

	if (dir)
	{
		numSteps-=steps;
	}else
	{
		numSteps+=steps;
	}

	if (false == enableFeedback)
	{
		moveToAngle(getDesiredLocation(),motorParams.currentMa);
	}
	if (state) enableTCInterrupts();
}


void StepperCtrl::move(int dir, uint16_t steps)
{
	int64_t ret;
	int32_t n;



	updateStep(dir,steps);

	if (false == enableFeedback)
	{
		n=systemParams.microsteps;
		ret=((int64_t)numSteps * A4954_NUM_MICROSTEPS+(n/2))/n;
		n=A4954_NUM_MICROSTEPS*motorParams.fullStepsPerRotation;
		while(ret>n)
		{
			ret-=n;
		}
		while(ret<-n)
		{
			ret+=n;
		}
		n=(int32_t)(ret);
		LOG("s is %d %d",n,steps);
		stepperDriver.move(n,motorParams.currentMa);
	}


}



int64_t StepperCtrl::getDesiredLocation(void)
{
	int64_t ret;
	int32_t n;
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	n=motorParams.fullStepsPerRotation * systemParams.microsteps;
	ret=((int64_t)numSteps * (int64_t)ANGLE_STEPS+(n/2))/n ;
	if (state) enableTCInterrupts();
	return ret;
}

int64_t StepperCtrl::getDesiredSteps(void)//modOE
{
	return (int64_t)numSteps;
}
#define N_SAMPLES2 (500)
int32_t StepperCtrl::measureMeanEncoder(void)
{
	int i;
	uint32_t t0,t1;
	int32_t mean,first;

	//wait for at least 100,000 microseconds
	delay(100);

	mean=0;
	first=0;
	for (i=0; i<N_SAMPLES2; i++)
	{
		int32_t x;
		x=(int32_t)((uint16_t)encoder.readEncoderAngle())<<2; //convert the 14 bit encoder value to a 16 bit number
		if (encoder.getError())
		{

			SerialUSB.println("AS5047 Error");
			delay(1000);
			return 0;
		}
		if (i==0)
		{
			first=x;
		}
		if (abs(x-first)>65536/2)
		{
			if (x<first)
			{
				x=x+65536;
			}else
			{
				x=x-65536;
			}
		}
		mean+=x;
	}
	mean=mean/N_SAMPLES2;
	return mean;

}

/*
#define N_SAMPLES (1000)
int StepperCtrl::measure(void)
{
	uint16_t angle[N_SAMPLES];
	int i;
	uint32_t t0,t1;
	t0=micros();
	for (i=0; i<N_SAMPLES; i++)
	{
		angle[i]=((uint16_t)encoder.readEncoderAngle())<<2; //convert the 14 bit encoder value to a 16 bit number
	}
	t1=micros();
	LOG("delta Micros %u %u",t1-t0,t1);
	SerialUSB.print(t1-t0);
	for (i=0; i<N_SAMPLES; i++)
	{
		SerialUSB.print(',');
		SerialUSB.print(angle[i]);
		//sprintf(str,"%s,%u",str,angle[i]);
	}
	SerialUSB.print("\n\r");
	return 0;
}

 */


void StepperCtrl::moveToAbsAngle(int32_t a)
{

	int64_t ret;
	int32_t n;


	n=motorParams.fullStepsPerRotation * systemParams.microsteps;

	ret=(((int64_t)a+zeroAngleOffset)*n)/(int32_t)ANGLE_STEPS;
	bool state=enterCriticalSection();
	numSteps=ret;
	exitCriticalSection(state);
}

void StepperCtrl::moveToAngle(int32_t a, uint32_t ma)
{
	//we need to convert 'Angle' to A4954 steps
	a=a % ANGLE_STEPS;  //we only interested in the current angle


	a=DIVIDE_WITH_ROUND( (a*motorParams.fullStepsPerRotation*A4954_NUM_MICROSTEPS), ANGLE_STEPS);

	//LOG("move %d %d",a,ma);
	stepperDriver.move(a,ma);

}


int64_t StepperCtrl::getCurrentLocation(void)
{
	Angle a;
	int32_t x;
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	a=calTable.fastReverseLookup(sampleAngle());
	x=(int32_t)a - (int32_t)((currentLocation) & ANGLE_MAX);

	if (x>((int32_t)ANGLE_STEPS/2))
	{
		currentLocation -= ANGLE_STEPS;
	}
	if (x<-((int32_t)ANGLE_STEPS/2))
	{
		currentLocation += ANGLE_STEPS;
	}
	currentLocation=(currentLocation & 0xFFFFFFFFFFFF0000UL) | (uint16_t)a;
	if (state) enableTCInterrupts();
	return currentLocation;

}

int64_t StepperCtrl::getCurrentAngle(void)
{
	int64_t x;
	x=getCurrentLocation()-zeroAngleOffset;
	return x;
}


int64_t StepperCtrl::getDesiredAngle(void)
{
	int64_t x;
	x=getDesiredLocation()-zeroAngleOffset;
	return x;
}

void StepperCtrl::setVelocity(int64_t vel)
{
	bool state=enterCriticalSection();

	velocity=vel;
	exitCriticalSection(state);
}

int64_t StepperCtrl::getVelocity(void)
{
	int64_t vel;
	bool state=enterCriticalSection();

	vel=velocity;
	exitCriticalSection(state);
	return vel;
}

//this is the velocity PID feedback loop
bool StepperCtrl::vpidFeedback(void)
{
	int32_t fullStep=ANGLE_STEPS/motorParams.fullStepsPerRotation;
	static int64_t lastY=getCurrentLocation();
	static int32_t lastError=0;
	static int64_t Iterm=0;
	int64_t y,z;
	int32_t v,dy;
	int64_t u;

	//get the current location
	y =getCurrentLocation();

	v=y-lastY;

	dy=(y-lastY);

	z=y;

#ifdef ENABLE_PHASE_PREDICTION
	z=y+dy; //predict where we are for phase advancement
//	if (ABS(dy)>fullStep/2 			          //we have moved 50% of max speed
//		&& ABS(dy)<ANGLE_FROM_DEGREES(5.0)	) //but not so fast it could be an error
//	{
//		z=y+dy; //predict where we are for phase advancement
//	}
#endif


	lastY=y;

	v=v*NZS_CONTROL_LOOP_HZ;


	if (enableFeedback) //if ((micros()-lastCall)>(updateRate/10))
	{
		int32_t error,U;
		error = velocity-v;


		Iterm += (vPID.Ki * error);
		if (Iterm>(16*4096*CTRL_PID_SCALING *motorParams.currentMa))
		{
			Iterm=(16*4096*CTRL_PID_SCALING *motorParams.currentMa);
		}
		if (Iterm<-(16*4096*CTRL_PID_SCALING *motorParams.currentMa))
		{
			Iterm=-(16*4096*CTRL_PID_SCALING*motorParams.currentMa);
		}

		u=((vPID.Kp * error) + Iterm - (vPID.Kd *(lastError-error)));
		U=abs(u)/CTRL_PID_SCALING/1024; //scale the error to make PID params close to 1.0;//scale the error to make PID params close to 1.0 by dividing by 1024

		if (U>motorParams.currentMa)
		{
			U=motorParams.currentMa;
		}




		//when error is positive we need to move reverse direction
		if (u>0)
		{
			z=z+(fullStep);
		}else
		{
			z=z-(fullStep);

		}

		moveToAngle(z,U);
		loopError=error;
		lastError=error;
	} else
	{
		lastError=0;
		Iterm=0;
	}

	if (abs(lastError)>(systemParams.errorLimit))
	{
		return 1;
	}
	return 0;
}


//Since we are doing fixed point math our
// threshold needs to be large.
// We need a large threshold when we have fast update
// rate as well. But for most part it is random
bool StepperCtrl::pidFeedback(void)
{
	static int count=0;

	static int32_t maxError=0;
	static int32_t lastError=0;
	static int32_t Iterm=0;
	int64_t y;
	static int64_t lastY=getCurrentLocation();
	int32_t fullStep=ANGLE_STEPS/motorParams.fullStepsPerRotation;
	int32_t dy;

	y=getCurrentLocation();
	dy=y-lastY;
	lastY=y;

#ifdef ENABLE_PHASE_PREDICTION
	y=y+dy; //predict that our new position
//	if (ABS(dy)>fullStep/2 			          //we have moved 50% of max speed
//		&& ABS(dy)<ANGLE_FROM_DEGREES(5.0)	) //but not so fast it could be an error
//	{
//		y=y+dy; //predict that our new position
//	}
#endif

	if (enableFeedback) //if ((micros()-lastCall)>(updateRate/10))
	{
		int64_t error,u;
		int32_t U,x;

		//error is in units of degrees when 360 degrees == 65536
		error=(getDesiredLocation()-y); //error is currentPos-desiredPos

		Iterm+=(pPID.Ki * error);

		//Over the long term we do not want error
		// to be much more than our threshold
		if (Iterm> (motorParams.currentMa*CTRL_PID_SCALING) )
		{
			Iterm=(motorParams.currentMa*CTRL_PID_SCALING) ;
		}
		if (Iterm<-(motorParams.currentMa*CTRL_PID_SCALING)  )
		{
			Iterm=-(motorParams.currentMa*CTRL_PID_SCALING) ;
		}

		u=((pPID.Kp * error) + Iterm - (pPID.Kd *(lastError-error)));

		U=abs(u)/CTRL_PID_SCALING;
		if (U>motorParams.currentMa)
		{
			U=motorParams.currentMa;
		}

		//when error is positive we need to move reverse direction
		if (u>0)
		{
			y=y+fullStep;
		}else
		{
			y=y-fullStep;

		}

		moveToAngle(y,U);
		loopError=error;
		lastError=error;

	}else
	{
		lastError=0;
		Iterm=0;
	}

	if (abs(lastError)>(systemParams.errorLimit))
	{
		return 1;
	}
	return 0;
}

//this was written to do the PID loop not modeling a DC servo
// but rather using features of stepper motor.
bool StepperCtrl::simpleFeedback(void)
{
	static uint32_t t0=0;
	static uint32_t calls=0;

	static int32_t maxError=0;
	static int32_t lastError=0;
	static int32_t i=0;
	static int32_t iTerm=0;
	static int64_t lastY=getCurrentLocation();
	static int32_t velocity=0;

	int32_t fullStep=ANGLE_STEPS/motorParams.fullStepsPerRotation;


	int64_t y;
	int32_t dy;

	//estimate our current location based on the encoder
	y=getCurrentLocation();
	dy=y-lastY;
	lastY=y;

#ifdef ENABLE_PHASE_PREDICTION
	y=y+dy; //predict that our new position
//	if (ABS(dy)>fullStep/2 			          //we have moved 50% of max speed
//		&& ABS(dy)<ANGLE_FROM_DEGREES(5.0)	) //but not so fast it could be an error
//	{
//		y=y+dy; //predict where we are for phase advancement
//	}
#endif


	//we can limit the velocity by controlling the amount we move per call to this function
	// this only works for velocity greater than 100rpm
	/*	if (velocity!=0)
	{
		fullStep=velocity/NZS_CONTROL_LOOP_HZ;
	}
	if (fullStep==0)
	{
		fullStep=1; //this RPM of (1*NZS_CONTROL_LOOP_HZ)/60 ie at 6Khz it is 100RPM
	}
	 */
	if (enableFeedback)
	{
		int64_t error;
		int32_t u;
		int32_t ma;

		int32_t x;

		//error is in units of degrees when 360 degrees == 65536
		error=(getDesiredLocation()-y);//measureError(); //error is currentPos-desiredPos


		data[i]=(int16_t)error;
		i++;
		if (i>=N_DATA)
		{
			i=0;
		}

		if (abs(error)<fullStep)
		{
			iTerm+=(sPID.Ki*error);
			x=iTerm/CTRL_PID_SCALING;
		}else
		{
			x=0;
			iTerm=0;
		}

		if (x>fullStep)
		{
			x=fullStep;
		}
		if (x<-fullStep)
		{
			x=-fullStep;
		}

		u=(sPID.Kp * error)/CTRL_PID_SCALING+x+(sPID.Kd *(error-lastError))/CTRL_PID_SCALING;


		//limit error to full step
		if (u>fullStep)
		{
			u=fullStep;
		}
		if (u<-fullStep)
		{
			u=-fullStep;
		}

		ma=(abs(u)*(motorParams.currentMa-motorParams.currentHoldMa))
								/ fullStep + motorParams.currentHoldMa;

		//ma=(abs(u)*(NVM->SystemParams.currentMa))/fullStep;

		if (ma>motorParams.currentMa)
		{
			ma=motorParams.currentMa;
		}



		y=y+u;
		moveToAngle(y,ma); //35us

		lastError=error;
		loopError=error;
		//stepperDriver.limitCurrent(99);
	}

	if (abs(lastError)>(systemParams.errorLimit))
	{
		return 1;
	}
	stepperDriver.limitCurrent(99); //reduce noise on low error
	return 0;
}


void StepperCtrl::enable(bool enable)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	bool feedback=enableFeedback;

	stepperDriver.enable(enable); //enable or disable the stepper driver as needed


	if (enabled==true && enable==false)
	{
		feedback = false;
	}
	if (enabled==false && enable==true) //if we are enabling previous disabled motor
	{
		feedback = true;
		setLocationFromEncoder();
	}

	enabled=enable;
	enableFeedback=feedback;
	if (state) enableTCInterrupts();
}

/*
void StepperCtrl::testRinging(void)
{
	uint16_t c;
	int32_t steps;
	int32_t microSteps=systemParams.microsteps;
	bool feedback=enableFeedback;

	enableFeedback=false;
	systemParams.microsteps=1;
	motorReset();
	for (c=2000; c>10; c=c-10)
	{
		SerialUSB.print("Current ");
		SerialUSB.println(c);
		steps+=A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,NVM->SystemParams.currentMa);
		currentLimit=false;
		measure();
	}
	systemParams.microsteps=microSteps;
	motorReset();
	enableFeedback=feedback;
}
 */

bool StepperCtrl::processFeedback(void)
{
	bool ret;
	int32_t us;
	us=micros();
	switch (systemParams.controllerMode)
	{
		case CTRL_POS_PID:
		{
			ret=pidFeedback();
			break;
		}
		default:
		case CTRL_SIMPLE:
		{
			ret=simpleFeedback();
			break;
		}
		case CTRL_POS_VELOCITY_PID:
		{
			ret=vpidFeedback();
			break;
		}
		//TODO if disable feedback and someone switches mode
		// they will have to turn feedback back on.
		case CTRL_OFF:
		{
			enableFeedback=false;
			break;
		}
		case CTRL_OPEN:
		{
			enableFeedback=false;
			break;
		}
	}
	loopTimeus=micros()-us;
	return ret;
}


//auto tuning of PID parameters based on documentation here:
// http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library
// http://www.controleng.com/search/search-single-display/relay-method-automates-pid-loop-tuning/4a5774decc.html
void StepperCtrl::PID_Autotune(void)
{
	int32_t noiseMin, noiseMax, error;
	int32_t eMin, eMax;
	int64_t mean;
	int32_t startAngle, thres;
	int32_t i,j;
	int32_t t0,t1;
	int32_t times[100];
	int32_t angle;

	//save previous state;
	uint16_t microSteps=systemParams.microsteps;
	bool feedback=enableFeedback;
	feedbackCtrl_t prevCtrl=systemParams.controllerMode;

	//disable interrupts and feedback controller
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	systemParams.controllerMode=CTRL_POS_PID;
	enableFeedback=false;
	motorReset();
	//nvmWritePID(1,0,0,2);
	//set the number of microsteps to 1
	//systemParams.microsteps=1;
	for (i=0; i<10; i++)
	{
		angle=getCurrentLocation();
	}
	//	pKp=NVM->PIDparams.Kp;
	//	pKi=NVM->PIDparams.Ki;
	//	pKd=NVM->PIDparams.Kd;
	//	threshold=NVM->PIDparams.Threshold;

	//enableTCInterrupts();
	moveToAngle(angle,motorParams.currentMa);


	//moveToAngle(angle,NVM->SystemParams.currentMa);
	/*
	//next lets measure our noise on the encoder
	noiseMin=(int32_t)ANGLE_MAX;
	noiseMax=-(int32_t)ANGLE_MAX;
	mean=0;
	j=1000000UL/NZS_CONTROL_LOOP_HZ;
	prevAngle=sampleAngle();
	for (i=0; i<(NZS_CONTROL_LOOP_HZ/2); i++)
	{
		Angle a;
		a=sampleAngle();
		error=(int32_t)(prevAngle-a);

		if (error<noiseMin)
		{
			noiseMin=error;
		}

		if (error>noiseMax)
		{
			noiseMax=error;
		}
		mean=mean+(int32_t)a;
		delayMicroseconds(j);

	}
	mean=mean/i;
	while (mean>ANGLE_MAX)
	{
		mean=mean-ANGLE_STEPS;
	}
	while (mean<0)
	{
		mean=mean+ANGLE_STEPS;
	}
	//mean is the average of the encoder.
	 */



	stepperDriver.move(0,motorParams.currentMa);
	delay(1000);

	//now we need to do the relay control
	for (i=0; i<10; i++)
	{
		startAngle=getCurrentLocation();
		LOG("Start %d", (int32_t)startAngle);
	}
	thres=startAngle + (int32_t)((ANGLE_STEPS/motorParams.fullStepsPerRotation)*10/9);
	LOG("Thres %d, start %d",(int32_t)thres,(int32_t)startAngle);
	eMin=(int32_t)ANGLE_MAX;
	eMax=-(int32_t)ANGLE_MAX;
	int32_t reset=0;
	int32_t force=(motorParams.currentMa);

	for (i=0; i<100; i++)
	{
		int32_t error;
		if (reset)
		{
			motorReset();
			stepperDriver.move(0,motorParams.currentMa);
			delay(1000);
			startAngle=getCurrentLocation();
			LOG("Start %d", (int32_t)startAngle);
			force=force-100;

			eMin=(int32_t)ANGLE_MAX;
			eMax=-(int32_t)ANGLE_MAX;

			if (force<100)
			{
				i=100;
				break;
			}
			LOG("force set to %d",force);
			i=0;
		}
		reset=0;

		stepperDriver.move(A4954_NUM_MICROSTEPS,force);
		//moveToAngle(startAngle+(ANGLE_STEPS/motorParams.fullStepsPerRotation),force);
		//stepperDriver.move(A4954_NUM_MICROSTEPS,NVM->SystemParams.currentMa);
		t0=micros();

		error=0;
		while(error<=((ANGLE_STEPS/motorParams.fullStepsPerRotation)/2+40))
		{
			int32_t y;
			y=getCurrentLocation();
			error=y-startAngle;
			//LOG("Error1 %d",error);
			if (error<eMin)
			{
				eMin=error;
			}
			if (error>eMax)
			{
				eMax=error;
			}
			if (abs(error)>ANGLE_STEPS/motorParams.fullStepsPerRotation*2)
			{
				LOG("large Error1 %d, %d, %d",error, y, startAngle);

				reset=1;
				break;
			}

		}

		stepperDriver.move(0,force);

		//stepperDriver.move(0,NVM->SystemParams.currentMa);
		t1=micros();

		error=(ANGLE_STEPS/motorParams.fullStepsPerRotation);
		while(error>=((ANGLE_STEPS/motorParams.fullStepsPerRotation)/2-40))
		{
			error=getCurrentLocation()-startAngle;
			//LOG("Error2 %d",error);
			if (error<eMin)
			{
				eMin=error;
			}
			if (error>eMax)
			{
				eMax=error;
			}
			if (abs(error)>ANGLE_STEPS/motorParams.fullStepsPerRotation*2)
			{
				LOG("large Error2 %d",error);
				reset=1;
				break;
			}
		}

		times[i]=t1-t0;

	}
	for (i=0; i<100; i++)
	{
		LOG("Time %d %d",i,times[i]);
	}
	LOG("errorMin=%d",eMin);
	LOG("errorMax=%d",eMax);

	motorReset();
	systemParams.controllerMode=prevCtrl;
	systemParams.microsteps=microSteps;
	enableFeedback=feedback;
	if (state) enableTCInterrupts();

}

void StepperCtrl::printData(void)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	int32_t i;
	for(i=0; i<N_DATA; i++)
	{
		LOG ("%d\n",data[i]);
	}

	if (state) enableTCInterrupts();

}

#pragma GCC pop_options
