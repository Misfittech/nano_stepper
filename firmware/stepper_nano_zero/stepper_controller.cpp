#include "stepper_controller.h"
#ifndef MECHADUINO_HARDWARE
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#endif 


#include "nonvolatile.h" //for programmable parameters
#include <Wire.h>

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


	TC5->COUNT16.CC[0].reg = 48000000UL/NZS_CONTROL_LOOP_HZ;
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

void enableTCInterrupts() {

	TC5_ISR_Enabled=true;
	NVIC_EnableIRQ(TC5_IRQn);
	TC5->COUNT16.INTENSET.bit.OVF = 1;
	//  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
	//  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}

void disableTCInterrupts() {

	TC5_ISR_Enabled=false;
	//NVIC_DisableIRQ(TC5_IRQn);
	TC5->COUNT16.INTENCLR.bit.OVF = 1;
}

void StepperCtrl::updatePIDs(void)
{
	pPID.Kd=NVM->pPID.Kd*CTRL_PID_SCALING;
	pPID.Ki=NVM->pPID.Ki*CTRL_PID_SCALING;
	pPID.Kp=NVM->pPID.Kp*CTRL_PID_SCALING;

	vPID.Kd=NVM->vPID.Kd*CTRL_PID_SCALING;
	vPID.Ki=NVM->vPID.Ki*CTRL_PID_SCALING;
	vPID.Kp=NVM->vPID.Kp*CTRL_PID_SCALING;

	sPID.Kd=NVM->sPID.Kd*CTRL_PID_SCALING;
	sPID.Ki=NVM->sPID.Ki*CTRL_PID_SCALING;
	sPID.Kp=NVM->sPID.Kp*CTRL_PID_SCALING;

}


void  StepperCtrl::motorReset(void)
{
	//when we reset the motor we want to also sync the motor
	// phase.  Therefore we move forward a few full steps then back
	// to sync motor phasing, leaving the motor at "phase 0"
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();

	stepperDriver.move(0,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*2,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*3,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS*2,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(A4954_NUM_MICROSTEPS,NVM->SystemParams.currentMa);
	delay(100);
	stepperDriver.move(0,NVM->SystemParams.currentMa);
	delay(500);

	setLocationFromEncoder(); //measure new starting point
	if (state) enableTCInterrupts();
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
		if (CALIBRATION_TABLE_SIZE == fullStepsPerRotation)
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
		numSteps=DIVIDE_WITH_ROUND( ((int32_t)a *fullStepsPerRotation*numMicroSteps),ANGLE_STEPS);
		currentLocation=(uint16_t)a;
	}
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
uint16_t StepperCtrl::measureStepSize(void)
{
	int32_t angle1,angle2,x,i;
	bool feedback=enableFeedback;
	int32_t microsteps=numMicroSteps;

	numMicroSteps=1;
	enableFeedback=false;
	forwardWiring=true; //assume we are forward wiring to start with
	/////////////////////////////////////////
	//// Measure the full step size /////
	/// Note we assume machine can take one step without issue///

	motorReset(); //this puts stepper motor at stepAngle of zero


	angle1=measureMeanEncoder();
	stepperDriver.move(A4954_NUM_MICROSTEPS,NVM->SystemParams.currentMa); //move one full step 'forward'
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
	if (angle2>angle1)
	{
		forwardWiring=true;
		stepperDriver.setRotationDirection(true);
		LOG("Forward rotating");
	}else
	{
		//the motor is wired backwards so correct in stepperDriver
		forwardWiring=false;
		stepperDriver.setRotationDirection(false);
		LOG("Reverse rotating");
	}
	x=(abs(angle2-angle1)*36000)/(int32_t)ANGLE_STEPS;
	// if x is ~180 we have a 1.8 degree step motor, if it is ~90 we have 0.9 degree step
	LOG("%angle delta %d %d (%d %d)",x,abs(angle2-angle1),angle1,angle2 );

	numMicroSteps=microsteps;
	enableFeedback=feedback;

	return x;
}


int32_t StepperCtrl::measureError(void)
{
	//LOG("current %d desired %d %d",(int32_t) currentLocation, (int32_t)getDesiredLocation(), numSteps);

	return ((int32_t)currentLocation-(int32_t)getDesiredLocation());
}

bool StepperCtrl::changeMicrostep(uint16_t microSteps)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	numMicroSteps=microSteps;
	motorReset();
	if (state) enableTCInterrupts();
	return true;
}


Angle StepperCtrl::maxCalibrationError(void)
{
	//Angle startingAngle;
	bool done=false;
	int32_t mean;
	int32_t maxError=0, j;
	int16_t dist;
	uint16_t angle=0;
	bool feedback=enableFeedback;
	uint16_t microSteps=numMicroSteps;
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

	numMicroSteps=1;
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
		stepperDriver.move(steps,NVM->SystemParams.currentMa);
		if (400==fullStepsPerRotation)
		{
			updateStep(0,1);
			steps=steps+A4954_NUM_MICROSTEPS;
			stepperDriver.move(steps,NVM->SystemParams.currentMa);
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
	numMicroSteps=microSteps;
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
	uint16_t microSteps=numMicroSteps;
	bool feedback=enableFeedback;
	bool state=TC5_ISR_Enabled;

	disableTCInterrupts();

	enableFeedback=false;
	numMicroSteps=1;
	LOG("reset motor");
	motorReset();
	LOG("Starting calibration");
	steps=0;

	LOG("Starting calibration");
	j=0;
	while(!done)
	{
		Angle cal,desiredAngle;
		desiredAngle=(uint16_t)(getDesiredLocation() & 0x0FFFFLL);
		cal=calTable.getCal(desiredAngle);
		mean=measureMeanEncoder();

		LOG("Previous cal distance %d, %d, mean %d, cal %d",j, cal-Angle((uint16_t)mean), mean, (uint16_t)cal);

		calTable.updateTableValue(j,mean);

		updateStep(0,1);
		steps=steps+A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,NVM->SystemParams.currentMa);
		if (400==fullStepsPerRotation)
		{
			updateStep(0,1);
			steps=steps+A4954_NUM_MICROSTEPS;
			stepperDriver.move(steps,NVM->SystemParams.currentMa);
		}

		j++;
		if (j>=CALIBRATION_TABLE_SIZE)
		{
			done=true;
		}


	}
	calTable.saveToFlash(); //saves the calibration to flash
	calTable.printCalTable();

	numMicroSteps=microSteps;
	motorReset();
	enableFeedback=feedback;
	if (state) enableTCInterrupts();
	return done;
}

void StepperCtrl::LCDShow(char *str)
{
#ifdef MECHADUINO_HARDWARE //mech does not have LCD
	return;
#else
	clearAndShowHeader();
	display.setTextSize(DISPLAY_TEXT_SIZE);
	display.setTextColor(WHITE);
	display.setCursor(0,DISPLAY_LINE(0));
	display.println(str);
#endif
}

void StepperCtrl::UpdateLcd(void)
{
#ifdef MECHADUINO_HARDWARE //mech does not have LCD
	return;
#else
	char str[100];
	static int highRPM=0;
	int32_t deg,x,y,err;

	int32_t lastAngle=0;
	static int32_t RPM=0;
	int32_t lasttime=0;
	int32_t d;
	bool state;

	clearAndShowHeader();


	//sprintf(str,"uSteps %d", numMicroSteps);

	display.setTextSize(DISPLAY_TEXT_SIZE);
	display.setTextColor(WHITE);

	state=TC5_ISR_Enabled;
	disableTCInterrupts();
	lastAngle=getDesiredLocation();
	lasttime=millis();
	if (state) enableTCInterrupts();
	delay(100);

	state=TC5_ISR_Enabled;
	disableTCInterrupts();
	deg=getDesiredLocation();
	err=measureError();
	y=millis()-lasttime;
	if (state) enableTCInterrupts();



	d=(int32_t)(Angle(Angle(lastAngle)-Angle(deg)));

	while (d>ANGLE_STEPS/2)
	{
		d=ANGLE_STEPS-d;
	}

	while (d<-ANGLE_STEPS/2)
	{
		d=ANGLE_STEPS+d;
	}

	x=(d*(60*1000UL))/(y * ANGLE_STEPS);

	//filter out errors which have high RPMs
	if ((x-RPM)>500 && highRPM==0)
	{
		LOG("high RPMs, %d %d %d %d %d", y, lastAngle, deg,x,d);
		highRPM=1;
		x=0;
	}else
	{
		highRPM=0;
	}
	lastAngle=deg;
	RPM=x; //(7*RPM+x)/8; //average RPMs
	if (enableFeedback)
	{
		if (controlType==CTRL_SIMPLE)
		{
			sprintf(str, "%dRPM simp",RPM);

		}else if(controlType==CTRL_POS_PID)
		{
			sprintf(str, "%dRPM pPID",RPM);
		}
	}
	if ((false==enableFeedback) || (controlType==CTRL_OFF))
	{
		sprintf(str, "%dRPM open",RPM);
	}

	display.setCursor(0,DISPLAY_LINE(0));
	display.println(str);


	//LOG("error is %d",err);


	err=(err*360*100)/(int32_t)ANGLE_STEPS;
	x=err/100;
	y=abs(err-x*100);

	sprintf(str,"%01d.%02d err", x,y);
	display.setCursor(0,DISPLAY_LINE(1));
	display.println(str);
	//LOG("%s %d %d %d", str, err, x, y);
	//
	//	sprintf(str,"%01d.%02d Amps", NVM->SystemParams.currentMa/1000, NVM->SystemParams.currentMa%1000);
	//	display.setCursor(0,20);
	//	display.println(str);
#ifndef NZS_LCD_ABSOULTE_ANGLE
	deg=deg & ANGLE_MAX; //limit to 360 degrees
#endif

	deg=(deg*360*10)/(int32_t)ANGLE_STEPS;
	x=(deg)/10;
	y=abs(deg-(x*10));

	//LOG("deg is %d, %d, %d",deg, x, y);
	sprintf(str,"%03d.%01ddeg", x,y);
	display.setCursor(0,DISPLAY_LINE(2));
	display.println(str);

	display.display();

#endif //not mechaduino
}

//does the LCD menu system
void StepperCtrl::menu(void)
{
#ifdef MECHADUINO_HARDWARE //mech does not have LCD
	return;
#else
	bool done=false;
	int menuItem=0;
	char str[100];
	int sw1State=0;
	int sw3State=0;

	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_SW4, INPUT_PULLUP);


	while (!done)
	{
		clearAndShowHeader();
		display.setTextSize(DISPLAY_TEXT_SIZE);
		display.setTextColor(WHITE);

		if (menuItem==0)
		{
			sprintf(str,"*Run Cal");
			display.setCursor(0,DISPLAY_LINE(0));
			display.println(str);
		}else
		{
			sprintf(str," Run Cal");
			display.setCursor(0,DISPLAY_LINE(0));
			display.println(str);
		}

		if (menuItem==1)
		{
			sprintf(str,"*Check Cal");
			display.setCursor(0,DISPLAY_LINE(1));
			display.println(str);
		}else
		{
			sprintf(str," Check Cal");
			display.setCursor(0,DISPLAY_LINE(1));
			display.println(str);
		}

		if (menuItem==2)
		{
			sprintf(str,"*Exit");
			display.setCursor(0,DISPLAY_LINE(2));
			display.println(str);
		}else
		{
			sprintf(str," Exit");
			display.setCursor(0,DISPLAY_LINE(2));
			display.println(str);
		}

		display.display();

		if (sw1State==1)
		{
			while (digitalRead(PIN_SW1)==0);
			sw1State=0;
		}

		if (digitalRead(PIN_SW1)==0)
		{
			sw1State=1;
			menuItem=(menuItem+1)%3;
		}

		if (sw3State==1)
		{
			while (digitalRead(PIN_SW3)==0);
			sw3State=0;
		}

		if (digitalRead(PIN_SW3)==0)
		{
			sw3State=1;
			switch(menuItem)
			{
				case 0:
					clearAndShowHeader();
					display.setTextSize(DISPLAY_TEXT_SIZE);
					display.setTextColor(WHITE);
					display.setCursor(0,DISPLAY_LINE(0));
					display.println("Running");
					display.setCursor(0,DISPLAY_LINE(1));
					display.println("Cal");
					display.display();
					calibrateEncoder();
					break;
				case 1:
				{
					clearAndShowHeader();
					display.setTextSize(DISPLAY_TEXT_SIZE);
					display.setTextColor(WHITE);
					display.setCursor(0,DISPLAY_LINE(0));
					display.println("Testing");
					display.setCursor(0,DISPLAY_LINE(1));
					display.println("Cal");
					display.display();
					int32_t error,x,y,m;
					error=maxCalibrationError();
					x=(error*100 *360)/ANGLE_STEPS;
					m=x/100;
					y=abs(x-(m*100));
					clearAndShowHeader();
					display.setTextSize(DISPLAY_TEXT_SIZE);
					display.setTextColor(WHITE);
					display.setCursor(0,DISPLAY_LINE(0));
					display.println("Error");

					sprintf(str, "%02d.%02d deg",m,y);
					display.setCursor(0,DISPLAY_LINE(1));
					display.println(str);
					display.display();
					while (digitalRead(PIN_SW3));
					break;
				}
				case 2:
					return;
					break;

			}

		}

	}
#endif
}

void StepperCtrl::showCalError(void)
{
#ifdef MECHADUINO_HARDWARE
  return;
#else
	char str[100];

	clearAndShowHeader();
	display.setTextSize(DISPLAY_TEXT_SIZE);
	display.setTextColor(WHITE);

	sprintf(str,"Calibration");
	display.setCursor(0,DISPLAY_LINE(0));
	display.println(str);
	sprintf(str,"Error");
	display.setCursor(0,DISPLAY_LINE(1));
	display.println(str);
	display.display();
#endif
}

void StepperCtrl::showSplash(void)
{
#ifdef MECHADUINO_HARDWARE //mech does not have LCD
	return;
#else
	char str[100];

	display.clearDisplay();
	display.setTextSize(DISPLAY_TEXT_SIZE);
	display.setTextColor(WHITE);

	sprintf(str,"Misfit Tech");
	display.setCursor(0,DISPLAY_LINE(0));
	display.println(str);
  sprintf(str,"http://misfittech.net");
  display.setCursor(0,DISPLAY_LINE(1));
  display.println(str);
	sprintf(str,"Nano Zero Stepper");
	display.setCursor(0,DISPLAY_LINE(2));
	display.println(str);
	display.setCursor(0,DISPLAY_LINE(3));
	display.println(VERSION);
	display.display();
#endif //no mechaduino
}

void StepperCtrl::clearAndShowHeader(void)
{
#ifdef MECHADUINO_HARDWARE //mech does not have LCD
  return;
#else
  char str[100];

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  sprintf(str,"Misfit Tech");
  display.setCursor(0,0);
  display.println(str);
  sprintf(str,"Nano Zero Stepper");
  display.setCursor(0,8);
  display.println(str);

  
  display.setCursor(72,0);
  display.print(VERSION);
#endif //no mechaduino
}


int StepperCtrl::begin(void)
{
	int i;
	int32_t x;

	enableFeedback=true;
	forwardWiring=true; //assume we are forward wiring to start with

	degreesPerFullStep=0;
	currentLocation=0;
	numSteps=0;
	currentLimit=false;

#ifndef MECHADUINO_HARDWARE //mech does not have LCD
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	Wire.setClock(800000);
	showSplash();
#endif


	stepperDriver.begin();
	//stepperDriver.limitCurrent(99);
	fullStepsPerRotation=200;

	LOG("NVM calvalid %d", NVM->CalibrationTable.status);
	LOG("NVM cal[0] %d", NVM->CalibrationTable.table[0]);

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
		nvmWrite_vPID(1.0, 0, 0);
	}


	updatePIDs(); //update the local cache from the NVM

	if (false == NVM->SystemParams.parametersVaild)
	{
		nvmWriteSystemParms(2500,1500,327);
	}
	//UpdateLcd();

	encoder.begin(PIN_AS5047D_CS);
	calTable.init();


	changeMicrostep(1);
	x=measureStepSize();

	if (x>050 && x<130)
	{
		degreesPerFullStep= ANGLE_FROM_DEGREES(0.9);
		fullStepsPerRotation=400;
		LOG("Motor appears to be 0.9 degree per step");
	}else
	{
		//assume we have a 1.8 degree motor (which is most common for 3D printers)
		degreesPerFullStep= ANGLE_FROM_DEGREES(1.8);
		fullStepsPerRotation=200;
		LOG("Motor appears to be 1.8 degree per step");
	}

#ifndef MECHADUINO_HARDWARE //mech does not have LCD

	while (calTable.calValid() == false)
	{
		enableFeedback=false;
		showCalError(); //show calibration error
		delay(1000); //wait so user sees it
		menu(); //run the menu as long as calibration is not valid
	}
#endif

	//verify the calbiration is good which forces a flash update
	if (calTable.calValid() == false)
	{
		SerialUSB.println("Not Calibrated");
		ERROR("Not calibrated");
		enableFeedback=false;
	} else
	{
		enableFeedback=true;
	}

	//turn microstepping on
	changeMicrostep(16);
	motorReset();

	setupTCInterrupts();
	enableTCInterrupts();

}

Angle StepperCtrl::sampleAngle(void)
{
	uint16_t angle;

#ifdef NZS_AS5047_PIPELINE
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
	int i;
	int32_t mean=0;

	last=sampleAngle();
	mean=(int32_t)last;
	for (i=0; i<(numSamples-1); i++)
	{
		int32_t d;
		d=last-sampleAngle(); //take the difference which handles roll over
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

	updateStep(dir,steps);
	if (false == enableFeedback)
	{
		moveToAngle(getDesiredLocation(),NVM->SystemParams.currentMa);
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
		n=numMicroSteps;
		ret=((int64_t)numSteps * A4954_NUM_MICROSTEPS+(n/2))/n;
		n=A4954_NUM_MICROSTEPS*fullStepsPerRotation;
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
		stepperDriver.move(n,NVM->SystemParams.currentMa);
	}
	currentLimit=false;

}



int64_t StepperCtrl::getDesiredLocation(void)
{
	int64_t ret;
	int32_t n;
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	n=fullStepsPerRotation * numMicroSteps;
	ret=((int64_t)numSteps * (int64_t)ANGLE_STEPS+(n/2))/n;
	if (state) enableTCInterrupts();
	return ret;
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
			LCDShow("AS5047 Error");
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




void StepperCtrl::moveToAbsAngle(int32_t a)
{

	int64_t ret;
	int32_t n;

	n=fullStepsPerRotation * numMicroSteps;

	ret=((int64_t)a*n)/(int32_t)ANGLE_STEPS;
	numSteps=ret;
}

void StepperCtrl::moveToAngle(int32_t a, uint32_t ma)
{
	//we need to convert 'Angle' to A4954 steps
	a=a % ANGLE_STEPS;  //we only interested in the current angle


	a=DIVIDE_WITH_ROUND( (a*fullStepsPerRotation*A4954_NUM_MICROSTEPS), ANGLE_STEPS);

	//LOG("move %d %d",a,ma);
	stepperDriver.move(a,ma);
	currentLimit=false;
}


int32_t StepperCtrl::getCurrentLocation(void)
{
	Angle a;
	int32_t x;
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	a=calTable.fastReverseLookup(sampleAngle());
	x=(int32_t)a - (int32_t)(currentLocation & ANGLE_MAX);

	if (x>((int32_t)ANGLE_STEPS/2))
	{
		currentLocation -= ANGLE_STEPS;
	}
	if (x<-((int32_t)ANGLE_STEPS/2))
	{
		currentLocation += ANGLE_STEPS;
	}
	currentLocation=(currentLocation & 0xFFFF0000UL) | (uint16_t)a;
	if (state) enableTCInterrupts();
	return currentLocation;

}

float StepperCtrl::getCurrentAngle(void)
{
	int32_t x;
	float f;
	x=getCurrentLocation();
	f=(x*360.0)/(float)ANGLE_STEPS;
	return f;
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
	int32_t y;

	int32_t fullStep=ANGLE_STEPS/fullStepsPerRotation;

	y=getCurrentLocation();

	if (enableFeedback) //if ((micros()-lastCall)>(updateRate/10))
	{
		int32_t error,u;
		int32_t U,x;

		//error is in units of degrees when 360 degrees == 65536
		error=measureError(); //error is currentPos-desiredPos

		Iterm+=(pPID.Ki * error);

		x=Iterm;

		//Over the long term we do not want error
		// to be much more than our threshold
		if (x> (NVM->SystemParams.currentMa*CTRL_PID_SCALING) )
		{
			x=(NVM->SystemParams.currentMa*CTRL_PID_SCALING) ;
		}
		if (x<-(NVM->SystemParams.currentMa*CTRL_PID_SCALING)  )
		{
			x=-(NVM->SystemParams.currentMa*CTRL_PID_SCALING) ;
		}

		u=((pPID.Kp * error) + Iterm - (pPID.Kd *(error-lastError)));

		U=abs(u)/CTRL_PID_SCALING;
		if (U>NVM->SystemParams.currentMa)
		{
			U=NVM->SystemParams.currentMa;
		}

		//when error is positive we need to move reverse direction
		if (u>0)
		{
			y=y-fullStep;
		}else
		{
			y=y+fullStep;

		}

		moveToAngle(y,U);
		lastError=error;

	}else
	{  //feedback system is not enabled
		if (stepperDriver.microsSinceStep()>200)
		{
			if (false == currentLimit)
			{
				//stepperDriver.limitCurrent(99);
				currentLimit=true;
				LOG("current Limiting");
			}
		}
	}

	if (abs(lastError)>(NVM->SystemParams.errorLimit))
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

	int32_t fullStep=ANGLE_STEPS/fullStepsPerRotation;

	int32_t y;



	//estimate our current location based on the encoder
	y=getCurrentLocation();

	if (enableFeedback)
	{
		int32_t error,u;
		int32_t ma;

		int32_t x;

		//error is in units of degrees when 360 degrees == 65536
		error=-measureError(); //error is currentPos-desiredPos

		data[i]=(int16_t)error;
		i++;
		if (i>N_DATA)
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

		ma=(abs(u)*(NVM->SystemParams.currentMa-NVM->SystemParams.currentHoldMa))/fullStep + NVM->SystemParams.currentHoldMa;

		//ma=(abs(u)*(NVM->SystemParams.currentMa))/fullStep;

		if (ma>NVM->SystemParams.currentMa)
		{
			ma=NVM->SystemParams.currentMa;
		}

		//ma=NVM->SystemParams.currentMa;
		//		if (abs(u)<ANGLE_FROM_DEGREES(0.1))
		//		{
		//			u=u/2;
		//		}
		y=y+u;
		moveToAngle(y,ma); //35us

		lastError=error;
		//stepperDriver.limitCurrent(99);
	}

	if (abs(lastError)>(NVM->SystemParams.errorLimit))
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
	uint16_t microSteps=numMicroSteps;
	bool feedback=enableFeedback;

	enabled=enable;
	stepperDriver.enable(enabled); //enable or disable the stepper driver as needed

	if (enabled==false && enable==true) //if we are enabling previous disabled motor
	{
		enableFeedback=false;
		numMicroSteps=1;
		LOG("reset motor");
		motorReset();
	}

	numMicroSteps=microSteps;
	enableFeedback=feedback;
	if (state) enableTCInterrupts();
}

void StepperCtrl::testRinging(void)
{
	uint16_t c;
	int32_t steps;
	int32_t microSteps=numMicroSteps;
	bool feedback=enableFeedback;

	enableFeedback=false;
	numMicroSteps=1;
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
	numMicroSteps=microSteps;
	motorReset();
	enableFeedback=feedback;
}

bool StepperCtrl::processFeedback(void)
{
	bool ret;
	int32_t us;
	us=micros();
	switch (controlType)
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

	}
	loopTimeus=micros()-us;
	return ret;
}
void StepperCtrl::setControlMode(feedbackCtrl_t mode)
{
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	controlType=mode;
	LOG("mode set to %d",(int)mode);
	if (state) enableTCInterrupts();
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
	uint16_t microSteps=numMicroSteps;
	bool feedback=enableFeedback;
	feedbackCtrl_t prevCtrl=controlType;

	//disable interrupts and feedback controller
	bool state=TC5_ISR_Enabled;
	disableTCInterrupts();
	controlType=CTRL_POS_PID;
	enableFeedback=false;
	motorReset();
	//nvmWritePID(1,0,0,2);
	//set the number of microsteps to 1
	//numMicroSteps=1;
	for (i=0; i<10; i++)
	{
		angle=getCurrentLocation();
	}
	//	pKp=NVM->PIDparams.Kp;
	//	pKi=NVM->PIDparams.Ki;
	//	pKd=NVM->PIDparams.Kd;
	//	threshold=NVM->PIDparams.Threshold;

	//enableTCInterrupts();
	moveToAngle(angle,NVM->SystemParams.currentMa);


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



	stepperDriver.move(0,NVM->SystemParams.currentMa);
	delay(1000);

	//now we need to do the relay control
	for (i=0; i<10; i++)
	{
		startAngle=getCurrentLocation();
		LOG("Start %d", (int32_t)startAngle);
	}
	thres=startAngle + (int32_t)((ANGLE_STEPS/fullStepsPerRotation)*10/9);
	LOG("Thres %d, start %d",(int32_t)thres,(int32_t)startAngle);
	eMin=(int32_t)ANGLE_MAX;
	eMax=-(int32_t)ANGLE_MAX;
	int32_t reset=0;
	int32_t force=(NVM->SystemParams.currentMa);

	for (i=0; i<100; i++)
	{
		int32_t error;
		if (reset)
		{
			motorReset();
			stepperDriver.move(0,NVM->SystemParams.currentMa);
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
		//moveToAngle(startAngle+(ANGLE_STEPS/fullStepsPerRotation),force);
		//stepperDriver.move(A4954_NUM_MICROSTEPS,NVM->SystemParams.currentMa);
		t0=micros();

		error=0;
		while(error<=((ANGLE_STEPS/fullStepsPerRotation)/2+40))
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
			if (abs(error)>ANGLE_STEPS/fullStepsPerRotation*2)
			{
				LOG("large Error1 %d, %d, %d",error, y, startAngle);

				reset=1;
				break;
			}

		}

		stepperDriver.move(0,force);

		//stepperDriver.move(0,NVM->SystemParams.currentMa);
		t1=micros();

		error=(ANGLE_STEPS/fullStepsPerRotation);
		while(error>=((ANGLE_STEPS/fullStepsPerRotation)/2-40))
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
			if (abs(error)>ANGLE_STEPS/fullStepsPerRotation*2)
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
	controlType=prevCtrl;
	numMicroSteps=microSteps;
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
