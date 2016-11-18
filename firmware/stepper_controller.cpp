#include <Adafruit_GFX.h>
#include <gfxfont.h>

#include "stepper_controller.h"
#include "nonvolatile.h" //for programmable parameters



void  StepperCtrl::motorReset(void)
{
	//when we reset the motor we want to also sync the motor
	// phase.  Therefore we move forward a few full steps then back
	// to sync motor phasing, leaving the motor at "phase 0"
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
	delay(100);

	setLocationFromEncoder(); //measure new starting point
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
		a=calTable.reverseLookup(x);

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
	encoder.diagnostics(ptrStr);
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

	return (int32_t)(currentLocation-getDesiredLocation());
}

bool StepperCtrl::changeMicrostep(uint16_t microSteps)
{
	numMicroSteps=microSteps;
	motorReset();
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

		mean=measureMeanEncoder();
		desiredAngle=(uint16_t)(getDesiredLocation() & 0x0FFFFLL);

		cal=calTable.getCal(desiredAngle);
		dist=Angle(mean)-cal;
		act=calTable.reverseLookup(cal);

		LOG("actual %d, cal %d",mean,(uint16_t)cal);
		LOG("desired %d",(uint16_t)desiredAngle);
		LOG("numSteps %d", numSteps);

		LOG("cal error for step %d is %d, %d",j,dist,act-desiredAngle);
		LOG("mean %d, cal %d",mean, (uint16_t)cal);

		requestStep(0,1);

		steps+=A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,NVM->SystemParams.currentMa);

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

	enableFeedback=false;
	numMicroSteps=1;
	motorReset();
	steps=0;

	j=0;
	while(!done)
	{
		Angle cal,desiredAngle;
		desiredAngle=(uint16_t)(getDesiredLocation() & 0x0FFFFLL);
		cal=calTable.getCal(desiredAngle);
		mean=measureMeanEncoder();

		LOG("Previous cal distance %d, %d, mean %d, cal %d",j, cal-Angle((uint16_t)mean), mean, (uint16_t)cal);

		calTable.updateTableValue(j,mean);
		requestStep(0,1);
		steps=steps+A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,NVM->SystemParams.currentMa);

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
	return done;
}

void StepperCtrl::UpdateLcd(void)
{
	char str[100];

	display.clearDisplay();
	sprintf(str,"uSteps %d", numMicroSteps);

	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.println(str);


	sprintf(str,"%01d.%02d Amps", NVM->SystemParams.currentMa/1000, NVM->SystemParams.currentMa%1000);
	display.setCursor(0,20);
	display.println(str);

	sprintf(str,"%01d.%02d deg", 0,0);
	display.setCursor(0,40);
	display.println(str);

	display.display();


}

int StepperCtrl::begin(void)
{
	int i;
	int32_t x;

	enableFeedback=false;
	forwardWiring=true; //assume we are forward wiring to start with

	degreesPerFullStep=0;
	currentLocation=0;
	numSteps=0;
	currentLimit=false;

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	stepperDriver.begin();
	fullStepsPerRotation=200;

	LOG("NVM calvalid %d", NVM->CalibrationTable.status);
	LOG("NVM cal[0] %d", NVM->CalibrationTable.table[0]);

	if (false == NVM->PIDparams.parametersVaild)
	{
		nvmWritePID(200,5,0,1000);
	}

	if (false == NVM->SystemParams.parametersVaild)
	{
		nvmWriteSystemParms(2000,500,150);
	}
	UpdateLcd();

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

	if (calTable.calValid() == false)
	{
		LOG("Requesting Calibration");
		calibrateEncoder();
	}

	//verify the calbiration is good which forces a flash update
	if (calTable.calValid() == false)
	{
		ERROR("Could not calibrate");
	}
	//turn microstepping on
	changeMicrostep(16);
	motorReset();

}

Angle StepperCtrl::sampleAngle(void)
{
	uint16_t angle;

	angle=((uint32_t)encoder.readEncoderAngle())<<2; //convert the 14 bit encoder value to a 16 bit number
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
	enableFeedback=enable;
	motorReset();
}

void StepperCtrl::requestStep(int dir, uint16_t steps)
{
	if (dir)
	{
		numSteps-=steps;
	}else
	{
		numSteps+=steps;
	}

	//	if (false == enableFeedback)
	//	{
	//		int32_t s;
	//		s=(steps*A4954_NUM_MICROSTEPS)/numMicroSteps;
	//		stepperDriver.move(s,NVM->SystemParams.currentMa);
	//	}

}


void StepperCtrl::move(int dir, uint16_t steps)
{
	int64_t ret;
	int32_t n;

	n=numMicroSteps;

	if (dir)
	{
		numSteps-=steps;

	}else
	{
		numSteps+=steps;
	}

	if (false == enableFeedback)
	{
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

	n=fullStepsPerRotation * numMicroSteps;
	ret=((int64_t)numSteps * ANGLE_STEPS+(n/2))/n;
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
	Serial.print(t1-t0);
	for (i=0; i<N_SAMPLES; i++)
	{
		Serial.print(',');
		Serial.print(angle[i]);
		//sprintf(str,"%s,%u",str,angle[i]);
	}
	Serial.print("\n\r");
	return 0;
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

	a=calTable.reverseLookup(sampleMeanEncoder(1));
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
	return currentLocation;

}
//Since we are doing fixed point math our
// threshold needs to be large.
// We need a large threshold when we have fast update
// rate as well. But for most part it is random
bool StepperCtrl::process(void)
{
	static int count=0;

	static uint32_t calls=0;
	static uint32_t t0=0;
	static uint32_t lastCall=0;
	static uint32_t lastForward=0;
	static uint32_t lastReverse=0;

	static int32_t maxError=0;
	static int32_t lastError=0;
	static int32_t Iterm=0;



	currentLocation=getCurrentLocation();

	if (enableFeedback) //if ((micros()-lastCall)>(updateRate/10))
	{
		int32_t error,u;
		int32_t t=micros();
		int32_t y,U;

		//		while ( (micros()-lastCall)<333)
		//		{
		//
		//		}

		lastCall=micros();
		//my goal was working and stable for the PID
		// parameters, may not be quickest.
		int32_t pKp=NVM->PIDparams.Kp;
		int32_t pKi=NVM->PIDparams.Ki;
		int32_t pKd=NVM->PIDparams.Kd;
		int32_t threshold=NVM->PIDparams.Threshold;

		//error is in units of degrees when 360 degrees == 65536
		error=measureError(); //error is currentPos-desiredPos

		Iterm+=(pKi * error);

		//Over the long term we do not want error
		// to be much more than our threshold
		if (Iterm> (threshold*10.5) )
		{
			Iterm=(threshold*10.5) ;
		}
		if (Iterm<-(threshold*10.5)  )
		{
			Iterm=-(threshold*10.5) ;
		}

		u=((pKp * error) + Iterm - (pKd *(error-lastError)));

		y=(currentLocation);  //our current encoder estimate of angle

		U=abs(u)/threshold;
		if (U>NVM->SystemParams.currentMa)
		{
			U=NVM->SystemParams.currentMa;
		}

		int x=327;

		//when error is positive we need to move reverse direction
		if (u>0)
		{
			y=y-x;//ANGLE_FROM_DEGREES(1.8);
		}else
		{
			y=y+x;//ANGLE_FROM_DEGREES(1.8);

		}

		moveToAngle(y,U);


		calls++;

		if ((millis()-t0)>1000)
		{
			//LOG("Loc %d, %d %d",x,(uint32_t)currentLocation, (uint32_t)getDesiredLocation());
			LOG("Error %d, u %d, y %d %d, calls %d, U %d",error,u,y,(int32_t)currentLocation,calls, U);
			t0=millis();
			calls=0;
		}

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
bool StepperCtrl::process2(void)
{
	static uint32_t t0=0;
	static uint32_t calls=0;

	static int32_t maxError=0;
	static int32_t lastError=0;
	static int32_t Iterm=0;
	int32_t y;

	//estimate our current location based on the encoder
	y=getCurrentLocation();

	if (enableFeedback)
	{
		int32_t error,u;
		int32_t ma;

		int32_t pKp=NVM->PIDparams.Kp;
		int32_t pKi=NVM->PIDparams.Ki;
		int32_t pKd=NVM->PIDparams.Kd;
		int32_t threshold=NVM->PIDparams.Threshold;

		//error is in units of degrees when 360 degrees == 65536
		error=-measureError(); //error is currentPos-desiredPos

		Iterm+=(pKi*error);
		Iterm=(127*Iterm)/128;

		//Over the long term we do not want error
		// to be much more than one full step
		if (Iterm> (threshold*327/2) )
		{
			Iterm=(threshold*327/2) ;
		}
		if (Iterm<-(threshold*327/2)  )
		{
			Iterm=-(threshold*327/2) ;
		}

		u=((pKp * error) + Iterm - (pKd *(error-lastError)));

		u=u/threshold;


		//limit error to full step
		if (u>327)
		{
			u=327;
		}
		if (u<-327)
		{
			u=-327;
		}

		ma=abs(u)/327*(NVM->SystemParams.currentMa-NVM->SystemParams.currentHoldMa) + NVM->SystemParams.currentHoldMa;

		if (ma>NVM->SystemParams.currentMa)
		{
			ma=NVM->SystemParams.currentMa;
		}


		y=y+u;
		moveToAngle(y,ma);
		calls++;
		lastError=error;

		if ((millis()-t0)>1000)
		{
			//LOG("Loc %d, %d %d",x,(uint32_t)currentLocation, (uint32_t)getDesiredLocation());
			LOG("Error %d, u %d, y %d, loc %d, calls %d, ma %d",error,u,y,(int32_t)currentLocation,calls, ma);
			t0=millis();
			stepperDriver.limitCurrent(99);
			calls=0;
		}
	}

	if (abs(lastError)>(NVM->SystemParams.errorLimit))
	{
		return 1;
	}
	return 0;
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
		Serial.print("Current ");
		Serial.println(c);
		steps+=A4954_NUM_MICROSTEPS;
		stepperDriver.move(steps,NVM->SystemParams.currentMa);
		currentLimit=false;
		measure();
	}
	numMicroSteps=microSteps;
	motorReset();
	enableFeedback=feedback;
}

