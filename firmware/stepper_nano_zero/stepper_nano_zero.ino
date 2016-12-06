#include "syslog.h"
#include "board.h"
#include "as5047d.h"
#include "stepper_controller.h"
#include "commands.h"

StepperCtrl stepperCtrl;


//this function is called on the rising edge of a step from external device
static void stepInput(void)
{
  static int dir;
  //read our direction pin
  dir = digitalRead(PIN_DIR_INPUT);

  stepperCtrl.requestStep(dir,1);
}

#ifdef USE_ENABLE_PIN
//this function is called when error pin chagnes as enable signal
static void enableInput(void)
{
  static int enable;
  //read our enable pin 
  enable = digitalRead(PIN_ERROR);
  stepperCtrl.enable(enable);
}
#endif

  

void setup() {
  // put your setup code here, to run once:

  boardSetupPins();
  int to=20;
  SerialUSB.begin(460800);
  
  Serial5.begin(460800);
  SysLogInit(&Serial5,LOG_DEBUG); //use SWO for the sysloging
  LOG("Power up!");
  while (!SerialUSB) 
  {
    to--;
    if (to == 0)
    {
      break;
    }
    delay(500);
   };     //wait for serial

  //SysLogInit(&Serial5,LOG_DEBUG); //use SWO for the sysloging
  LOG("SErial up!");
  stepperCtrl.begin(); //start controller before accepting step inputs
  LOG("command init!");
  commandsInit(); //setup command handler system
  
  attachInterrupt(digitalPinToInterrupt(PIN_STEP_INPUT), stepInput, RISING);
  
  #ifdef USE_ENABLE_PIN //if we use enable pin setup interrupt to handle it
  attachInterrupt(digitalPinToInterrupt(PIN_ERROR), enableInput, CHANGE);
  #endif

  LOG("SETUP DONE!");
}


void TC5_Handler()
{
if (TC5->COUNT16.INTFLAG.bit.OVF == 1)
{
	int error;
	
	error=(stepperCtrl.processFeedback()); //handle the control loop
	YELLOW_LED(error);
#ifndef USE_ENABLE_PIN
	if (error)
	{	//assume high is inactive and low is active on error pin
		digitalWrite(PIN_ERROR,LOW);
	}else 
	{
		digitalWrite(PIN_ERROR,HIGH);
	}
#endif

	TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
}

}

void loop() {
	static uint32_t t0=0;

   commandsProcess(); //handle commands

   	if ((millis()-t0)>1000)
		{
			//LOG("Loc %d, %d %d",x,(uint32_t)currentLocation, (uint32_t)getDesiredLocation());
			//LOG("Error %d, Iterm %d, y %d, loc %d, calls %d, ma %d",error,Iterm,y,(int32_t)currentLocation,calls, ma);
			t0=millis();
			//stepperDriver.limitCurrent(99);
			stepperCtrl.UpdateLcd();
			LOG("Loop time %d", stepperCtrl.getLoopTime());
			//calls=0;
		}
   
   return;
}
