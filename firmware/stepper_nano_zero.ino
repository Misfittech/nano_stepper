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
/*
  if (dir==0)
  {
  stepperCtrl.step(STEPPER_FORWARD);
  }else
  {
  stepperCtrl.step(STEPPER_REVERSE);
  }
  */
}



  

void setup() {
  // put your setup code here, to run once:

  boardSetupPins();
  int to=20;
  Serial.begin(460800);
  Serial5.begin(460800);
  SysLogInit(&Serial5,LOG_DEBUG); //use SWO for the sysloging
  LOG("Power up!");
  while (!Serial) 
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

  LOG("SETUP DONE!");
}


void TC5_Handler()
{
if (TC5->COUNT16.INTFLAG.bit.OVF == 1)
{
//	static int i=0;
//	YELLOW_LED(i);
//	i=(i+1) & 0x01;
	
	YELLOW_LED(stepperCtrl.process2()); //handle the control loop
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
			//calls=0;
		}
   
   return;
}
