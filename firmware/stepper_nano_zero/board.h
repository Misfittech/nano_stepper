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
#ifndef __BOARD_H__
#define __BOARD_H__

#include <Arduino.h>

//define this if you are using the Mechaduino
//#define MECHADUINO_HARDWARE

//define this if using the NEMA 23 10A hardware
//#define NEMA_23_10A_HW


#define NZS_FAST_CAL // define this to use 32k of flash for fast calibration table
#define NZS_FAST_SINE //uses 2048 extra bytes to implement faster sine tables


#define NZS_AS5047_PIPELINE //does a pipeline read of encoder, which is slightly faster

#define NZS_CONTROL_LOOP_HZ (6000) //update rate of control loop


#define NZS_LCD_ABSOULTE_ANGLE  //define this to show angle from zero in positive and negative direction
								// for example 2 rotations from start will be angle of 720 degrees

#define VERSION "FW: 0.10" //this is what prints on LCD during splash screen

#define SERIAL_BAUD (115200) //baud rate for the serial ports


#define F_CPU (48000000UL)


/* TODO are flaged with TODO
 *   TODO - add detection of magnet to make sure PCB is on motor
 */

/* change log
 *   0.02 added fixes for 0.9 degree motor
 *   0.03 added code for using error pin as an enable pin, enable feedback by default
 *   0.04
 *   0.05 added different modes added support for mechaduino
 *   0.06 added time out pipeline read, add some error logging on encoder failure for mechaduino
 *   0.07 many cahnges including
 *   	- fixed error on display when doing a move 99999
 *   	- added velocity and position PID modes
 *   	- fixed LCD menu and put LCD code in own file
 *   	- include LCD source files from adafruit as that ssd1306 need lcd resoultion fix
 *   	- added motor parameters to NVM such step size and rotation are only check on first boot
 *   	- added test on power up to see if motor power is applied.
 *   	- added factory reset command
 *   	- pPID is not stable in my testing.
 *   0.08
 *   	- moved enable pin processing out of interrupt context
 *   	- added mode for inverted logic on the enable pin
 *   	- added pin definitions for NEMA23 10A hardware
 *   	- Changed enable such that it does not move motor but just sets current posistion
 *	 0.09
 *	 	- enabled auto detection of LCD
 *	 	- cleaned up the commands, made motorparams and systemparams individual commands
 *	 	- added the option to the move command to move at a constant RPM
 *	 	- Added the setzero command to zero the relative location of motor
 *	 	- Added the stop command to stop the planner based moves.
 *	 0.10
 *	 	-Fixed bug in switching control mode to 3
 *
 */


/*
 *  Typedefs that are used across multiple files/modules
 */
typedef enum {
	CW_ROTATION=0,
	CCW_ROTATION=1,
} RotationDir_t;

typedef enum {
	ERROR_PIN_MODE_ENABLE=0, //error pin works like enable on step sticks
	ERROR_PIN_MODE_ACTIVE_LOW_ENABLE=1, //error pin works like enable on step sticks
	ERROR_PIN_MODE_ERROR=2,  //error pin is low when there is angle error
	ERROR_PIN_MODE_BIDIR=3,   //error pin is bidirection open collector

} ErrorPinMode_t;

typedef enum {
	CTRL_OFF =0, 	 //controller is disabled
	CTRL_OPEN=1, 	 //controller is in open loop mode
	CTRL_SIMPLE = 2, //simple error controller
	CTRL_POS_PID =3, //PID  Position controller
	CTRL_POS_VELOCITY_PID =4, //PID  Velocity controller
} feedbackCtrl_t;


// ******** TIMER USAGE A4954 versions ************
//TCC1 is used for DAC PWM to the A4954
//TCC0 can be used as PWM for the input pins on the A4954
//D0 step input could use TCC1 or TCC0 if not used
//TC5 is use for timing the control loop

// ******** TIMER USAGE NEMA23 10A versions ************
//TCC1 is used for DAC PWM to the comparators
//TCC0 PWM for the FET IN pins
//D10 step input could use TC3 or TCC0 if not used
//TC5 is use for timing the control loop



//mechaduio and Arduino Zero has defined serial ports differently than NZS
#ifdef MECHADUINO_HARDWARE
#warning "Compiling source for Mechaduino NOT NZS"
#define Serial5 Serial 
#else
#define SerialUSB Serial
#endif 

#define PIN_STEP_INPUT  (0)
#define PIN_DIR_INPUT   (1)

#define PIN_MOSI        (23)
#define PIN_SCK         (24)
#define PIN_MISO        (22)

#ifdef MECHADUINO_HARDWARE
#define PIN_ERROR 		(19)  //analogInputToDigitalPin(PIN_A5))
#else
#define PIN_ERROR		(10)
#endif

#define PIN_AS5047D_CS  (16)//analogInputToDigitalPin(PIN_A2))
#ifndef MECHADUINO_HARDWARE
#define PIN_AS5047D_PWR	(11) //pull low to power on AS5047D
#endif

//these pins use the TIMER in the A4954 driver
// changing the pin definitions here may require changes in the A4954.cpp file

#define PIN_FET_IN1		(5)
#define PIN_FET_IN2		(6)
#define PIN_FET_IN3		(7)
#define PIN_FET_IN4		(2)
#define PIN_FET_VREF1	(4)
#define PIN_FET_VREF2	(3)
#define PIN_FET_ENABLE		(12)
//current sense pin from each H-bridge
#define ISENSE_FET_A	 (17) //analogInputToDigitalPin(PIN_A3)
#define ISENSE_FET_B	 (8)
//Comparators analog inputs
#define COMP_FET_A		 (18)//analogInputToDigitalPin(PIN_A4))
#define COMP_FET_B		 (9)




#ifndef MECHADUINO_HARDWARE
#define PIN_YELLOW_LED  (8)
#endif


#define PIN_SW1		(19)//analogInputToDigitalPin(PIN_A1))
#define PIN_SW3		(14)//analogInputToDigitalPin(PIN_A0))
#define PIN_SW4		(15)//analogInputToDigitalPin(PIN_A5))

#ifdef NEMA_23_10A_HW
#undef PIN_YELLOW_LED
#define PIN_YELLOW_LED  	(26) //TXLED (PA27)
#endif //NEMA_23_10A_HW


#define PIN_RED_LED     (13)
#define PIN_A4954_IN3		(5)
#define PIN_A4954_IN4		(6)
#define PIN_A4954_IN2		(7)
#ifdef MECHADUINO_HARDWARE
#define PIN_A4954_IN1		(8)
#else
#define PIN_A4954_IN1		(18) //analogInputToDigitalPin(PIN_A4))
#endif
#define PIN_A4954_VREF34	(4)
#define PIN_A4954_VREF12	(9)



//Here are some useful macros
#define DIVIDE_WITH_ROUND(x,y)  ((x+y/2)/y)


#define GPIO_LOW(pin) {PORT->Group[g_APinDescription[(pin)].ulPort].OUTCLR.reg = (1ul << g_APinDescription[(pin)].ulPin);}
#define GPIO_HIGH(pin) {PORT->Group[g_APinDescription[(pin)].ulPort].OUTSET.reg = (1ul << g_APinDescription[(pin)].ulPin);}

#define GPIO_OUTPUT(pin) {PORT->Group[g_APinDescription[(pin)].ulPort].PINCFG[g_APinDescription[(pin)].ulPin].reg&=~(uint8_t)(PORT_PINCFG_INEN) ;  PORT->Group[g_APinDescription[(pin)].ulPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[(pin)].ulPin) ;}


//sets up the pins for the board
static void boardSetupPins(void)
{
	//setup switch pins
#ifdef PIN_SW1
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_SW4, INPUT_PULLUP);
#endif

	pinMode(PIN_STEP_INPUT, INPUT_PULLDOWN);
	pinMode(PIN_DIR_INPUT, INPUT_PULLDOWN);

	pinMode(PIN_ERROR, INPUT_PULLUP); //default error pin as enable pin with pull up

	pinMode(PIN_AS5047D_CS,OUTPUT);
	digitalWrite(PIN_AS5047D_CS,LOW); //pull CS LOW by default (chip powered off)

	//turn the AS5047D off by default
#ifdef PIN_AS5047D_PWR
	pinMode(PIN_AS5047D_PWR,OUTPUT);
	digitalWrite(PIN_AS5047D_PWR,HIGH);
#endif

	pinMode(PIN_MOSI,OUTPUT);
	digitalWrite(PIN_MOSI,LOW);
	pinMode(PIN_SCK,OUTPUT);
	digitalWrite(PIN_SCK,LOW);
	pinMode(PIN_MISO,INPUT);

	//setup the A4954 pins
	digitalWrite(PIN_A4954_IN3,LOW);
	pinMode(PIN_A4954_IN3,OUTPUT);
	digitalWrite(PIN_A4954_IN4,LOW);
	pinMode(PIN_A4954_IN4,OUTPUT);
	digitalWrite(PIN_A4954_IN2,LOW);
	pinMode(PIN_A4954_IN2,OUTPUT);
	digitalWrite(PIN_A4954_IN1,LOW);
	pinMode(PIN_A4954_IN1,OUTPUT);

	//setup the PWM for current on the A4954, set for low current
	digitalWrite(PIN_A4954_VREF12,LOW);
	digitalWrite(PIN_A4954_VREF34,LOW);
	pinMode(PIN_A4954_VREF34, OUTPUT);
	pinMode(PIN_A4954_VREF12, OUTPUT);



	pinMode(PIN_RED_LED,OUTPUT);
#ifdef PIN_YELLOW_LED
	pinMode(PIN_YELLOW_LED,OUTPUT);
	digitalWrite(PIN_YELLOW_LED,HIGH);
#endif


}

static void inline YELLOW_LED(bool state)
{
#ifdef PIN_YELLOW_LED
	digitalWrite(PIN_YELLOW_LED,!state);
#endif
}

static void inline RED_LED(bool state)
{
	digitalWrite(PIN_RED_LED,state);
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ABS(a) (((a)>(0))?(a):(-(a)))
#define DIV(x,y) (((y)>(0))?((x)/(y)):(4294967295))

#define NVIC_IS_IRQ_ENABLED(x) (NVIC->ISER[0] & (1 << ((uint32_t)(x) & 0x1F)))!=0


#endif//__BOARD_H__

