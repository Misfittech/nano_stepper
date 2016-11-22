#ifndef __BOARD_H__
#define __BOARD_H__

#include <Arduino.h>

#define NZS_FAST_CAL // define this to use 32k of flash for fast calibration table
#define NZS_FAST_SINE //uses 2048 extra bytes to implement faster sine tables

/* does not work at moment
//#define NZS_AS5047_PIPELINE //does a pipeline read of encoder, which is slightly faster
 * DOES NOT WORK
 */

#define NZS_CONTROL_LOOP_HZ (4000) //update rate of control loop, this should be limited to less than 5k
//TCC0 is used for DAC PWM to the A4954
//TCC1 can be used as PWM for the input pins on the A4954

#define VERSION "FW: 0.1"


#define PIN_STEP_INPUT  (0)
#define PIN_DIR_INPUT   (1)
#define PIN_ERROR_INPUT (10)
#define PIN_ERROR_OUTPUT   (12)

#define PIN_AS5047D_CS  (16)//analogInputToDigitalPin(PIN_A2))
#define PIN_AS5047D_PWR	(11) //pull low to power on AS5047D

#define PIN_A4954_IN3		(5)
#define PIN_A4954_IN4		(6)
#define PIN_A4954_IN2		(7)
#define PIN_A4954_IN1		(18) //analogInputToDigitalPin(PIN_A4))
#define PIN_A4954_VREF34	(4)
#define PIN_A4954_VREF12	(9)

#define PIN_SW1		(19)//analogInputToDigitalPin(PIN_A5))
#define PIN_SW3		(14)//analogInputToDigitalPin(PIN_A0))
#define PIN_SW4		(15)//analogInputToDigitalPin(PIN_A1))

#define PIN_MOSI        (23)
#define PIN_SCK         (24)
#define PIN_MISO        (22)

#define PIN_RED_LED     (13)
#define PIN_YELLOW_LED  (8)

//Here are some useful macros
#define DIVIDE_WITH_ROUND(x,y)  ((x+y/2)/y)


//sets up the pins for the board
static void boardSetupPins(void)
{
	//setup switch pins
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_SW4, INPUT_PULLUP);

	pinMode(PIN_STEP_INPUT, INPUT_PULLDOWN);
	pinMode(PIN_DIR_INPUT, INPUT_PULLDOWN);
	pinMode(PIN_ERROR_INPUT, INPUT_PULLDOWN);
	pinMode(PIN_ERROR_OUTPUT, OUTPUT);

	pinMode(PIN_AS5047D_CS,OUTPUT);
	digitalWrite(PIN_AS5047D_CS,LOW); //pull CS LOW by default (chip powered off)

	//turn the AS5047D off by default
	pinMode(PIN_AS5047D_PWR,OUTPUT);
	digitalWrite(PIN_AS5047D_PWR,HIGH);


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
	pinMode(PIN_YELLOW_LED,OUTPUT);


	digitalWrite(PIN_YELLOW_LED,HIGH);


}

static void inline YELLOW_LED(bool state)
{
	digitalWrite(PIN_YELLOW_LED,!state);
}

static void inline RED_LED(bool state)
{
	digitalWrite(PIN_RED_LED,state);
}


#endif//__BOARD_H__

