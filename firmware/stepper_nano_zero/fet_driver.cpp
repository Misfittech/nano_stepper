/*
 * fet_driver.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: tstern
 *
 * This file supports using discrete FETs as drivers for stepper motor
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/
#include "fet_driver.h"
#include "wiring_private.h"
#include "syslog.h"
#include "angle.h"
#include "Arduino.h"
#include "sine.h"


/*
 *  The discrete FETs on the NEMA 23 10A board are configured such that each H-bridge has:
 *    IN1 - Input 1
 *    IN2 - Input 2
 *    Enable - Enable driver
 *    Isense - current sense
 *
 *    The truth table for the H-Bridge is:
 *    Enable	IN1		IN2		Bridge State
 *    0			x		x		floating (FETs off)
 *    1			0		0		coil shorted to Gnd
 *    1			0		1		forward
 *    1			1		0		reverse
 *    1			1		1		coil shorted to VCC
 *
 *    For peak current control there is two state (fast decay, and slow decay)
 *
 *    Fast Decay
 *    When driving coil in forward direction and current peak is reached the fast decay turns
 *    The bridge in the reverse direction. This cause the reverse EMF from coil to charge
 *    capacitors back up and the current on the coil to drop very quickly
 *
 *    Slow Decay
 *    During this mode the current decay is slower by shorting the coil leads to ground.
 *    This in effect shorts the coil leads and reverse EMF is converted to heat.
 *
 *    In the Fast Decay mode we reverse the motor, this in effect is trying to drive coil
 *    current in the reverse direction. This in effect reduces current faster than just
 *    shorting the coil out.
 *
 *    see www.misfittech.net's blog for more information on this subject
 *
 */

/* driver code's logic
 *
 * 	This driver code needs not only to control the FETs but also handle the current limits.
 *
 * 	The way the code handles limiting current is by using two comparators internal to
 * 	the microprocessor.
 *
 * 	We first use two PWM signals to generate reference voltage for each comparator.
 * 	Then when the current sense voltage exceeds this reference voltage an interrupt is
 * 	generated. In the interrupt handler we will then set the decay mode as needed.
 *
 * 	It will have to be determined if we will use a fixed time decay mode like the A4954,
 * 	or use current as the threshold. There is a lot to do here to maintain quite operation,
 * 	that is we need this current control to be running at more than 20khz to be quite.
 *
 * 	Additionally we can use ADC on the current sense for detecting the flyback and
 * 	get some idea of the inductance. This can be used for stall dection as well as
 * 	auto tuning of some of the driver parameters.
 */



#pragma GCC push_options
#pragma GCC optimize ("-Ofast")


typedef enum {
	COIL_FORWARD =0,
	COIL_REVERSE =1,
	COIL_BRAKE =2
} CoilState_t;
static uint8_t pinState=0; //this variable keeps track of the pin states


#define DAC_MAX (0x01FFL)
// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
	//int32_t t0=1000;
	while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
	{
//		t0--;
//		if (t0==0)
//		{
//			break;
//		}

	}
}


static inline void coilA(CoilState_t state)
{

	switch(state){

		case COIL_FORWARD:
			GPIO_HIGH(PIN_FET_IN1);
			GPIO_LOW(PIN_FET_IN2);
			break;

		case COIL_REVERSE:
			GPIO_HIGH(PIN_FET_IN2);
			GPIO_LOW(PIN_FET_IN1);
			break;

		case COIL_BRAKE:
			GPIO_LOW(PIN_FET_IN2);
			GPIO_LOW(PIN_FET_IN1);
			break;

		default:
			ERROR("Not a known state");
			break;
	}
}

static inline void coilB(CoilState_t state)
{

	switch(state){
		case COIL_FORWARD:
			GPIO_HIGH(PIN_FET_IN3);
			GPIO_LOW(PIN_FET_IN4);
			break;

		case COIL_REVERSE:
			GPIO_HIGH(PIN_FET_IN4);
			GPIO_LOW(PIN_FET_IN3);
			break;

		case COIL_BRAKE:
			GPIO_LOW(PIN_FET_IN3);
			GPIO_LOW(PIN_FET_IN4);
			break;

		default:
			ERROR("Not a known state");
			break;
	}
}

/*
 * The SAMD21 has two analog comparators
 *  COMP_FET_A(A4/PA05) and COMP_FET_B(D9/PA07) are the reference voltages
 *
 *  ISENSE_FET_A(A3/PA04) and ISENSE_FET_B(D8/PA06) are the current sense
 *
 */
static void setupComparators(void)
{
	//setup the pins as analog inputs
	pinPeripheral(COMP_FET_A, PIO_ANALOG); //AIN[1]
	pinPeripheral(COMP_FET_B, PIO_ANALOG); 	//AIN[3]
	pinPeripheral(ISENSE_FET_A, PIO_ANALOG);  //AIN[0]
	pinPeripheral(ISENSE_FET_B, PIO_ANALOG);  //AIN[2]

	//enable the clock for the Analog comparator
	PM->APBCMASK.reg |= PM_APBCMASK_AC; //enable clock in the power manager

	//setup the GCLK for the analog and digital clock to the AC
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_AC_ANA )) ;
	 while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_AC_DIG )) ;
	 while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;


	//we will drive the CMP0 and CMP1 high when our current is exceeded.
	// To do this we will set ISense Pins as the non-inverting input
	AC->CTRLA=0x0; //disable AC_COMPCTRL_ENABLE
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->CTRLB=0x0; // set start bits low (will not be used)
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->COMPCTRL[0].reg = 	AC_COMPCTRL_FLEN_MAJ3_Val | //add a 3 bit majority digital filter
							AC_COMPCTRL_HYST | //enable hysterisis
							AC_COMPCTRL_MUXPOS_PIN0 | //non-inverting is AIN[0]
							AC_COMPCTRL_MUXNEG_PIN1 | //inverting pin is AIN[1]
							AC_COMPCTRL_INTSEL_RISING | //interrupt on the rising edge (TODO we might want on both edges)
							AC_COMPCTRL_SPEED_HIGH;  //set to high speed mode, we don't care about power consumption
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->COMPCTRL[1].reg = 	AC_COMPCTRL_FLEN_MAJ3_Val | //add a 3 bit majority digital filter
							AC_COMPCTRL_HYST | //enable hysterisis
							AC_COMPCTRL_MUXPOS_PIN2 | //non-inverting is AIN[2]
							AC_COMPCTRL_MUXNEG_PIN3 | //inverting pin is AIN[3]
							AC_COMPCTRL_INTSEL_RISING | //interrupt on the rising edge (TODO we might want on both edges)
							AC_COMPCTRL_SPEED_HIGH;  //set to high speed mode, we don't care about power consumption
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;


}

void A4954::begin()
{
	//Setup the FET inputs
	GPIO_OUTPUT(PIN_FET_IN1);
	GPIO_OUTPUT(PIN_FET_IN2);
	GPIO_OUTPUT(PIN_FET_IN3);
	GPIO_OUTPUT(PIN_FET_IN4);
	GPIO_OUTPUT(PIN_FET_ENABLE);
	GPIO_LOW(PIN_FET_ENABLE);
	coilA(COIL_BRAKE);
	coilB(COIL_BRAKE);

	//setup the comparator for current sense


	//setup the PWM for current on the A4954, set for low current
	digitalWrite(PIN_A4954_VREF12,LOW);
	digitalWrite(PIN_A4954_VREF34,LOW);
	pinMode(PIN_A4954_VREF34, OUTPUT);
	pinMode(PIN_A4954_VREF12, OUTPUT);

	enabled=true;
	lastStepMicros=0;
	forwardRotation=true;

	enableTCC0(90);
	setupDAC();
	//
	//	WARNING("Setting DAC for 500mA output");
	//	setDAC((int32_t)((int64_t)1000*(DAC_MAX))/3300,(int32_t)((int64_t)1000*(DAC_MAX))/3300);
	//	bridge1(0);
	//	bridge2(0);
	//	while(1)
	//	{
	//
	//	}
	return;
}



static inline void bridge2(CoilState_t state)
{
	if (state==0)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN3].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN3].ulPin].bit.PMUXEN = 0;
		GPIO_OUTPUT(PIN_A4954_IN3); //pinMode(PIN_A4954_IN3,OUTPUT);
		GPIO_OUTPUT(PIN_A4954_IN4);//pinMode(PIN_A4954_IN4,OUTPUT);
		GPIO_HIGH(PIN_A4954_IN3);//digitalWrite(PIN_A4954_IN3, HIGH);
		GPIO_LOW(PIN_A4954_IN4);//digitalWrite(PIN_A4954_IN4, LOW);
		//pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);
		pinState=(pinState & 0x03) | 0x4;
	}
	if (state==1)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN4].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN4].ulPin].bit.PMUXEN = 0;
		GPIO_OUTPUT(PIN_A4954_IN4);//pinMode(PIN_A4954_IN4,OUTPUT);
		GPIO_OUTPUT(PIN_A4954_IN3);//pinMode(PIN_A4954_IN3,OUTPUT);
		GPIO_LOW(PIN_A4954_IN3);//digitalWrite(PIN_A4954_IN3, LOW);
		GPIO_HIGH(PIN_A4954_IN4);//digitalWrite(PIN_A4954_IN4, HIGH);
		//pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);
		pinState=(pinState & 0x03) | 0x8;
	}
	if (state==3)
	{
		GPIO_LOW(PIN_A4954_IN3);
		GPIO_LOW(PIN_A4954_IN4);
		//digitalWrite(PIN_A4954_IN3, LOW);
		//digitalWrite(PIN_A4954_IN4, LOW);
	}
}

void enableTCC0(uint8_t percent)
{
#ifdef MECHADUINO_HARDWARE
	return;
#else
	Tcc* TCCx = TCC0 ;


	uint32_t ulValue=((uint32_t)(100-percent)*480)/100;
	//ERROR("Enable TCC0");

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;

	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

	//ERROR("Setting TCC %d %d",ulValue,ulPin);
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC(TCCx);

	// Set TCx as normal PWM
	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	syncTCC(TCCx);

	// Set TCx in waveform mode Normal PWM
	TCCx->CC[1].reg = (uint32_t)ulValue; //ch5 //IN3
	syncTCC(TCCx);

	TCCx->CC[2].reg = (uint32_t)ulValue; //ch6 //IN4
	syncTCC(TCCx);

	TCCx->CC[3].reg = (uint32_t)ulValue; //ch7  //IN2
	syncTCC(TCCx);

	TCCx->CC[1].reg = (uint32_t)ulValue; //ch1 == ch5 //IN1

	syncTCC(TCCx);

	// Set PER to maximum counter value (resolution : 0xFF)
	TCCx->PER.reg = 480;
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC(TCCx);
	//ERROR("Enable TCC0 DONE");
#endif
}

void setDAC(uint32_t DAC1, uint32_t DAC2)
{
	TCC1->CC[1].reg = (uint32_t)DAC1; //D9 PA07 - VREF12
	syncTCC(TCC1);
	TCC1->CC[0].reg = (uint32_t)DAC2; //D4 - VREF34
	syncTCC(TCC1);



}

void setupDAC(void)
{
	Tcc* TCCx = TCC1 ;


	pinPeripheral(PIN_A4954_VREF34, PIO_TIMER_ALT);
	pinPeripheral(PIN_A4954_VREF12, PIO_TIMER);

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;

	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

	//ERROR("Setting TCC %d %d",ulValue,ulPin);
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC(TCCx);

	// Set TCx as normal PWM
	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	syncTCC(TCCx);

	// Set TCx in waveform mode Normal PWM
	TCCx->CC[1].reg = (uint32_t)0;
	syncTCC(TCCx);

	TCCx->CC[0].reg = (uint32_t)0;
	syncTCC(TCCx);

	// Set PER to maximum counter value (resolution : 0xFFF = 12 bits)
	// =48e6/2^12=11kHz frequency
	TCCx->PER.reg = DAC_MAX;
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC(TCCx);

}




void A4954::limitCurrent(uint8_t percent)
{
#ifdef MECHADUINO_HARDWARE
	return;
#else
	//WARNING("current limit %d",percent);
	enableTCC0(percent);
	if (pinState & 0x01)
	{
		pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT); //TCC0 WO[7]
	}
	if (pinState & 0x02)
	{
		pinPeripheral(PIN_A4954_IN1, PIO_TIMER); //TCC0 WO[1]
	}
	if (pinState & 0x04)
	{
		pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);
	}
	if (pinState & 0x08)
	{
		pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);
	}
#endif
}






//this is precise move and modulo of A4954_NUM_MICROSTEPS is a full step.
// stepAngle is in A4954_NUM_MICROSTEPS units..
// The A4954 has no idea where the motor is, so the calling function has to
// to tell the A4954 what phase to drive motor coils.
// A4954_NUM_MICROSTEPS is 256 by default so stepAngle of 1024 is 360 degrees
// Note you can only move up to +/-A4954_NUM_MICROSTEPS from where you
// currently are.
int32_t A4954::move(int32_t stepAngle, uint32_t mA)
{
	uint16_t angle;
	int32_t cos,sin;
	int32_t dacSin,dacCos;


	if (enabled == false)
	{
		WARNING("A4954 disabled");
		setDAC(0,0); //turn current off
		bridge1(3); //tri state bridge outputs
		bridge2(3); //tri state bridge outputs
	}

	//WARNING("move %d %d",stepAngle,mA);
	//handle roll overs, could do with modulo operator
	stepAngle=stepAngle%SINE_STEPS;
	//	while (stepAngle<0)
	//	{
	//		stepAngle=stepAngle+SINE_STEPS;
	//	}
	//	while (stepAngle>=SINE_STEPS)
	//	{
	//		stepAngle=stepAngle-SINE_STEPS;
	//	}

	//figure out our sine Angle
	// note our SINE_STEPS is 4x of microsteps for a reason
	angle=(stepAngle+(SINE_STEPS/8)) % SINE_STEPS;

	//calculate the sine and cosine of our angle
	sin=sine(angle);
	cos=cosine(angle);

	//if we are reverse swap the sign of one of the angels
	if (false == forwardRotation)
	{
		cos=-cos;
	}

	//scale sine result by current(mA)
	dacSin=((int32_t)mA*(int32_t)abs(sin))/SINE_MAX;

	//convert value into 12bit DAC scaled to 3300mA max
	dacSin=(int32_t)((int64_t)dacSin*(DAC_MAX))/3300;

	//scale cosine result by current(mA)
	dacCos=((int32_t)mA*(int32_t)abs(cos))/SINE_MAX;

	//convert value into 12bit DAC scaled to 3300mA max
	dacCos=(int32_t)((int64_t)dacCos*(DAC_MAX))/3300;

	//WARNING("dacs are %d %d %d",dacSin,dacCos,stepLoc);

	setDAC(dacSin,dacCos);

	if (sin>0)
	{
		bridge1(1);
	}else
	{
		bridge1(0);
	}
	if (cos>0)
	{
		bridge2(1);
	}else
	{
		bridge2(0);
	}
	//	YELLOW_LED(led);
	//	led=(led+1) & 0x01;
	lastStepMicros=micros();
	return stepAngle;
}
#pragma GCC pop_options //fast optimization



