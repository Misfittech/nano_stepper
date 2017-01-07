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

#ifdef NEMA_23_10A_HW

#define FET_DRIVER_FREQ  (100000UL) //FET PWM pin driver frequency


volatile uint32_t coilA_Value=0;
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

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

typedef enum {
	CURRENT_ON = 0,
	CURRENT_FAST_DECAY = 1,
	CURRENT_SLOW_DECAY = 2,
} CurrentMode_t;

typedef enum {
	COIL_FORWARD =0,
	COIL_REVERSE =1,
	COIL_BRAKE =2
} CoilState_t;

typedef struct {
	bool currentIncreasing; //true when we are increasing current
	CurrentMode_t currentState; //how is bridge driven
} BridgeState_t;

volatile BridgeState_t BridgeA, BridgeB;


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


static void coilA_PWM(uint32_t ulValue)
{
		// PIN_FET_IN1	 (PA15)		(5)  (TCC0 WO[5], aka ch1)
	//PIN_FET_IN2    (PA20)		(6)  (TCC0 WO[6], aka ch2)
	Tcc* TCCx = TCC0 ;

	if (ulValue>(F_CPU/FET_DRIVER_FREQ))
	{
		ulValue=F_CPU/FET_DRIVER_FREQ;
	}

	TCCx->CC[1].reg = (uint32_t)ulValue; //ch1 == ch5 //IN3
	syncTCC(TCCx);

	TCCx->CC[2].reg = (uint32_t)ulValue; //ch2 == ch6 //IN4
	syncTCC(TCCx);

}

static void coilB_PWM(uint32_t ulValue)
{
	//PIN_FET_IN3	 (PA21)		(7)	 (TCC0 WO[7], aka ch3)
	//PIN_FET_IN4    (PA14)		(2)  (TCC0 WO[4], aka ch0)
	Tcc* TCCx = TCC0 ;

	if (ulValue>(F_CPU/FET_DRIVER_FREQ))
	{
		ulValue=F_CPU/FET_DRIVER_FREQ;
	}

	TCCx->CC[0].reg = (uint32_t)ulValue; //ch0 == ch4 //IN1
	syncTCC(TCCx);

	TCCx->CC[3].reg = (uint32_t)ulValue; //ch3 == ch7  //IN2
	syncTCC(TCCx);

}

static void enableTCC0(void)
{
	Tcc* TCCx = TCC0 ;

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;

	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

	//ERROR("Setting TCC %d %d",ulValue,ulPin);
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC(TCCx);

	// Set TCx as normal PWM
	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	syncTCC(TCCx);

	// Set PER to maximum counter value (resolution : 0xFF)
	TCCx->PER.reg = F_CPU/FET_DRIVER_FREQ; //set frequency to 100Khz
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC(TCCx);
	//ERROR("Enable TCC0 DONE");

}

static void setDAC(uint32_t DAC1, uint32_t DAC2)
{
	TCC1->CC[1].reg = (uint32_t)DAC1; //D9 PA07 - VREF12
	syncTCC(TCC1);
	TCC1->CC[0].reg = (uint32_t)DAC2; //D4 - VREF34
	syncTCC(TCC1);
}



static void setupDAC(void)
{
	Tcc* TCCx = TCC1 ;


	pinPeripheral(PIN_FET_VREF1, PIO_TIMER_ALT);
	pinPeripheral(PIN_FET_VREF2, PIO_TIMER_ALT);

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
	AC->CTRLA.reg=0x01; //disable AC_COMPCTRL_ENABLE and reset
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->CTRLB.reg=0x0; // set start bits low (will not be used)
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->COMPCTRL[0].reg = 	AC_COMPCTRL_FLEN_MAJ3_Val | //add a 3 bit majority digital filter
							AC_COMPCTRL_HYST | //enable hysterisis
							AC_COMPCTRL_MUXPOS_PIN0 | //non-inverting is AIN[0]
							AC_COMPCTRL_MUXNEG_PIN1 | //inverting pin is AIN[1]
							AC_COMPCTRL_INTSEL_RISING | //interrupt on the rising edge (TODO we might want on both edges)
							AC_COMPCTRL_SPEED_HIGH |
							AC_COMPCTRL_ENABLE;  //set to high speed mode, we don't care about power consumption
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;
	AC->COMPCTRL[1].reg = 	//AC_COMPCTRL_FLEN_MAJ3_Val | //add a 3 bit majority digital filter
							//AC_COMPCTRL_HYST | //enable hysterisis
							AC_COMPCTRL_MUXPOS_PIN2 | //non-inverting is AIN[2]
							AC_COMPCTRL_MUXNEG_PIN3 | //inverting pin is AIN[3]
							AC_COMPCTRL_INTSEL_RISING | //interrupt on the rising edge (TODO we might want on both edges)
							AC_COMPCTRL_SPEED_HIGH |
							//AC_COMPCTRL_SWAP |
							AC_COMPCTRL_ENABLE;  //set to high speed mode, we don't care about power consumption
	while ( AC->STATUSB.bit.SYNCBUSY == 1 ) ;

	//enable the comparator
	AC->CTRLA.reg=AC_CTRLA_ENABLE;
	while ( AC->STATUSB.bit.SYNCBUSY == 1 );



	AC->INTENSET.bit.COMP0=1;
	AC->INTENSET.bit.COMP1=1;
	NVIC_EnableIRQ(AC_IRQn); //enable the comparator interrupt
}


static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  volatile int32_t t0=1000;
  while ((ADC->STATUS.bit.SYNCBUSY == 1) && t0>0)
  {
	  t0--;
	  if (t0<=0)
	  {
		  break;
	  }
  }
}

static uint32_t ADCRead(uint32_t ulPin, uint32_t gain)
{
  uint32_t valueRead = 0;
  uint32_t gainValue=0;

  if ( ulPin <= 5 ) // turn '0' -> 'A0'
  {
    ulPin += A0 ;
  }
  if (ulPin == 6) ulPin = PIN_A6;
  if (ulPin == 7) ulPin = PIN_A7;

  pinPeripheral(PIN_A4, PIO_ANALOG);

   pinPeripheral(ulPin, PIO_ANALOG);

   syncADC();
     ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
  syncADC();
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_IOGND; //ADC_INPUTCTRL_MUXNEG_PIN5;   // No Negative input (Internal Ground)

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input

  switch (gain)
  {
	  case 1:
		  gainValue=ADC_INPUTCTRL_GAIN_1X_Val;
		  break;
	  case 2:
		  gainValue=ADC_INPUTCTRL_GAIN_2X_Val;
		  break;
	  case 4:
		  gainValue=ADC_INPUTCTRL_GAIN_4X_Val;
		  break;
	  case 8:
		  gainValue=ADC_INPUTCTRL_GAIN_8X_Val;
		  break;
	  case 16:
		  gainValue=ADC_INPUTCTRL_GAIN_16X_Val;
		  break;
	  default:
		  gainValue=ADC_INPUTCTRL_GAIN_1X_Val;
		  break;
  }

   syncADC();
  ADC->CTRLB.bit.DIFFMODE = 1;

  syncADC();
  ADC->INPUTCTRL.bit.GAIN = gainValue;

  // syncADC();
 // ADC->AVGCTRL.reg=5;

  // syncADC();
  //ADC->REFCTRL.reg=ADC_REFCTRL_REFSEL_INT1V;

  syncADC();
  ADC->SAMPCTRL.reg=0x01;
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC


  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.bit.RESRDY = 1;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();

  return valueRead; //mapResolution(valueRead, _ADCResolution, _readResolution);

}


//this is called when the comparator indicates the current is too high
void AC_Handler( void )
{
	if (AC->INTFLAG.bit.COMP0)
	{
		//YELLOW_LED(1);
		AC->INTFLAG.bit.COMP0=1;
	}
	if (AC->INTFLAG.bit.COMP1)
	{
		YELLOW_LED(1);
		coilA_Value++;
//		if (coilA_Value>1)
//		{
//			coilA_Value--;
//		}
		AC->INTFLAG.bit.COMP1=1;
	}
}

void FetDriver::begin()
{
	int16_t i;
	uint32_t t0;
	int32_t i0=0;;

	//enable 1V reference
	SYSCTRL->VREF.reg |= SYSCTRL_VREF_BGOUTEN;

	int32_t min,max,avg;
	//Setup the FET inputs
	GPIO_OUTPUT(PIN_FET_IN1);
	GPIO_OUTPUT(PIN_FET_IN2);
	GPIO_OUTPUT(PIN_FET_IN3);
	GPIO_OUTPUT(PIN_FET_IN4);
	GPIO_OUTPUT(PIN_FET_ENABLE);
	GPIO_HIGH(PIN_FET_ENABLE);
	coilA(COIL_BRAKE);
	coilB(COIL_BRAKE);

	//setup the comparator for current sense

	setupDAC();
	setDAC(5,5);
	enableTCC0();
	setupComparators();


	ERROR("Enable PWM");
	pinPeripheral(PIN_FET_IN3, PIO_TIMER_ALT); //TCC0 WO[7]

//
//	for (i=40; i<55; i++)
//	{
//		coilB_PWM(i);
//		delay(200);
//		ADCRead(8,16);
//		LOG("COMP is 0x%04X ", AC->STATUSA.reg);
//		LOG("%d ADC is %d ",i, ADCRead(8,16));
//		YELLOW_LED(0);
//	}

	//ADCRead(8,16);
	AC->INTENCLR.bit.COMP1=1;
	coilA_Value=0;
	i=45;
	coilB_PWM(i);
	AC->INTENSET.bit.COMP1=1;
	while(1)
	{
		AC->INTENCLR.bit.COMP1=1;
		YELLOW_LED(0);
		AC->INTENSET.bit.COMP1=1;
		if ((millis()-t0)>10000)
		{
			int j;
			min=0xFFFFFF;
			max=(int16_t)ADCRead(8,16);
			avg=0;
			j=0;
			t0=micros();
			while( (micros()-t0)<1000)
			{
				 int16_t valueRead;

				  valueRead = ADCRead(8,16);

				  if (valueRead<min)
				  {
					  min=valueRead;
				  }
				  if (valueRead>max)
				  {
					  max=valueRead;
				  }
				  avg+=valueRead;
				  j++;
			}


			int32_t ma,x,duty;
			duty=i-45;
			duty=(1000*duty)/(F_CPU/FET_DRIVER_FREQ);

			LOG("min %d max %d, avg %d j %d, %d", min, max, (avg*10)/j, j,(avg*10)/j*(1000-duty)/1000);

			x=(avg*10)/j*(1000-duty)/1000;
			x=(x*600)/1000+200;

			LOG("mA %d\n\r",x);

			if (i<150)
			{
				i=100;
			}else
			{
				i=45;
			}
			LOG("COMP is 0x%04X ", AC->STATUSA.reg);
			LOG("%d ADC is %d %d",i, ADCRead(8,16),coilA_Value);
			t0=millis();
			AC->INTENCLR.bit.COMP1=1;
			coilA_Value=0;
			coilB_PWM(i);
			AC->INTENSET.bit.COMP1=1;
		}
	}
	return;

	//setup the PWM for current on the A4954, set for low current
	digitalWrite(PIN_A4954_VREF12,LOW);
	digitalWrite(PIN_A4954_VREF34,LOW);
	pinMode(PIN_A4954_VREF34, OUTPUT);
	pinMode(PIN_A4954_VREF12, OUTPUT);

	enabled=true;
	lastStepMicros=0;
	forwardRotation=true;

	enableTCC0();
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











//this is precise move and modulo of A4954_NUM_MICROSTEPS is a full step.
// stepAngle is in A4954_NUM_MICROSTEPS units..
// The A4954 has no idea where the motor is, so the calling function has to
// to tell the A4954 what phase to drive motor coils.
// A4954_NUM_MICROSTEPS is 256 by default so stepAngle of 1024 is 360 degrees
// Note you can only move up to +/-A4954_NUM_MICROSTEPS from where you
// currently are.
int32_t FetDriver::move(int32_t stepAngle, uint32_t mA)
{
	uint16_t angle;
	int32_t cos,sin;
	int32_t dacSin,dacCos;

	return 0;

//	if (enabled == false)
//	{
//		WARNING("A4954 disabled");
//		setDAC(0,0); //turn current off
//		bridge1(3); //tri state bridge outputs
//		bridge2(3); //tri state bridge outputs
//	}
//
//	//WARNING("move %d %d",stepAngle,mA);
//	//handle roll overs, could do with modulo operator
//	stepAngle=stepAngle%SINE_STEPS;
//	//	while (stepAngle<0)
//	//	{
//	//		stepAngle=stepAngle+SINE_STEPS;
//	//	}
//	//	while (stepAngle>=SINE_STEPS)
//	//	{
//	//		stepAngle=stepAngle-SINE_STEPS;
//	//	}
//
//	//figure out our sine Angle
//	// note our SINE_STEPS is 4x of microsteps for a reason
//	angle=(stepAngle+(SINE_STEPS/8)) % SINE_STEPS;
//
//	//calculate the sine and cosine of our angle
//	sin=sine(angle);
//	cos=cosine(angle);
//
//	//if we are reverse swap the sign of one of the angels
//	if (false == forwardRotation)
//	{
//		cos=-cos;
//	}
//
//	//scale sine result by current(mA)
//	dacSin=((int32_t)mA*(int32_t)abs(sin))/SINE_MAX;
//
//	//convert value into 12bit DAC scaled to 3300mA max
//	dacSin=(int32_t)((int64_t)dacSin*(DAC_MAX))/3300;
//
//	//scale cosine result by current(mA)
//	dacCos=((int32_t)mA*(int32_t)abs(cos))/SINE_MAX;
//
//	//convert value into 12bit DAC scaled to 3300mA max
//	dacCos=(int32_t)((int64_t)dacCos*(DAC_MAX))/3300;
//
//	//WARNING("dacs are %d %d %d",dacSin,dacCos,stepLoc);
//
//	setDAC(dacSin,dacCos);
//
//	if (sin>0)
//	{
//		bridge1(1);
//	}else
//	{
//		bridge1(0);
//	}
//	if (cos>0)
//	{
//		bridge2(1);
//	}else
//	{
//		bridge2(0);
//	}
//	//	YELLOW_LED(led);
//	//	led=(led+1) & 0x01;
//	lastStepMicros=micros();
//	return stepAngle;
}
#pragma GCC pop_options //fast optimization

#endif //NEMA_23_10A_HW


