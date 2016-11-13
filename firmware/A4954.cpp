#include "A4954.h"
#include "wiring_private.h"
#include "syslog.h"
#include "angle.h"
#include "Arduino.h"

static uint8_t pinState=0;

#define SINE_STEPS (1024L)
#define SINE_MAX (65535L)

//prvent someone for making a mistake with the code
#if ((A4954_NUM_MICROSTEPS*4) != SINE_STEPS)
#error "SINE_STEPS must be 4x of Micro steps for the move function"
#endif

static const uint16_t sineTable[257]={
		0,402,804,1206,1608,2010,2412,2814,3216,3617,4019,4420,4821,5222,5623,6023,
		6424,6824,7223,7623,8022,8421,8820,9218,9616,10014,10411,10808,11204,11600,11996,12391,
		12785,13179,13573,13966,14359,14751,15142,15533,15924,16313,16703,17091,17479,17866,18253,18639,
		19024,19408,19792,20175,20557,20939,21319,21699,22078,22456,22834,23210,23586,23960,24334,24707,
		25079,25450,25820,26189,26557,26925,27291,27656,28020,28383,28745,29106,29465,29824,30181,30538,
		30893,31247,31600,31952,32302,32651,32999,33346,33692,34036,34379,34721,35061,35400,35738,36074,
		36409,36743,37075,37406,37736,38064,38390,38715,39039,39361,39682,40001,40319,40635,40950,41263,
		41575,41885,42194,42500,42806,43109,43411,43712,44011,44308,44603,44897,45189,45479,45768,46055,
		46340,46624,46905,47185,47464,47740,48014,48287,48558,48827,49095,49360,49624,49885,50145,50403,
		50659,50913,51166,51416,51664,51911,52155,52398,52638,52877,53113,53348,53580,53811,54039,54266,
		54490,54713,54933,55151,55367,55582,55794,56003,56211,56417,56620,56822,57021,57218,57413,57606,
		57797,57985,58171,58356,58537,58717,58895,59070,59243,59414,59582,59749,59913,60075,60234,60391,
		60546,60699,60850,60998,61144,61287,61429,61567,61704,61838,61970,62100,62227,62352,62475,62595,
		62713,62829,62942,63053,63161,63267,63371,63472,63571,63668,63762,63853,63943,64030,64114,64196,
		64276,64353,64428,64500,64570,64638,64703,64765,64826,64883,64939,64992,65042,65090,65136,65179,
		65219,65258,65293,65327,65357,65386,65412,65435,65456,65475,65491,65504,65515,65524,65530,65534,
		65535,
};





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
		//		delay(1);
	}
}



void enableTCC0(uint8_t percent)
{
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
	TCCx->CC[1].reg = (uint32_t)ulValue; //ch5
	syncTCC(TCCx);

	TCCx->CC[2].reg = (uint32_t)ulValue; //ch6
	syncTCC(TCCx);

	TCCx->CC[3].reg = (uint32_t)ulValue; //ch7
	syncTCC(TCCx);

	TCCx->CC[1].reg = (uint32_t)ulValue; //ch1 == ch5

	syncTCC(TCCx);

	// Set PER to maximum counter value (resolution : 0xFF)
	TCCx->PER.reg = 480;
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC(TCCx);
	//ERROR("Enable TCC0 DONE");

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

	// Set PER to maximum counter value (resolution : 0xFF)
	TCCx->PER.reg = 0x0FFF;
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC(TCCx);

}


void A4954::begin()
{
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


	lastStepMicros=0;
	forwardRotation=true;

	enableTCC0(90);
	setupDAC();
	return;
}

void A4954::limitCurrent(uint8_t percent)
{
	enableTCC0(percent);
	if (pinState & 0x01)
	{
		pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT);
	}
	if (pinState & 0x02)
	{
		pinPeripheral(PIN_A4954_IN1, PIO_TIMER);
	}
	if (pinState & 0x04)
	{
		pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);
	}
	if (pinState & 0x08)
	{
		pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);
	}
}

static inline void bridge1(int state)
{
	if (state==0)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN1].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN1].ulPin].bit.PMUXEN = 0;
		pinMode(PIN_A4954_IN1,OUTPUT);
		pinMode(PIN_A4954_IN2,OUTPUT);
		digitalWrite(PIN_A4954_IN1, HIGH);
		digitalWrite(PIN_A4954_IN2, LOW);
		//pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT);
		pinState=(pinState & 0x0C) | 0x1;
	}
	if (state==1)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN2].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN2].ulPin].bit.PMUXEN = 0;
		pinMode(PIN_A4954_IN2,OUTPUT);
		pinMode(PIN_A4954_IN1,OUTPUT);
		digitalWrite(PIN_A4954_IN1, LOW);
		digitalWrite(PIN_A4954_IN2, HIGH);
		//pinPeripheral(PIN_A4954_IN1, PIO_TIMER);
		pinState=(pinState & 0x0C) | 0x2;
	}
	if (state==3)
	{
		digitalWrite(PIN_A4954_IN1, LOW);
		digitalWrite(PIN_A4954_IN2, LOW);
	}
}

static inline void bridge2(int state)
{
	if (state==0)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN3].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN3].ulPin].bit.PMUXEN = 0;
		pinMode(PIN_A4954_IN3,OUTPUT);
		pinMode(PIN_A4954_IN4,OUTPUT);
		digitalWrite(PIN_A4954_IN3, HIGH);
		digitalWrite(PIN_A4954_IN4, LOW);
		//pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);
		pinState=(pinState & 0x03) | 0x4;
	}
	if (state==1)
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN4].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN4].ulPin].bit.PMUXEN = 0;
		pinMode(PIN_A4954_IN4,OUTPUT);
		pinMode(PIN_A4954_IN3,OUTPUT);
		digitalWrite(PIN_A4954_IN3, LOW);
		digitalWrite(PIN_A4954_IN4, HIGH);
		//pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);
		pinState=(pinState & 0x03) | 0x8;
	}
	if (state==3)
	{
		digitalWrite(PIN_A4954_IN3, LOW);
		digitalWrite(PIN_A4954_IN4, LOW);
	}
}




int16_t sine(uint16_t angle)
{
	int sign=1;
	int16_t ret;
	//our sine table has 1024 points per rotation so convert angle to closest step

	if (angle>=(SINE_STEPS/2))
	{
		sign=-1;
	}

	angle=angle % (SINE_STEPS/2); //limit to 0-180 as sign takes care of 180-360

	if (angle>(SINE_STEPS/4-1))  //if we are greater than 90 we need to look up table backwards
	{
		angle=(SINE_STEPS/2)-angle;
	}

	ret=(int16_t)(sineTable[angle]/2)*sign;
	return ret;
}

int16_t cosine(uint16_t angle)
{

	int sign=1;
	int16_t ret;
	//our sine table has 1024 points per rotation so convert angle to closest step

	if (angle>=(SINE_STEPS/4) and angle<(3*(SINE_STEPS/4)))
	{
		sign=-1;
	}

	angle=angle % (SINE_STEPS/2); //limit to 0-180 as sign takes care of 180-360

	if (angle>(SINE_STEPS/4-1))  //if we are greater than 90 we need to look up table backwards
	{
		angle=(SINE_STEPS/2)-angle;
	}

	//for cosine we need 90 degree phase shift
	angle=(SINE_STEPS/4)-angle;

	ret=(int16_t)(sineTable[angle]/2)*sign;
	return ret;
}


//this is precise move and modulo of A4954_NUM_MICROSTEPS is a full step.
int32_t A4954::move(int32_t stepAngle, uint32_t mA)
{
	uint16_t angle;
	int32_t cos,sin;
	int32_t dacSin,dacCos;

	//WARNING("move %d %d",stepAngle,mA);
	//handle roll overs, could do with modulo operator
	while (stepAngle<0)
	{
		stepAngle=stepAngle+SINE_STEPS;
	}
	while (stepAngle>=SINE_STEPS)
	{
		stepAngle=stepAngle-SINE_STEPS;
	}

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
	dacSin=(int32_t)((int64_t)dacSin*(0x0FFFL))/3300;

	//scale cosine result by current(mA)
	dacCos=((int32_t)mA*(int32_t)abs(cos))/SINE_MAX;

	//convert value into 12bit DAC scaled to 3300mA max
	dacCos=(int32_t)((int64_t)dacCos*(0x0FFFL))/3300;

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


