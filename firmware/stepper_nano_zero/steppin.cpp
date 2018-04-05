#include "steppin.h"
#include "stepper_controller.h"
#include "wiring_private.h"

extern StepperCtrl stepperCtrl;

volatile int32_t stepsChanged=0;
#define WAIT_TC32_REGS_SYNC(x) while(x->COUNT32.STATUS.bit.SYNCBUSY);

#if (PIN_STEP_INPUT != 0)
#error "this code only works with step pin being D0 (PA11, EXTINT11)"
#endif

int32_t getSteps(void)
{
//#ifndef USE_NEW_STEP
//	return 0;
//#endif
	int32_t x;
#ifdef USE_TC_STEP
	int32_t y;

	WAIT_TC32_REGS_SYNC(TC4)
	x=(int32_t)TC4->COUNT32.COUNT.reg;
	y=x-stepsChanged;
	stepsChanged=x;
	return y;

#else
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT11;
	x=stepsChanged;
	stepsChanged=0;
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT11;
	return x;
#endif
}
//this function is called on the rising edge of a step from external device
static void stepInput(void)
{
	static int dir;
	//read our direction pin
	dir = digitalRead(PIN_DIR_INPUT);

	if (CW_ROTATION == NVM->SystemParams.dirPinRotation)
	{
		dir=!dir; //reverse the rotation
	}

#ifndef USE_NEW_STEP
	stepperCtrl.requestStep(dir,1);
#else
	if (dir)
	{
		stepsChanged++;
	}else
	{
		stepsChanged--;
	}
#endif
}

void enableEIC(void)
{
	if (EIC->CTRL.bit.ENABLE == 0)
	{
		// Enable GCLK for IEC (External Interrupt Controller)
		GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));

		// Enable EIC
		EIC->CTRL.bit.ENABLE = 1;
		while (EIC->STATUS.bit.SYNCBUSY == 1) { }
	}
}


void setupStepEvent(void)
{
	//we will set up the EIC to generate an even on rising edge of step pin
	//make sure EIC is setup
	enableEIC();


	// Assign step pin to EIC
	// Step pin is PA11, EXTINT11
	pinPeripheral(PIN_STEP_INPUT, PIO_EXTINT);

	//***** setup EIC ******
	EIC->EVCTRL.bit.EXTINTEO11=1; //enable event for EXTINT11
	//setup up external interurpt 11 to be rising edge triggered
	EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE3(EIC_CONFIG_SENSE3_HIGH_Val);

	//diable actually generating an interrupt, we only want event triggered
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT11;

	//**** setup the event system ***
	// Enable GCLK for EVSYS channel 0
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EVSYS_CHANNEL_0));
	while (GCLK->STATUS.bit.SYNCBUSY);
	EVSYS->CHANNEL.reg=EVSYS_CHANNEL_CHANNEL(0)
								| EVSYS_CHANNEL_EDGSEL_RISING_EDGE
								| EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11)
								| EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

	EVSYS->USER.reg = 	EVSYS_USER_CHANNEL(1)
								| EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);

	//**** setup the Timer counter ******
	// Enable GCLK for TC4 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;  //reset TC4
	WAIT_TC32_REGS_SYNC(TC4)

	TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32    // Set Timer counter Mode to 32 bits
	| TC_CTRLA_WAVEGEN_MFRQ  //normal counting mode (not using waveforms)
	| TC_CTRLA_PRESCALER_DIV1; //count each pulse
	WAIT_TC32_REGS_SYNC(TC4)

	TC4->COUNT32.CTRLBCLR.reg=0xFF; //clear all values.
	WAIT_TC32_REGS_SYNC(TC4)

	TC4->COUNT32.EVCTRL.reg=TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_COUNT; //enable event input and count
	WAIT_TC32_REGS_SYNC(TC4)

	TC4->COUNT32.COUNT.reg=0;
	WAIT_TC32_REGS_SYNC(TC4)

	// Enable TC
	TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
	WAIT_TC32_REGS_SYNC(TC4)
}

static void dirChanged_ISR(void)
{
	static int dir;
	//read our direction pin
	dir = digitalRead(PIN_DIR_INPUT);

	if (CW_ROTATION == NVM->SystemParams.dirPinRotation)
	{
		dir=!dir; //reverse the rotation
	}
	if (dir)
	{
		TC4->COUNT32.CTRLBSET.bit.DIR=1;
		WAIT_TC32_REGS_SYNC(TC4)
	} else
	{
		TC4->COUNT32.CTRLBCLR.bit.DIR=1;
		WAIT_TC32_REGS_SYNC(TC4)
	}
}


void stepPinSetup(void)
{


#ifdef USE_TC_STEP
	//attachInterrupt configures the EIC as highest priority interrupts.
	attachInterrupt(digitalPinToInterrupt(PIN_DIR_INPUT), dirChanged_ISR, CHANGE);
	setupStepEvent();


#else
	attachInterrupt(digitalPinToInterrupt(PIN_STEP_INPUT), stepInput, RISING);
	NVIC_SetPriority(EIC_IRQn, 0); //set port A interrupt as highest priority
#endif

}
