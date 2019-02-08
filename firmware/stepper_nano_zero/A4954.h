/**********************************************************************
	Copyright (C) 2019  MisfitTech LLC,  All rights reserved.

 	MisfitTech uses a dual license model that allows the software to be used under
	a standard GPL open source license, or a commercial license.  The standard GPL
	license  requires that all software statically linked with MisfitTec Code is
	also distributed under the same GPL V2 license terms.  Details of both license
	options follow:

	- Open source licensing -
	MisfitTech is a free download and may be used, modified, evaluated and
	distributed without charge provided the user adheres to version two of the GNU
	General Public License (GPL) and does not remove the copyright notice or this
	text.  The GPL V2 text is available on the gnu.org web site

	- Commercial licensing -
	Businesses and individuals that for commercial or other reasons cannot comply
	with the terms of the GPL V2 license must obtain a low cost commercial license
	before incorporating MisfitTech code into proprietary software for distribution in
	any form.  Commercial licenses can be purchased from www.misfittech.net
	and do not require any source files to be changed.


	This code is distributed in the hope that it will be useful.  You cannot
	use MisfitTech's code unless you agree that you use the software 'as is'.
	MisfitTech's code is provided WITHOUT ANY WARRANTY; without even the implied
	warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
	PURPOSE. MisfitTech LLC disclaims all conditions and terms, be they
	implied, expressed, or statutory.


    Written by Trampas Stern for MisfitTech.

    Misfit Tech invests time and resources providing this open source code,
    please support MisfitTech and open-source hardware by purchasing
	products from MisfitTech, www.misifittech.net!
 *********************************************************************/
#ifndef __A4954__H__
#define __A4954__H__
#include <Arduino.h>
#include "board.h"
#include "angle.h"
#include "sine.h"

#define A4954_NUM_MICROSTEPS (256)
#define A4954_MIN_TIME_BETWEEN_STEPS_MICROS  (1000)

//prevent someone for making a mistake with the code
#if ((A4954_NUM_MICROSTEPS*4) != SINE_STEPS)
#error "SINE_STEPS must be 4x of Micro steps for the move function"
#endif



/*
 *  When it comes to the stepper driver if we use angles
 *  we will always have a rounding error. For example
 *  a 0-65536(360) angle for 1.8 degree step is 327.68 so
 *  if you increment 200 of these as 327 you have a 13.6 error
 *  after one rotation.
 *  If you use floating point the effect is the same but takes longer.
 *
 *  The only error-less accumulation system is to use native units, ie full
 *  steps and microsteps.
 *
 */

class A4954
{
private:
	uint32_t lastStepMicros; // time in microseconds that last step happened
	bool forwardRotation=true;
	volatile bool enabled=true;

public:
	void begin(void);

	//moves motor where the modulo of A4954_NUM_MICROSTEPS is a full step.
	int32_t move(int32_t stepAngle, uint32_t mA);

	uint32_t microsSinceStep(void) {return micros()-lastStepMicros;};
	void setRotationDirection(bool forward) {forwardRotation=forward;};

	void enable(bool enable);
	void limitCurrent(uint8_t percent); //higher more current
};



#endif //__A4954__H__
