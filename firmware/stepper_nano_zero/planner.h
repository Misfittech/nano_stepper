/**********************************************************************
 *      Author: tstern
 *
	Copyright (C) 2018  MisfitTech,  All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Written by Trampas Stern for MisfitTech.

    Misfit Tech invests time and resources providing this open source code,
    please support MisfitTech and open-source hardware by purchasing
	products from MisfitTech, www.misifittech.net!
 *********************************************************************/

/*
 *  This file implements a trajectory planner for use with serial
 *  interface. It allows the smart stepper to move at constant velocity.
 *  Additionally you can move to some location at constant velocity or
 *  with a trajectory curve
 */

#ifndef PLANNER_H_
#define PLANNER_H_
#include "board.h"
#include "stepper_controller.h"

#define PLANNER_UPDATE_RATE_HZ (3000UL) //how often planner updates PID

typedef enum {
	PLANNER_NONE =0,
	PLANNER_CV =1, //constant velocity
	//PLANNER_CA =2, //constant accleration
	//PLANNER_S_CURVE =3, //s-curve move
} PlannerMode;
class Planner
{
	private:
		StepperCtrl *ptrStepperCtrl;
		volatile PlannerMode currentMode=PLANNER_NONE;
		//todo we should not use floating point, rather use "Angle"
		volatile float endAngle;
		volatile float startAngle;
		volatile float currentSetAngle;
		volatile float tickIncrement;

	public:
		void begin(StepperCtrl *ptrStepper);
		bool moveConstantVelocity(float finalAngle, float rpm); //moves to the final location at a constant RPM
		void tick(void); //this is called on regular tick interval
		void stop(void);
		bool done(void) {return currentMode==PLANNER_NONE;}
};


extern Planner SmartPlanner;


#endif /* PLANNER_H_ */
