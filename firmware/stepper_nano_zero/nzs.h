/*
 * nzs.h
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
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

#ifndef NZS_H_
#define NZS_H_

#include "board.h"
#include "nzs_lcd.h"
#include "stepper_controller.h"
#include "planner.h"

typedef struct
{
	int64_t angle;
	uint16_t encoderAngle;
	uint8_t valid;
}eepromData_t;

class NZS //nano Zero Stepper
{

	public:
		void begin(void);
		void loop(void);

};


#endif /* NZS_H_ */
