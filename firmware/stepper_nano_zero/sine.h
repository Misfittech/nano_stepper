/**********************************************************************
 * sine.h
 *
 *  Created on: Dec 24, 2016
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


#ifndef SINE_H_
#define SINE_H_

#include "board.h"

#define SINE_STEPS (1024L)

#define SINE_MAX ((int32_t)(32768L))


int16_t sine(uint16_t angle);
int16_t cosine(uint16_t angle);


#endif /* SINE_H_ */
