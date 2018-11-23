/*
 * angle.h
 *
 *  Created on: Sep 1, 2016
 *      Author: TStern
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
 *********************************************************************/

#ifndef ANGLE_H_
#define ANGLE_H_
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define ANGLE_STEPS (0x010000UL)
#define ANGLE_MAX ((uint16_t)0x0FFFF)

#define ANGLE_FROM_DEGREES(x) ((int32_t) ( ((float)ANGLE_STEPS*(float)(x)+180.0)/360.0 ) )
#define ANGLE_T0_DEGREES(x) ( (float) ((float(x)*360.0)/((float)ANGLE_STEPS) ))
class Angle
{
private:
	uint16_t angle;
public:
	Angle(void) {angle=0;}
	Angle(int32_t x) {angle=(uint16_t)x;}
	Angle(const Angle &x) {angle=x.angle;}

	int16_t operator-( const Angle &a2)
	{
		int32_t x,y,dx;
		x=(int32_t)angle;
		y=(int32_t)a2.angle;
		dx=x-y;
		if (abs(x-y)>ANGLE_STEPS/2)
		{
			//we have a wrap condition
			if (x>y)
			{
				dx=x-(y+ANGLE_STEPS);
			}else if (x<y)
			{
				dx=(ANGLE_STEPS+x)-y;
			}
		}
		return (int16_t)dx;
	}
//
//	int16_t operator-( const int32_t y)
//	{
//		int32_t x,y,dx;
//		x=(int32_t)angle;
//		while(y>ANGLE_MAX)
//		{
//			y=y-ANGLE_STEPS;
//		}
//		while(y<-ANGLE_MAX)
//		{
//			y=y+ANGLE_STEPS;
//		}
//
//		dx=x-y;
//		if (abs(x-y)>ANGLE_STEPS/2)
//		{
//			//we have a wrap condition
//			if (x>y)
//			{
//				dx=x-(y+ANGLE_STEPS);
//			}else if (x<y)
//			{
//				dx=(ANGLE_STEPS+x)-y;
//			}
//		}
//		return (int16_t)dx;
//	}

	Angle operator+(const Angle &y)
	{
		uint16_t a;
		a=angle+ (uint16_t)y.angle;
		return Angle(a);
	}
	Angle operator+(const long int x)
	{
		int32_t a;
		a=(int32_t)angle+ x;
		while (a>=ANGLE_STEPS)
		{
			a=a-ANGLE_STEPS;
		}
		while (a<0)
		{
			a=a+ANGLE_STEPS;
		}
		return Angle((uint16_t)a);
	}
	Angle operator+(const unsigned long int x)
	{
		uint32_t a;
		a=(uint32_t)angle+ x;
		while (a>=ANGLE_STEPS)
		{
			a=a-ANGLE_STEPS;
		}
		return Angle((uint16_t)a);
	}

	operator uint16_t() const {return angle;}
	operator uint32_t() const {return (uint32_t)angle;}
	operator int32_t() const {return (int32_t)angle;}



};


#endif /* ANGLE_H_ */
