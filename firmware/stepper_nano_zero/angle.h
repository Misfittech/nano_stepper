/**********************************************************************
	Copyright (C) 2018  MisfitTech LLC,  All rights reserved.

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
