/*
 * nzs.h
 *
 *  Created on: Dec 8, 2016
 *      Author: tramp_000
 */

#ifndef NZS_H_
#define NZS_H_

#include "board.h"
#include "nzs_lcd.h"
#include "stepper_controller.h"

class NZS //nano Zero Stepper
{

	public:
		void begin(void);
		void loop(void);

};


#endif /* NZS_H_ */
