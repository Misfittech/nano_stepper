/*
 * nzs_lcd.h
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
 *
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

#ifndef NZS_LCD_H_
#define NZS_LCD_H_

#include "Arduino.h"
#include "syslog.h"
#include "board.h"
#include "stepper_controller.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "gfxfont.h"


typedef struct {
	char str[15];
} options_t;

typedef struct {
	char str[15];

	//only one of the following should be not null
	int (*func)(int argc, char *argv[]);
	options_t *ptrOptions;

} menuItem_t;





class LCD
{
	private:
		bool displayEnabled;
		Adafruit_SSD1306 display;
		StepperCtrl *ptrStepperCtrl;
		menuItem_t *ptrMenu;
		int32_t menuIndex;
		bool menuActive;

		options_t *ptrOptions;
		int32_t optionIndex;

		int32_t buttonState;

		void updateLCD(void);
		void showMenu(void);
		void updateMenu(void);
		void showOptions(void);
	public:
		void forceMenuActive(void);
		void setMenu(menuItem_t *pMenu);
		void begin(StepperCtrl *ptrStepperCtrl); //sets up the LCD
		void process(void); //processes the LCD and updates as needed
		void showSplash(void);
		void lcdShow(const char *line1, const char *line2,const char *line3);


};


#endif /* NZS_LCD_H_ */
