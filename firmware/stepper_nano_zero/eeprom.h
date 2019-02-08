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
#ifndef EEPROM_H_
#define EEPROM_H_
#include "Flash.h"
#include "calibration.h"
#include "board.h"

/*
 *  This EEPROM implementation provides 60bytes of "eeprom space"  (we reserve 4 bytes for overhead)
 * 	The EEPROM uses two rows of flash (256 bytes per row), which
 * 	for the SAMD21G18A this allows a minimual 200k writes, but typically 1200k
 */

typedef enum {
	EEPROM_OK =0,
	EEPROM_FAILED=1,
	EEPROM_CORRUPT=2,
} eepromError_t;


eepromError_t eepromInit(void);
int eepromWriteCache(uint8_t *ptrData, uint32_t size); //returns number bytes written to cache
eepromError_t eepromFlush(void); //flush the cache to flash memory
int eepromRead(uint8_t *ptrData, uint32_t size); //returns number of bytes actually read, whcih could be less than size requested

#endif /* EEPROM_H_ */
