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
#ifndef __FLASH__H__
#define __FLASH__H__

#include <Arduino.h>
#include "syslog.h"


#define FLASH_PAGE_SIZE_NZS (64) //bytes
#define FLASH_ROW_SIZE (FLASH_PAGE_SIZE_NZS*4) //defined in the datasheet as 4x page size
#define FLASH_ERASE_VALUE (0xFF) //value of flash after an erase

#define FLASH_ALLOCATE(name, size) \
	__attribute__((__aligned__(FLASH_ROW_SIZE))) \
   const uint8_t name[(size+(FLASH_ROW_SIZE-1))/FLASH_ROW_SIZE*FLASH_ROW_SIZE] = { };

bool flashInit(void); //this checks that our assumptions are true

bool flashErase(const volatile void *flash_ptr, uint32_t size);
void flashWrite(const volatile void *flash_ptr,const void *data,uint32_t size);
void flashWritePage(const volatile void *flash_ptr, const void *data, uint32_t size);

//you can read by dereferencing pointer but we will add a read
static inline int32_t flashRead(const volatile void *flash_ptr, void *data, uint32_t size)
{
  memcpy(data, (const void *)flash_ptr, size);
}




#endif //__FLASH__H__
