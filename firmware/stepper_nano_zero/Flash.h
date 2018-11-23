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
