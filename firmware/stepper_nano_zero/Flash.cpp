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
#include "Flash.h"
#include "syslog.h"

bool flashInit(void){
	if (NVMCTRL->PARAM.bit.PSZ != 3)
	{
		ERROR("FLASH PAGE SIZE is not 64 bytes");
		return false;
	}
	return true;
}


static void erase(const volatile void *flash_ptr)
{
	NVMCTRL->ADDR.reg = ((uint32_t)flash_ptr) / 2;
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
	while (!NVMCTRL->INTFLAG.bit.READY) { }
}

bool flashErase(const volatile void *flash_ptr, uint32_t size)
{
	const uint8_t *ptr = (const uint8_t *)flash_ptr;
	while (size > FLASH_ROW_SIZE) {
		erase(ptr);
		ptr += FLASH_ROW_SIZE;
		size -= FLASH_ROW_SIZE;
	}
	if (size>0)
	{
		erase(ptr);
	}
	return true; //TODO should verify the erase
}

static inline uint32_t read_unaligned_uint32(const void *data)
{
	union {
		uint32_t u32;
		uint8_t u8[4];
	} res;
	const uint8_t *d = (const uint8_t *)data;
	res.u8[0] = d[0];
	res.u8[1] = d[1];
	res.u8[2] = d[2];
	res.u8[3] = d[3];
	return res.u32;
}


void flashWrite(const volatile void *flash_ptr,const void *data, uint32_t size)
{
	uint32_t *ptrPage;
	uint8_t *destPtr;
	uint8_t *srcPtr;
	uint32_t bytesInBlock;
	__attribute__((__aligned__(4))) uint8_t buffer[FLASH_ROW_SIZE];
	uint32_t offset;

	destPtr=(uint8_t *)flash_ptr;
	srcPtr=(uint8_t *)data;

	//LOG("flash write called");
	while(size>0)
	{
		uint32_t i,j;

		//calculate the maximum number of bytes we can write in page
		offset=((uint32_t)destPtr)%(FLASH_ROW_SIZE); //offset into page
		bytesInBlock=FLASH_ROW_SIZE-offset; //this is how many bytes we need to overwrite in this page

		//LOG("offset %d, bytesInBlock %d size %d", offset, bytesInBlock,size);
		//get pointer to start of page
		ptrPage=(uint32_t *) ((((uint32_t)destPtr)/(FLASH_ROW_SIZE)) * FLASH_ROW_SIZE);

		//LOG("pointer to page %d(0x%08x) %d",(uint32_t)ptrPage,(uint32_t)ptrPage,destPtr);

		//fill page buffer with data from flash
		memcpy(buffer,ptrPage,FLASH_ROW_SIZE);

		//now fill buffer with new data that needs changing
		i=bytesInBlock;
		if (size<i)
		{
			i=size;
		}
		//LOG("changing %d bytes",i);
		memcpy(&buffer[offset],srcPtr,i);

		//erase page
		flashErase(ptrPage,FLASH_ROW_SIZE);
		//write new data to flash
		flashWritePage(ptrPage,buffer,FLASH_ROW_SIZE);

		uint32_t *ptr=(uint32_t *)buffer;
		for (j=0; j<FLASH_ROW_SIZE/4; j++)
		{
			if (*ptrPage != *ptr)
			{
				ERROR("write failed on byte %d %x %x",j,*ptrPage, *ptr);
			}
			ptrPage++;
			ptr++;
		}


		size=size-i; //decrease number of bytes to write
		srcPtr+=i; //increase pointer to next bytes to read
		destPtr+=i; //increment destination pointer
	}


}

void flashWritePage(const volatile void *flash_ptr, const void *data, uint32_t size)
{
	// Calculate data boundaries
	size = (size + 3) / 4; //convert bytes to words with rounding

	volatile uint32_t *dst_addr = (volatile uint32_t *)flash_ptr;
	const uint8_t *src_addr = (uint8_t *)data;

	if (0 != ((uint32_t)flash_ptr)%(FLASH_PAGE_SIZE))
	{
		ERROR("Flash page write must be on boundry");
		return;
	}

	// Disable automatic page write
	NVMCTRL->CTRLB.bit.MANW = 1;

	// Do writes in pages
	while (size)
	{
		// Execute "PBC" Page Buffer Clear
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
		while (NVMCTRL->INTFLAG.bit.READY == 0) { }

		// Fill page buffer
		uint32_t i;
		for (i=0; i<(FLASH_PAGE_SIZE/4) && size; i++) //we write 4 bytes at a time
		{
			*dst_addr = read_unaligned_uint32(src_addr);
			src_addr += 4;
			dst_addr++;
			size--; //size is set to number of 32bit words in first line above
		}

		// Execute "WP" Write Page
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
		while (NVMCTRL->INTFLAG.bit.READY == 0) { }
	}


}
