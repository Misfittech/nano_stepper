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
#ifndef __COMMAND_H
#define __COMMAND_H

#include <Arduino.h>
#include <stdarg.h>
#include "syslog.h"
/*
 * Usage:
 *
	#include <command.h>
	#include "uart_e0.h"

	sCmdUart KeyfobCmdUart; // UART used for the keyfob command line interface

	CMD_STR(help,"Displays this message");

	//List of supported commands
	sCommand KeyfobCmds[] =
	{
		COMMAND(help),
		{"",0,""}, //End of list signal
	};

	// print out the help strings for the commands
	static int help_cmd(sCmdUart *ptrUart,int argc, char * argv[])
	{
		sCommand cmd_list;
		int i;

		//now let's parse the command
		i=0;
		memcpy(&cmd_list, &KeyfobCmds[i], sizeof(sCommand));
		while(cmd_list.function!=0)
		{

			CommandPrintf(ptrUart,(cmd_list.name));
			CommandPrintf(ptrUart,PSTR(" - "));
			CommandPrintf(ptrUart,(cmd_list.help));
			CommandPrintf(ptrUart,PSTR("\n\r"));
			i=i+1;
			memcpy(&cmd_list, &KeyfobCmds[i], sizeof(sCommand));
		}
		return 0;
	}

	uint8_t KeyfobCmdGetChar(void)
	{
		uint8_t c;
		if (UARTE0_getc(&c)!=0)
		{
			ERROR("Uart getchar failed");
			return 0;
		}
		return c;
	}
	int KeyfobCmdInit(PIN tx_pin, PIN rx_pin, uint32_t baud)
	{
		LOG("UARTE0 init");
		UARTE0_Init(tx_pin, rx_pin, baud);
		CommandInit(&KeyfobCmdUart, UARTE0_kbhit, KeyfobCmdGetChar, UARTE0_putc,NULL); //set up the UART structure
		return 0;
	}

	int KeyfobCmdProcess(void)
	{
		return CommandProcess(&KeyfobCmdUart,KeyfobCmds,' ',KEYFOB_CMD_PROMPT);
	}

	Advantages:
	1. You can actually have more than one UART/device connected to same command line interface.
	2. works with harvard machines to save SRAM space using the PSTR functionality
	3. You can swap out commands "on the fly"


 */
#define MAX_CMD_LENGTH 60
#define MAX_ARGS 10
#define MAX_ARG_LENGTH 40
#define CMD_HISTORY 3 //number of commands in history buffer
#define ASCII_BACKSPACE 0x08
#define ASCII_ESC 0x1B
#define ASCII_UP_ARROW 0x9b
#define ANSI_UP "\x1B[A\0"

#define MAX_STRING 255
//const char ANSI_UP[]= {ASCII_ESC,'[','A',0};

typedef struct {
	uint8_t (*kbhit)(void);
	uint8_t (*getch)(void);
	uint8_t (*putch)(char data);
	uint8_t (*puts)(uint8_t *buffer, uint8_t size);
	uint8_t data;
	char buffer[MAX_CMD_LENGTH];

	char bufferHist[CMD_HISTORY][MAX_CMD_LENGTH];
	uint8_t histIndex;
	uint8_t buffIndex;
	uint8_t lastChar;
}sCmdUart;


#define COMMAND(NAME)  { NAME ## _str, NAME ## _cmd, NAME ## _help}


#ifdef PGM_P //check and see if the PGM_P is defined for the AVR

//If so then we use the strings in flash not SRAM
#define CMD_STR(NAME,STR) static const char NAME ## _help[] PROGMEM = STR;  static const char NAME ## _str[] PROGMEM = #NAME;  static int NAME ##_cmd(sCmdUart *ptrUart,int, char **);
//Command structure
typedef struct
{
	PGM_P name;
	int (*function) (sCmdUart *ptrUart,int, char **);
	PGM_P help;
} sCommand;
int CommandPrintf(sCmdUart *ptrUart, const char *fmt, ...);

#else

#define CMD_STR(NAME,STR)  static char NAME ## _help[] = STR;   static char NAME ## _str[] = #NAME;  static int NAME ##_cmd(sCmdUart *ptrUart,int, char **);

//Command structure
typedef struct
{
	char *name;
	int (*function) (sCmdUart *ptrUart,int, char **);
	char *help;
} sCommand;

int CommandPrintf(sCmdUart *ptrUart, char *fmt, ...);
#endif


int CommandInit(sCmdUart *ptrUart, uint8_t (*kbhit)(void), uint8_t (*getch)(void),uint8_t (*putch)(char data),uint8_t (*puts)(uint8_t *buffer, uint8_t size));
unsigned int CommandParse(sCmdUart *ptrUart,sCommand *ptrCmds, char *str, char delimitor);
int CommandProcess(sCmdUart *ptrUart,sCommand *ptrCmds, char delimitor, char *cmdPrompt);



#endif

