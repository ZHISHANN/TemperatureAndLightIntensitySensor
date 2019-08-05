/*
 * LCD.c
 *
 *  Created on: 2 Jul 2019
 *      Author: sshan
 */

#include "LCD.h"

void lcdDisplayString(char *msg) {
    while (msg[0] != '\0') {
    	lcdWriteMsg(msg[0]);
        msg++;
    }
}

void lcdClrScr()
{
	lcdWriteCmd(CLR_DIS_SCR);
}

void lcdReturnHome()
{
	lcdWriteCmd(RETURN_HOME);
}

void lcdFuncSet()
{
	// 4 bit, 2 line, 5x7 dots
	lcdWriteCmd(MODE_4BIT_2_LINE);
}

void lcdEntryMode()
{
	lcdWriteCmd(ENTRY_MODE);
}

void lcdDisplay(uint8_t displayStatus)
{
	// display on, cursor off
	lcdWriteCmd(displayStatus);
}

void displayNextLine()
{
	lcdWriteCmd(DDRAM_ADDR | LINE_2);
}

void lcdSetLocation(uint8_t location, uint8_t line)
{
	if(line == LINE_1_ADDR)
		lcdWriteCmd(DDRAM_ADDR | (LINE_1_ADDR + location));
	else
		lcdWriteCmd(DDRAM_ADDR | (LINE_2_ADDR + location));
}

void lcdInit()
{
	lcdFuncSet();
	//lcdClrScr();
	lcdDisplayOn();
	lcdEntryMode();
	lcdClrScr();
}

void lcdSetCustomLoc(uint8_t position, uint8_t line, uint8_t data)
{
	lcdSetLocation(position, line);
	lcdWriteMsg(data);
}

void lcdCreateCustom(uint8_t data, uint8_t* data_bytes)
{
	int i;

	// only 8 locations 0-7 for custom chars
	data &= 0x07;

	// Set CGRAM address
	lcdWriteCmd(CGRAM_ADDR | (data << 3));

	// custom char pattern
	for (i = 0; i < 8; i++)
	{
		lcdWriteMsg(data_bytes[i]);
	}
}
