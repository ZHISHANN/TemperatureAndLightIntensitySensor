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

void lcdReturnToHome()
{
	lcdWriteCmd(RETURN_HOME);
}

void lcdFuncSet(uint8_t mode)
{
	lcdWriteCmd(mode);
}

void lcdEntryMode()
{
	lcdWriteCmd(ENTRY_MODE);
}

void lcdDisplayStatus(uint8_t displayStatus)
{
	// display on, cursor off
	lcdWriteCmd(displayStatus);
}

void lcdGotoNextLine()
{
	lcdWriteCmd(DDRAM_ADDR | LINE_2_ADDR);
}

void lcdSetLocation(uint8_t column, uint8_t line)
{
	if(line == LINE_1_ADDR)
		lcdWriteCmd(DDRAM_ADDR | (LINE_1_ADDR + column));
	else
		lcdWriteCmd(DDRAM_ADDR | (LINE_2_ADDR + column));
}

void lcdInit()
{
	lcdClrScr();
	lcdFuncSet(MODE_8BIT_2_LINE);
	lcdDisplayStatus(DIS_ON_CUR_OFF);
	lcdEntryMode();
}

void lcdSetCustomLoc(uint8_t column, uint8_t line, uint8_t data)
{
	lcdSetLocation(column, line);
	lcdWriteMsg(data);
}

void lcdCreateCustom(uint8_t data, uint8_t* data_bytes)
{
	int i;

	// only 8 locations for custom chars
	data &= 0x07;

	// Set CGRAM address
	lcdWriteCmd(CGRAM_ADDR | (data << 3));

	// char pattern
	for (i = 0; i < 8; i++)
	{
		lcdWriteMsg(data_bytes[i]);
	}
}
