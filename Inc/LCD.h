/*
 * LCD.h
 *
 *  Created on: 2 Jul 2019
 *      Author: sshan
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

#define CLR_DIS_SCR				0x01
#define RETURN_HOME				0x02
#define SHIFT_CURSOR_LEFT  	 	0x04
#define SHIFT_CURSOR_RIGHT		0x06
#define SHIFT_DIS_LEFT  	 	0x07
#define SHIFT_DIS_RIGHT			0x05
#define SHIFT_ENTIRE_DIS_LEFT	0x18
#define SHIFT_ENTIRE_DIS_RIGHT	0x1C
#define DIS_OFF_CUR_OFF			0x08
#define DIS_ON_CUR_OFF			0x0C
#define DIS_ON_CIR_ON			0x0E
#define DIS_ON_CUR_BLINK		0x0F
#define FORCE_CUR_1st			0x80
#define FORCE_CUR_2nd			0xC0
#define DIS_OFF					0x00

#define BUSY_FLAG				0x80
#define BUSY_DELAY				4

#define CGRAM_ADDR				0x40
#define DDRAM_ADDR				0x80

#define LINE_1_ADDR				0x00			// DDRAM address at line 1
#define LINE_2_ADDR				0x40			// DDRAM address at line 2

#define MODE_4BIT_1_LINE		0x20
#define MODE_4BIT_2_LINE		0x28
#define LINE_1					0x00
#define LINE_2					0x40

#define ENTRY_MODE				0x06

#define hex0					0x30
#define hex1					0x31
#define hex2					0x32
#define hex3					0x33
#define hex4					0x34
#define hex5					0x35
#define hex6					0x36
#define hex7					0x37
#define hex8					0x38
#define hex9					0x39

// send command or data
typedef enum{
  COMMAND_REG,
  DATA_REG
}Register;

void lcdWriteNibble(uint8_t data);
uint8_t lcdReadNibble();
void lcdWrite4BitData(uint8_t data);
void toggle();
void lcdWriteCmd(uint8_t msg);
void lcdWriteMsg(uint8_t msg);
void lcdClrScr();
void DelayUs(volatile uint32_t us);
uint8_t waitBusy();
void lcdReturnHome();
void lcdDisplayString(char *msg);
void lcdFuncSet();
void lcdEntryMode();
void lcdDisplayOn();
void displayNextLine();
void lcdSetLocation(uint8_t location, uint8_t line);
void lcdSetCustomLoc(uint8_t position, uint8_t line, uint8_t data);
void lcdCreateCustom(uint8_t data, uint8_t* data_bytes);
void lcdInit();

#endif /* LCD_H_ */
