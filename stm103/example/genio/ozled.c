/* The screen is divided  into �8 pages� (I know it�s not divided into rows, but pages), and 128 columns.
For example, display a dot on the top left corner, we need to send a hex number 0x01 (B0000 0001) to the data register

  stmOzOLED - 0.96' I2C 128x64 OLED Driver Library
  Driver = SSD1306
  I2C 8Bit Address = 0x78.
  2014 Copyright (c) OscarLiang.net  All right reserved.

  Author: Oscar Liang
  */
#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;

#define OLED_ADDRESS					0x78 //0x3C = 7bit address --> 0x78 (8 bit addr)
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz
#define OLED_YPIXEL 32

#define stmOzOLED_Max_X					128	//128 Pixels
#if (OLED_YPIXEL == 64)
	#define stmOzOLED_Max_Y					64	//64  Pixels
	#define OLED_ADDRESS					0x7A //0x3D = 7bit address --> 0x7A (8 bit addr)
#elif(OLED_YPIXEL == 32)
	#define stmOzOLED_Max_Y					32	//32  Pixels
	#define OLED_ADDRESS					0x78 //0x3C = 7bit address --> 0x78 (8 bit addr)
#endif

// registers
#define stmOzOLED_COMMAND_MODE				0x80
#define stmOzOLED_DATA_MODE				0x40

// commands
#define stmOzOLED_CMD_DISPLAY_OFF			0xAE
#define stmOzOLED_CMD_DISPLAY_ON			0xAF
#define stmOzOLED_CMD_NORMAL_DISPLAY		0xA6
#define stmOzOLED_CMD_INVERSE_DISPLAY		0xA7
#define stmOzOLED_CMD_SET_BRIGHTNESS		0x81

#define stmOzOLED_RIGHT_SCROLL				0x26
#define stmOzOLED_LEFT_SCROLL				0x27
#define stmOzOLED_SET_VERTICAL_SCROLL_AREA 0xA3
#define stmOzOLED_VERTICAL_RIGHT_SCROLL	0x29
#define stmOzOLED_VERTICAL_LEFT_SCROLL		0x2A
#define stmOzOLED_CMD_ACTIVATE_SCROLL		0x2F
#define stmOzOLED_CMD_DEACTIVATE_SCROLL	0x2E

#define HORIZONTAL_ADDRESSING	0x00
#define PAGE_ADDRESSING			0x02

#define Scroll_Left				0x00
#define Scroll_Right			0x01
#define Scroll_Up				0x02
#define Scroll_Down				0x03

#define Scroll_2Frames			0x07
#define Scroll_3Frames			0x04
#define Scroll_4Frames			0x05
#define Scroll_5Frames			0x00
#define Scroll_25Frames			0x06
#define Scroll_64Frames			0x01
#define Scroll_128Frames		0x02
#define Scroll_256Frames		0x03


struct stmOzOLED {
	unsigned char addressingMode;
} g_OzOled;  // stmOzOLED object


/*
//protos
	void stmOzOLED_SendCmd(byte command);
	void stmOzOLED_sendData(byte Data);

	void printString(const char *String, byte X=255, byte Y=255, byte numChar=255);
	byte printNumber(long n, byte X=255, byte Y=255);
	byte printNumber(float float_num, byte prec=6, byte Y=255, byte numChar=255);
	void printBigNumber(const char *number, byte column=0, byte page=0, byte numChar=255);
	void drawBitmap(const byte *bitmaparray, byte X, byte Y, byte width, byte height);

	void init();

	void setCursorXY(byte Column, byte Row);
	void clearDisplay();
	//void clearPage(byte page);

	void setNormalDisplay();
	void setInverseDisplay();
	void setPowerOff();
	void setPowerOn();
	void setPageMode();
	void setHorizontalMode();
	void setBrightness(byte Brightness);

	void scrollRight(byte start, byte end, byte speed);
	void scrollLeft(byte start, byte end, byte speed);
	void scrollDiagRight();
	void scrollDiagLeft();
	void setActivateScroll(byte direction, byte startPage, byte endPage, byte scrollSpeed);
	void setDeactivateScroll();
*/

// 8x8 Font ASCII 32 - 127 Implemented
// Users can modify this to support more characters(glyphs)
// BasicFont is placed in code memory.
// This font be freely used without any restriction(It is placed in public domain)
//const unsigned char BasicFont[][8] PROGMEM = {
const unsigned char BasicFont[][8] = {
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
	{0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
	{0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
	{0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
	{0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
	{0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
	{0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
	{0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
	{0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
	{0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
	{0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
	{0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
	{0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
	{0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
	{0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
	{0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
	{0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
	{0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
	{0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
	{0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
	{0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
	{0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
	{0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
	{0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
	{0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
	{0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
	{0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
	{0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
	{0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
	{0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
	{0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
	{0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
	{0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
	{0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
	{0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
	{0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
	{0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
	{0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
	{0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
	{0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
	{0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
	{0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
	{0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
	{0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
	{0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
	{0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
	{0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
	{0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
	{0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
	{0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
	{0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
	{0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
	{0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
	{0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
	{0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
	{0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
	{0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
	{0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
	{0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
	{0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
	{0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
	{0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
	{0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
	{0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
	{0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
	{0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
	{0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
	{0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
	{0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
	{0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
	{0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
	{0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
	{0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
	{0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
	{0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00}
};


// Big numbers font, from 0 to 9 - 96 bytes each.
//const unsigned char bigNumbers [][96] PROGMEM = {
const unsigned char bigNumbers [][96] = {
{0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0,
0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xC1, 0xC0, 0xC0, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x83, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xC1, 0xC0, 0xC0, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x81, 0x83, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC1, 0x81, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x81, 0x83, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC1, 0x81, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x3C, 0x7E, 0x7E, 0x7E, 0x7E, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0xF0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

unsigned char ozledsendbuf[80];//volatile unsigned char ozledsendbuf[80];

void stmOzOLED_SendCmd(unsigned char command);
void stmOzOLED_sendData(unsigned char data);
void stmOzOLED_setCursorXY(unsigned char X, unsigned char Y);
extern void delayms(uint32_t ms);

void stmOzOLED_SendCmd(unsigned char command)
{
	ozledsendbuf[0]=stmOzOLED_COMMAND_MODE;//data mode
	ozledsendbuf[1]=command;
	stm_I2C_SendBurst(OLED_ADDRESS, &ozledsendbuf[0], 2);//I2CSendBurst(OLED_ADDRESS, ozledsendbuf, 2);
}

void stmOzOLED_sendData(unsigned char data){
	ozledsendbuf[0]=stmOzOLED_DATA_MODE;//data mode
	ozledsendbuf[1]=data;
	stm_I2C_SendBurst(OLED_ADDRESS, &ozledsendbuf[0], 2);//I2CSendBurst(OLED_ADDRESS, ozledsendbuf, 2);
}

void stmOzOLED_printChar(char C, unsigned char X, unsigned char Y){
	unsigned char i;
	if ( X < 128 ) stmOzOLED_setCursorXY(X, Y);

	//Ignore unused ASCII characters. Modified the range to support multilingual characters.
    if(C < 32 || C > 127)	C='*'; //star - indicate characters that can't be displayed

    for(i=0; i<8; i++) {      //read bytes from code memory
    		//stmOzOLED_sendData(pgm_read_byte(&BasicFont[C-32][i])); //font array starts at 0, ASCII starts at 32. Hence the translation
    	stmOzOLED_sendData(BasicFont[C-32][i]); //font array starts at 0, ASCII starts at 32. Hence the translation
    }
}

void stmOzOLED_printString(const char *String, unsigned char X, unsigned char Y, unsigned char numChar){
#ifdef USE_OLED
	unsigned char count=0;
	if ( X < 128 ) stmOzOLED_setCursorXY(X, Y);
   	while(String[count] && (count<numChar)){
   		stmOzOLED_printChar(String[count++],X+count,Y); //was stmOzOLED_printChar(String[count++],X,Y);
	}
#endif
}


unsigned char stmOzOLED_printNumber(long long_num, unsigned char X, unsigned char Y){

	if ( X < 128 )  stmOzOLED_setCursorXY(X, Y);

	unsigned char char_buffer[10] = "";
	unsigned char i = 0;
	unsigned char f = 0; // number of characters
	unsigned char xpos=0;

	if (long_num < 0) {

		f++;
		stmOzOLED_printChar('-',X,Y);
		long_num = -long_num;

	}
	else if (long_num == 0) {

		f++;
		stmOzOLED_printChar('0',X,Y);
		return f;

	}

	while (long_num > 0) {

		char_buffer[i++] = long_num % 10;
		long_num /= 10;

	}

	f += i;
	xpos=0;
	for(; i > 0; i--) {
		xpos++;
		stmOzOLED_printChar('0'+ char_buffer[i - 1],X+xpos,Y);

	}

	return f;

}
/*
byte stmOzOLED_printNumber(float float_num, unsigned char prec, unsigned char X, unsigned char Y){

	if ( X < 128 )
		stmOzOLED_setCursorXY(X, Y);

// prec - 6 maximum

	byte num_int = 0;
	byte num_frac = 0;
	byte num_extra = 0;

	long d = float_num; // get the integer part
	float f = float_num - d; // get the fractional part


	if (d == 0 && f < 0.0){

		printChar('-');
		num_extra++;
		printChar('0');
		num_extra++;
		f *= -1;

	}
	else if (d < 0 && f < 0.0){

		num_int = printNumber(d); // count how many digits in integer part
		f *= -1;

	}
	else{

		num_int = printNumber(d); // count how many digits in integer part

	}

	// only when fractional part > 0, we show decimal point
	if (f > 0.0){

		printChar('.');
		num_extra++;

		long f_shift = 1;

		if (num_int + prec > 8)
			prec = 8 - num_int;

		for (byte j=0; j<prec; j++){
			f_shift *= 10;
		}

		num_frac = printNumber((long)(f*f_shift)); // count how many digits in fractional part

	}

	return num_int + num_frac + num_extra;

}
*/


void stmOzOLED_printBigNumber(const char *number, unsigned char X, unsigned char Y, unsigned char numChar){
// big number pixels: 24 x 32
	unsigned char i;
 // Y - page
	unsigned char column = 0;
	unsigned char count = 0;

	while(number[count] && count<numChar){


		stmOzOLED_setCursorXY( X, Y);

		for(i=0; i<96; i++) {

			// if character is not "0-9" or ':'
			if(number[count] < 48 || number[count] > 58)
				stmOzOLED_sendData(0);
			else
				stmOzOLED_sendData(bigNumbers[number[count]-48][i]);//stmOzOLED_sendData(pgm_read_byte(&bigNumbers[number[count]-48][i]));


			if(column >= 23){
				column = 0;
				stmOzOLED_setCursorXY(X, ++Y);
			}
			else
				column++;

		}

		count++;

		X = X + 3;
		Y = Y - 4;


	}
}

/*
void stmOzOLED_drawBitmap(const unsigned char *bitmaparray, unsigned char X, unsigned char Y, unsigned char width, unsigned char height){
	unsigned char i;
// max width = 16
// max height = 8

	stmOzOLED_setCursorXY( X, Y );

	unsigned char column = 0;
	for(i=0; i<width*8*height; i++) {

		stmOzOLED_sendData( pgm_read_byte(&bitmaparray[i]));

		if(++column == width*8) {
			column = 0;
			stmOzOLED_setCursorXY( X, ++Y );
		}
	}

}
*/

void stmOzOLED_setCursorXY(unsigned char X, unsigned char Y){
	// Y - 1 unit = 1 page (8 pixel rows)
	// X - 1 unit = 8 pixel columns

    stmOzOLED_SendCmd(0x00 + (8*X & 0x0F)); 		//set column lower address
    stmOzOLED_SendCmd(0x10 + ((8*X>>4)&0x0F)); 		//set column higher address
	stmOzOLED_SendCmd(0xB0 + Y); 					//set page address

}


void stmOzOLED_clearDisplay(void)	{
	unsigned char page;
	unsigned char column;

	for(page=0; page<8; page++) {

		stmOzOLED_setCursorXY(0, page);
		for(column=0; column<128; column++){  //clear all columns
			stmOzOLED_sendData(0);
		}

	}

	stmOzOLED_setCursorXY( 0,0);

}

/*
void stmOzOLED_clearPage(byte page)	{
	// clear page and set cursor at beginning of that page

	setCursorXY(0, page);
	for(byte column=0; column<128; column++){  //clear all columns
		stmOzOLED_sendData(0x00);
	}

}
*/


void stmOzOLED_setInverseDisplay(void){

	stmOzOLED_SendCmd(stmOzOLED_CMD_INVERSE_DISPLAY);

}

void stmOzOLED_setNormalDisplay(void){

	stmOzOLED_SendCmd(stmOzOLED_CMD_NORMAL_DISPLAY);

}

void stmOzOLED_setPowerOff(void){
	stmOzOLED_SendCmd(stmOzOLED_CMD_DISPLAY_OFF);
}

void stmOzOLED_setPowerOn(void){

	stmOzOLED_SendCmd(stmOzOLED_CMD_DISPLAY_ON);

}

void stmOzOLED_setBrightness(unsigned char Brightness){

	stmOzOLED_SendCmd(stmOzOLED_CMD_SET_BRIGHTNESS);
	stmOzOLED_SendCmd(Brightness);

}

void stmOzOLED_setPageMode(void){
	g_OzOled.addressingMode = PAGE_ADDRESSING;
	stmOzOLED_SendCmd(0x20); 				//set addressing mode
	stmOzOLED_SendCmd(PAGE_ADDRESSING); 	//set page addressing mode
}

void stmOzOLED_setHorizontalMode(){
	g_OzOled.addressingMode = HORIZONTAL_ADDRESSING;
	stmOzOLED_SendCmd(0x20); 				//set addressing mode
	stmOzOLED_SendCmd(HORIZONTAL_ADDRESSING); 	//set page addressing mode
}


// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// scrollRight(0x00, 0x0F)  - start - stop
void stmOzOLED_scrollRight(unsigned char start, unsigned char end, unsigned char speed){

    stmOzOLED_SendCmd(stmOzOLED_RIGHT_SCROLL);  //Horizontal Scroll Setup
    stmOzOLED_SendCmd(0x00);	// dummy byte
    stmOzOLED_SendCmd(start);	// start page address
    stmOzOLED_SendCmd(speed);	// set time interval between each scroll
    stmOzOLED_SendCmd(end);	// end page address

    stmOzOLED_SendCmd(0x01);
    stmOzOLED_SendCmd(0xFF);

    stmOzOLED_SendCmd(0x2f);  //active scrolling

}


// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)   - start - stop
void stmOzOLED_scrollLeft(unsigned char start, unsigned char end, unsigned char speed){

    stmOzOLED_SendCmd(stmOzOLED_LEFT_SCROLL);  //Horizontal Scroll Setup
    stmOzOLED_SendCmd(0x00);	// dummy byte
    stmOzOLED_SendCmd(start);	// start page address
    stmOzOLED_SendCmd(speed);	// set time interval between each scroll
    stmOzOLED_SendCmd(end);	// end page address

    stmOzOLED_SendCmd(0x01);
    stmOzOLED_SendCmd(0xFF);

    stmOzOLED_SendCmd(0x2f);  //active scrolling

}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void stmOzOLED_scrollDiagRight(){

        stmOzOLED_SendCmd(stmOzOLED_SET_VERTICAL_SCROLL_AREA);
        stmOzOLED_SendCmd(0X00);
        stmOzOLED_SendCmd(stmOzOLED_Max_Y);
        stmOzOLED_SendCmd(stmOzOLED_VERTICAL_RIGHT_SCROLL); //Vertical and Horizontal Scroll Setup
        stmOzOLED_SendCmd(0X00); 	//dummy byte
        stmOzOLED_SendCmd(0x00);	 //define page0 as startpage address
        stmOzOLED_SendCmd(0X00);	//set time interval between each scroll ste as 6 frames
        stmOzOLED_SendCmd(0x07);	//define page7 as endpage address
        stmOzOLED_SendCmd(0X01);	//set vertical scrolling offset as 1 row
        stmOzOLED_SendCmd(stmOzOLED_CMD_ACTIVATE_SCROLL); //active scrolling

}

void stmOzOLED_scrollDiagLeft(){

        stmOzOLED_SendCmd(stmOzOLED_SET_VERTICAL_SCROLL_AREA);
        stmOzOLED_SendCmd(0X00);
        stmOzOLED_SendCmd(stmOzOLED_Max_Y);
        stmOzOLED_SendCmd(stmOzOLED_VERTICAL_LEFT_SCROLL); //Vertical and Horizontal Scroll Setup
        stmOzOLED_SendCmd(0X00); //dummy byte
        stmOzOLED_SendCmd(0x00);	 //define page0 as startpage address
       stmOzOLED_SendCmd(0X00);	//set time interval between each scroll ste as 6 frames
        stmOzOLED_SendCmd(0x07);	//define page7 as endpage address
        stmOzOLED_SendCmd(0X01);	//set vertical scrolling offset as 1 row
        stmOzOLED_SendCmd(stmOzOLED_CMD_ACTIVATE_SCROLL); //active scrolling

}


void stmOzOLED_setActivateScroll(unsigned char direction, unsigned char startPage, unsigned char endPage, unsigned char scrollSpeed){


/*
This function is still not complete, we need more testing also.
Use the following defines for 'direction' :

 Scroll_Left
 Scroll_Right

For Scroll_vericle, still need to debug more...

Use the following defines for 'scrollSpeed' :

 Scroll_2Frames
 Scroll_3Frames
 Scroll_4Frames
 Scroll_5Frames
 Scroll_25Frames
 Scroll_64Frames
 Scroll_128Frames
 Scroll_256Frames

*/


	if(direction == Scroll_Right) {

		//Scroll Right
		stmOzOLED_SendCmd(0x26);

	}
	else {

		//Scroll Left
		stmOzOLED_SendCmd(0x27);

	}
	/*
	else if (direction == Scroll_Up ){

		//Scroll Up
		stmOzOLED_SendCmd(0x29);

	}
	else{

		//Scroll Down
		stmOzOLED_SendCmd(0x2A);

	}
	*/
	stmOzOLED_SendCmd(0x00);//dummy byte
	stmOzOLED_SendCmd(startPage);
	stmOzOLED_SendCmd(scrollSpeed);
	stmOzOLED_SendCmd(endPage);		// for verticle scrolling, use 0x29 as command, endPage should = start page = 0

	/*
	if(direction == Scroll_Up) {

		stmOzOLED_SendCmd(0x01);

	}
	*/

	stmOzOLED_SendCmd(stmOzOLED_CMD_ACTIVATE_SCROLL);

}

void stmOzOLED_setDeactivateScroll(){

	stmOzOLED_SendCmd(stmOzOLED_CMD_DEACTIVATE_SCROLL);

}

bool g_OzLED_InitDone;
//extern bool g_I2C_InitDone;
extern unsigned char g_bI2CModuleConfigDone;
// =================== High Level ===========================
void stmOzOLED_init(char *str){

	if(g_OzLED_InitDone){
		printf("stmOzOLED_init() Already Done.\r\n");
		return;
	}

	if(!g_bI2CModuleConfigDone){
#ifdef USE_OLED
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
	gI2Cx = I2C2;
#else
	gI2Cx = I2C1;
#endif
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,I2C_SPEED);//100 or 400Kbps - for OLED
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}else{
		printf("stmOzOLED> I2C Init Already Done.");
	}

    stmOzOLED_setPowerOff(); 		//display off
    delayms(10);

    stmOzOLED_setPowerOn();			//display on (0xAF)

    delayms(10);
    stmOzOLED_setNormalDisplay();  	//default Set Normal Display
	stmOzOLED_setPageMode();		// default addressing mode
	stmOzOLED_clearDisplay();
	stmOzOLED_setCursorXY(0,0);

	stmOzOLED_SendCmd(0x8d); 		//SSD1306: ChargePump Setting
	stmOzOLED_SendCmd(0x14); 		//Enable ChargPump

	stmOzOLED_printString(str,1,0,15); //Print the String

	g_OzLED_InitDone = 1;
}

void stmOzOled_showClock(unsigned h, unsigned m, unsigned s, unsigned ypos)
{
	char str[20];
	sprintf(str,"%02u:%02u:%02u",h, m, s);
	stmOzOLED_printString(str,1,ypos,15);
}

void stmOzOled_Loop(){
	int i;
	char str[10];

	printf("OzOled Loop\r\n");
	stmOzOLED_init("HELLO");
	delayms(100);

	i=9;
	while(1){
	  sprintf(str,"%d",i);
	  stmOzOLED_printBigNumber( str,10,2,1);
	  printf("%d",i);
	  i--;
	  if(i==0)
		  i=9;
	  delayms(1000);
  };
}
