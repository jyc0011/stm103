/*
  LCD - A library for controling CLCD(UC164904/KS0074 Samsung) with SPI
 * linux... auxdisplay/charlcd.c and panel.c
 * 	  SPI : 1Mbps, 8 bit mode
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK - PC10
 *  MISO - PC11 -- NOT USED
 *  MOSI - PC12
 *  nCS1 - PD2
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"
//        GND    5V    VCi    CS    SI   SCK   SO   A   K
//         1                                            9
//	      +---------------------------------------------+
//+-------+---------+-----------+-----------+-----------+---------+
//| nCS0            |           |           |           | PD10    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS1            |           |           |           | PD15    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS2            |           |           |           | PE5     |
//+-----------------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
//use nCS0 of PD10
#define nCS0_CLCD_1      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_CLCD_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
//use nCS0 of PA15
#define nCS0_CLCD_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define nCS0_CLCD_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif

//use nCS1 of PD15
#define nCS1_CLCD_1      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
#define nCS1_CLCD_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}

//use nCS1 of PD15
#define nCS2_CLCD_1      {GPIO_SetBits(GPIOE, GPIO_Pin_5);}
#define nCS2_CLCD_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_5);}

#define nCS_CLCD_1 nCS0_CLCD_1
#define nCS_CLCD_0 nCS0_CLCD_0

extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSpiWrByte(unsigned char inbyte);
extern unsigned char stmSpiRdByte();
// Starting Byte = (1111 1-R/S-R(1)/W(0)-0); R/S=1 == Data; R/S=0 == Instruction.
// And then if data is 0x12 (0001 0010) --> LSB First : 0100 1000 (D0..D7) ->To be sent as 0100 0000 + 1000 0000
void stmCLCD_SendControl(unsigned char control)
{
	unsigned char lb, hb;
	lb =  (control << 7) & 0x80;
	lb |= (control << 5) & 0x40;
	lb |= (control << 3) & 0x20;
	lb |= (control << 1) & 0x10;

	hb =  (control << 3) & 0x80;
	hb |= (control << 1) & 0x40;
	hb |= (control >> 1) & 0x20;
	hb |= (control >> 3) & 0x10;

	nCS_CLCD_0;
	stmSpiWrByte(0xf8 | 0x01); 	//1111 1001 -- Starting Byte (Why it is different from the manual?)
	stmSpiWrByte(lb);			//LB (LSB FIRST)
	stmSpiWrByte(hb);			//HB
	nCS_CLCD_1;
}
void stmCLCD_SendCharData(char data){

	unsigned char lb, hb;
	//SWAP MSB FIRST INTO LSB FIRST
	//(1) Least Nibble -> Swap it, and append 0000
	lb = (data << 7) & 0x80;
	lb |= (data << 5) & 0x40;
	lb |= (data << 3) & 0x20;
	lb |= (data << 1) & 0x10;
	//(2)  Most Nibble -> Swap it, and append 0000
	hb = (data << 3) & 0x80;
	hb |= (data << 1) & 0x40;
	hb |= (data >> 1) & 0x20;
	hb |= (data >> 3) & 0x10;

	nCS_CLCD_0;
	stmSpiWrByte(0xf8 | 0x02); //1111 1010
	stmSpiWrByte(lb); //LSB First
	stmSpiWrByte(hb); //MSB Next
	nCS_CLCD_1;
}
void stmCLCD_Init(u8 SpiId, unsigned short spiMode, unsigned char data8or16,int nCS)
{

	if(SpiId == 3)
		stmSPI3_Config(1, nCS, spiMode, data8or16);
	else if(SpiId == 2)
		stmSPI2_Config(1, nCS, spiMode, data8or16);

	stmCLCD_SendControl(0x30);//Function Set (001-1(Data Length=8 bit)-
	delayms(1);
	stmCLCD_SendControl(0x0d);//Display OnOff Control(0000-11-C(curson on)-B(cursor blink off))
	delayms(1);
	stmCLCD_SendControl(0x01);//Clear Display
	delayms(1);
	stmCLCD_SendControl(0x06);//Entry mode Set (0000-01-Inc/Dec-S)
	delayms(1);
	//??
	stmCLCD_SendControl(0x3c);//Function Set(0011-1100) -- RE=1
	delayms(1);
	stmCLCD_SendControl(0x09);//Extended Function Set (0000-1-0-0-1) 5-bit, 4-line.
	delayms(1);
	stmCLCD_SendControl(0x38);//RE=0
	delayms(1);
	stmCLCD_SendControl(0x0e);//Display OnOff (0000-1-D-C-B)
	delayms(1);

	stmCLCD_SendControl(0x3c); //RE=1 (0011-1-1(RE=1)-00)
	delayms(1);
	stmCLCD_SendControl(0x07);// Entry Mode Set with Segment Bidirection Function BID (Seg80->Seg1)
	delayms(1);
	stmCLCD_SendControl(0x38); //RE=0 (0011-1-0(RE=0)-00)
	delayms(1);

	stmCLCD_SendControl(0xa0); //Set DDRAM Address...
	delayms(1);
	printf("CLCD> CLCD-SPI CONFIG DONE\r\n");
}

void stmCLCD_Loop() {
	int m = 0;
	int h=0;
	int nCS=0;
	char ch = '0';

	printf("CLCD-SPI Test\r\n");
	delayms(1000);
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
	stmCLCD_Init(USE_SPI2, stmSPIMODE0, stmSPIDATA8, nCS);
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	stmCLCD_Init(USE_SPI3, stmSPIMODE0, stmSPIDATA8,  nCS);
#endif
	while(1){
		stmCLCD_SendCharData(ch);
		delayms(100);
		if(ch=='z'){
			ch = '0';
		}else
			ch++;
	}
}


