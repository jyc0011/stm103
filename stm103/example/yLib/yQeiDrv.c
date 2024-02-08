/*
  QEI
	Quadrature Encoder Example.
	Using Timer peripheral, we can read incremental encoder.
	We use TIM4(PhA=PD12(TIM4_CH1); PhB=PD13(TIM4_CH2); Index=PD3)
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "core_cm3.h"    //for TIM4
#include "core_cmFunc.h" //for TIM4
#endif
#include "misc.h"
#include "cmdline.h"

//+--------+-----------+-----------+-----------+---------+---------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   |
//+--------+-----------+-----------+-----------+---------+---------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PG7
//+--------+-----------+-----------+-----------+---------+---------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |
//+--------+-----------+-----------+-----------+---------+
//| BEEP   |           |PB13       |PD14       | <==     |
//+--------+-----------+-----------+-----------+---------+
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+--------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low

//speeds
volatile int16_t leftCount;
volatile int16_t rightCount;
volatile int16_t fwdCount;
volatile int16_t rotCount;
//distances
volatile int32_t leftTotal;
volatile int32_t rightTotal;
volatile int32_t fwdTotal;
volatile int32_t rotTotal;

//PhA=PD12(TIM4_CH1); PhB=PD13(TIM4_CH2); Index=PD3

/*
void QeiReset (void)
{
  __disable_irq();
  oldLeftEncoder = 0;
  oldRightEncoder = 0;
  leftTotal = 0;
  rightTotal = 0;
  fwdTotal = 0;
  rotTotal = 0;
  TIM_SetCounter (ENCL_TIMER, 0);
  TIM_SetCounter (ENCR_TIMER, 0);
  encodersRead();
  __enable_irq();
}

void QeiRead (void)
{
  oldLeftEncoder = leftEncoder;
  leftEncoder = TIM_GetCounter (ENCL_TIMER) ;
  oldRightEncoder = rightEncoder;
  rightEncoder = -TIM_GetCounter (ENCR_TIMER) ;
  leftCount = leftEncoder - oldLeftEncoder;
  rightCount = rightEncoder - oldRightEncoder;
  fwdCount = leftCount + rightCount;
  rotCount = - (leftCount - rightCount);
  fwdTotal += fwdCount;
  rotTotal += rotCount;
  leftTotal += leftCount;
  rightTotal += rightCount;
}

void SysTick_Handler(void) //Get
{
	g_Val = TIM_GetCounter(TIM4);
	g_diff = g_oldVal – g_Val;
	printf("%d\r\n", TIM_GetCounter(TIM4));
}

void EXTI3_IRQHandler(void) //Index
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        TIM_SetCounter(TIM4, 0);            // reset encoder counter
        printf("SW!\r\n");

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
*/
unsigned short stm_Qei_GetValue(void){
	unsigned short retvals;
	retvals = TIM_GetCounter(TIM4);
}
//GPIO pin with PULL UP.
void stm_Qei_Config(uint32_t maxValue){
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
    //Configure Times as QEI Counter.
    //TIM4, PD12/13
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;//GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //for PD12/PD13

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

/*    //PD3: index line interrupt configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
*/
    // QEI counter configuration
    //TIM_TimeBaseStructure.TIM_Period = maxValue;//0xffff;  // encoder counter Max Value. On overflow, interrupt will be issued
    //TIM_TimeBaseStructure.TIM_Prescaler = 0x100;//0;//2;//0xf;//0;
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;//TIM_CKD_DIV1;//
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;//TIM_CounterMode_Up;//
    //TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 	//ConterClock...
    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising); 		//OK
    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling); 	 //

    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling); 	//
    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    	//
    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);    	//
    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);    //
    TIM_SetAutoreload (TIM4,maxValue);// 0xffff);

#else
    //Configure Times as QEI Counter.
    //TIM4, PD12/13
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //PD3: index line interrupt configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // QEI counter configuration
    TIM_TimeBaseStructure.TIM_Period = maxValue;//0xffff;  // encoder counter Max Value. On overflow, interrupt will be issued
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //Counter counts on TI2FP2 edge depending on TI1FP1 level.
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
   // TIM_SetAutoreload (TIM4,1023);// 0xffff);
#endif
    TIM_Cmd(TIM4, ENABLE);//Turn on the timer/counter

    __disable_irq();
    TIM_SetCounter(TIM4,maxValue>>1);//0);
    __enable_irq();
}
void stmQeiLoop(void)
{
	unsigned short g_oldVal, g_Val;
	unsigned short g_diff = 0;
    printf("\r\nQuadrature Encoder Test\r\n");
    //QeiReset();
    stm_Qei_Config(1023);//100);

    while(1)
    {
        //__WFI();
    	g_Val = stm_Qei_GetValue();
    	if(g_oldVal != g_Val){
    		g_oldVal = g_Val;
    		printf("%d ", g_Val);
    	}
    }
}



