#include <string.h>
#include <stdarg.h>
#include "example/yInc.h"
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
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "cmdline.h"
#include "core_cm3.h"

//#include "stm32f10x_nvic.h"
#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

#define B1_Pin GPIO_Pin_13
#define B1_GPIO_Port GPIOC
#define OUT1_Pin GPIO_Pin_2
#define OUT1_GPIO_Port GPIOC
#define OUT2_Pin GPIO_Pin_3
#define OUT2_GPIO_Port GPIOC
#define LD2_Pin GPIO_Pin_5
#define LD2_GPIO_Port GPIOA

extern void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void TIM2_IRQHandler(void);

#define DTMFBUFsz  256               // DTMF Input Buffer

typedef struct _DTMFCONFIG  {
  GPIO_InitTypeDef gpio;
  ADC_InitTypeDef  adc;
} DTMFCONFIG;

typedef struct DTMF  {
  GPIO_InitTypeDef gpio;
  ADC_InitTypeDef  adc;

  unsigned int   AIindex;         // Input Data Index
  unsigned int   AIcheck;         // Index Window Trigger for DTMF check
  unsigned char  digit;           // detected digit
  unsigned char  early;           // early detected digit
  unsigned char  new;             // set to 1 when new digit detected
  unsigned char  d[4];			  // last four detected digits
  unsigned int   d_i;             // index
  unsigned short ADInput[DTMFBUFsz];  // A/D Input Data

  unsigned short ADC_ConvertedValue;// holds the coverted Analog Value


} DTMF;

DTMFCONFIG  dtmfConf;
static DTMF dtmf_Input_Info;                // DTMF info of one input

//struct DTMF dtmf_Input_Info = { 0, N, 0 };  // DTMF info of one input


extern void DTMF_Detect (DTMF *t);// check for valid DTMF tone

// ----- Parameters and Variables for Tone Detector ----- 
// cos = (cos (2*PI*(DTMF_Freq/8000.0))) * 256*128;
#define DTMF_697Hz   27980        // DTMF Row Frequency
#define DTMF_770Hz   26956
#define DTMF_852Hz   25701
#define DTMF_941Hz   24219
#define DTMF_1209Hz  19073        // DTMF Column Frequency
#define DTMF_1336Hz  16325
#define DTMF_1477Hz  13085
#define DTMF_1633Hz   9315
#define DTMF_1394Hz  15014        // DTMF Row Frequency    2nd harm
#define DTMF_1540Hz  11583
#define DTMF_1704Hz   7549
#define DTMF_1882Hz   3032
#define DTMF_2418Hz -10565        // DTMF Column Frequency 2nd harm
#define DTMF_2672Hz -16503
#define DTMF_2954Hz -22318
#define DTMF_3266Hz -27472

#define N  114                    // Input Data deep
static  short DTMFin[N];          // Input Data of the ADC

//#define USE_DMA 1

//================================
//ADC_HandleTypeDef hadc;
//DMA_HandleTypeDef hdma_adc;
//TIM_HandleTypeDef hTIM2;
//UART_HandleTypeDef huart2;
#if USE_DMA
//#define BUFLEN 4
//volatile unsigned short aResultDMA[BUFLEN];
#endif

/*  DTMF Digit encoding */
static char DTMFchar[16] = {
  '1', '2', '3', 'A', 
  '4', '5', '6', 'B', 
  '7', '8', '9', 'C', 
  '*', '0', '#', 'D', 
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);



#if USE_DMA
static void dtmf_DMA_Init(void)
{
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  /* Set the Vector Table base location at 0x08000000 */
	  //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	  /* 2 bit for pre-emption priority, 2 bits for subpriority */
	  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	  NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	  // Enable the Ethernet global Interrupt
	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
/**
* @brief This function handles DMA1 channel 1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
	//GPIO_ToggleBits(GPIOB, GPIO_Pin_13);
    //HAL_DMA_IRQHandler(&hdma_adc);
}
#endif


#if 0
static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  //Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    //Configure for the selected ADC regular channel to be converted.

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
#else
/*  +-----+----------+--------+------+---------+
 *  | PA1 - ADC1/CH1 - DMA1/1 - TIM2 - Goetzel
 *  +-----+----------+--------+------+---------+
 */
//Init ADC @ PA1
//-ADC channel 1 is used as analog input. (12-bit resolution)
//-Use DMA channel 1 for ADC1
void dtmf_Adc_Config()
{

	DMA_InitTypeDef DMA_InitStruct;

	//PA1 : analog input (ADC1_CH1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	dtmfConf.gpio.GPIO_Pin = GPIO_Pin_1; //set to PA1
	dtmfConf.gpio.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
	GPIO_Init(GPIOA, &dtmfConf.gpio); //set PA1 to AIN for ADC1_CH1
#if USE_DMA
	//DMA Init
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);// RCC_DMA1_CLK_ENABLE();

	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStruct);
	//DMA_InitStruct.DMA_PeripheralBaseAddr = ;//DMA1_Channel1->CPAR = (unsigned)&(ADC1->DR);			//Set channel 1 peripheral address
	DMA_InitStruct.DMA_BufferSize = 1;		//DMA1_Channel1->CNDTR = 1;								//Transmit 1 word
	DMA_InitStruct.DMA_MemoryBaseAddr = (unsigned)&(dtmf_Input_Info.ADC_ConvertedValue);	//Set channel 1 memory address 	//DMA1_Channel1->CMAR = (unsigned)&dtmf_Input_Info.ADC_ConvertedValue;	//Set channel 1 memory address
	DMA_InitStruct.DMA_PeripheralBaseAddr = (unsigned)&(ADC1->DR);	//DMA1_Channel1->CPAR = (unsigned)&(ADC1->DR); //0x4001244C			//Set channel 1 peripheral address
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA_MemoryDataSize_Word;//
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);//DMA1_Channel1->CCR = 0x00002520; 						//configure DMA channel

	DMA_Cmd(DMA1_Channel1,ENABLE);          //DMA channel 1 enable //DMA1_Channel1->CCR |= (1 <<0 );						//DMA channel 1 enable
#endif
	//ADC
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //clock for ADC (max 14MHz, 72/6=12MHz)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable ADC clock

	//configure ADC parameters (Independent(no interleaved with other ADCs), Continuous, SoftwareTrigger without External Trigger
	dtmfConf.adc.ADC_NbrOfChannel  = 1;
	dtmfConf.adc.ADC_Mode = ADC_Mode_Independent;
	dtmfConf.adc.ADC_ScanConvMode = ENABLE; //DISABLE; //-- single channel...//ENABLE; //ADC_SCAN_DIRECTION_BACKWARD;
	dtmfConf.adc.ADC_ContinuousConvMode = ENABLE; //DMA
	dtmfConf.adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //No external trigger.
	dtmfConf.adc.ADC_DataAlign = ADC_DataAlign_Right;

	//dtmfConf.adc.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	//dtmfConf.adc.Resolution = ADC_RESOLUTION_12B;
	//hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//hadc.Init.LowPowerAutoWait = DISABLE;
	//  hadc.Init.LowPowerAutoPowerOff = DISABLE;
	//  hadc.Init.DiscontinuousConvMode = DISABLE;
	//  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	//  hadc.Init.DMAContinuousRequests = ENABLE;
	//  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

	ADC_Init(ADC1, &dtmfConf.adc);

	ADC_RegularChannelConfig(
			ADC1,
			ADC_Channel_1,
			1, //rank
			ADC_SampleTime_55Cycles5);//ADC_SampleTime_71Cycles5);// Sample time equal to 71.5 cycles	//PA1 as Input

	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
#if USE_DMA
	ADC_DMACmd(ADC1,ENABLE); //enable ADC1 DMA -- ADCx->CR2 |= CR2_DMA_Set;
#endif
	//enable ADC1
	ADC_Cmd(ADC1, ENABLE);


	//NVIC_EnableIRQ(ADC1_2_IRQn);

	//Calibrate ADC *optional?
	ADC_ResetCalibration(ADC1);	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	while(ADC_GetCalibrationStatus(ADC1));




	/*
	ADC_CommonInitTypeDef ADC_init;
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_init);
	ADC_CommonInit (&ADC_init);
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
	ADC_SoftwareStartConv(ADC1);
	*/

}
#endif
#if 0
/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  hTIM2.Instance = TIM2;
  hTIM2.Init.Prescaler = 5;
  hTIM2.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTIM2.Init.Period = 999;
  hTIM2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&hTIM2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&hTIM2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
#else
//8KHz timer....
void dtmf_TIM2_Init(void){
	GPIO_InitTypeDef init_AF;//
	TIM_TimeBaseInitTypeDef timer_init;
	NVIC_InitTypeDef   NVIC_InitStructure;

	//(2b) TIM init
   	//Freq = 36MHz/(Precaler+1)/(period+1)
   	//We need 8KHz
   	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

   	//8KHz
   	//Freq = TIM_CLK/(TIM_PSC+1)/(TIM_ARR+1)
   	//8000 = 72MHz/(71+1)/(124+1) = 72000000/72/125 = 8KHz
    TIM_TimeBaseStructInit(&timer_init);
   	timer_init.TIM_Prescaler 	= 71;
   	timer_init.TIM_ClockDivision = 0;
   	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
   	timer_init.TIM_Period 		= 124;//(Auto-Reload Register(ARR) at the next update event)
   	timer_init.TIM_RepetitionCounter = 0;
    //timer_init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   	TIM_TimeBaseInit(TIM2, &timer_init);
   	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Disable);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Reset);//TIM_TRGOSource_Update);//   //	//The UG(update generation) bit in the TIM_EGR register is used as the trigger output (TRGO).
                         //@arg TIM_TRGOSource_Update: The update event is selected as the trigger output (TRGO).

    TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);

   	TIM_Cmd(TIM2, ENABLE);
}
#endif


#if 0
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}
#else
void TIM2_IRQHandler(){

	int err;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) { // | TIM_IT_Trigger

    #ifdef USE_DMA
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //trigger
    	dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)] = dtmf_Input_Info.ADC_ConvertedValue;;//aResultDMA[0];
    	ADC_SoftwareStartConvCmd(ADC1, DISABLE); //may be deleted
    #else
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //trigger

    	dtmf_Input_Info.ADC_ConvertedValue = ADC_GetConversionValue(ADC1); //get data
    	dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)] = dtmf_Input_Info.ADC_ConvertedValue;//dtmf_Input_Info.ADC_ConvertedValue;//ADC_GetConversionValue(ADC1);//dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)] = HAL_ADC_GetValue(&hadc);
    	dtmf_Input_Info.AIindex++;

    	//printf("%d,", dtmf_Input_Info.ADC_ConvertedValue);

    	ADC_SoftwareStartConvCmd(ADC1, DISABLE); //may be deleted

    #endif

    	//GPIO_ToggleBits(GPIOC, GPIO_Pin_4);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
#endif
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void dtmp_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
#if 0
  /*Configure GPIO pin Output Level */
  GPIO_SetBits(GPIOC, OUT1_Pin|OUT2_Pin);// HAL_GPIO_WritePin(GPIOC, OUT1_Pin|OUT2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(LD2_GPIO_Port, LD2_Pin);//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOC, GPIO_Pin_4);//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);//GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;//B1_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IT_FALLING;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  //HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin */
  GPIO_InitStruct.GPIO_Pin = OUT1_Pin|OUT2_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.GPIO_Pin = LD2_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}
/* =========================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	int err;

#ifdef USE_DMA
	dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)] = aResultDMA[0];
#else

	err = HAL_ADC_Start(&hadc);
	if(err != HAL_OK)
		while(1);

	err = HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

	dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)] = HAL_ADC_GetValue(&hadc);
	dtmf_Input_Info.AIindex++;

	err = HAL_ADC_Stop(&hadc);
	if(err != HAL_OK)
		while(1);
#endif
	
	GPIO_ToggleBits(GPIOC, GPIO_PIN_4);
}
*/

void dtmf_dump(){
	unsigned char pos=0;
	unsigned char row=0;
	printf("ADC[00.255]=");
	for(pos=0;pos<N; pos++){
		if(pos % 16)
			printf("%03d,", DTMFin[pos]);
		else
			printf("\r\n[%u] ",row++);
	}
	printf("Done\r\n");
}
//  Calculate Power of Signal
static unsigned int Goertzel (int cos_fact)  {
  short *x;
  long  v0, v1, v2;
  int  pwr;
  int p1, p2, p01;
  unsigned int  i;

  v1  = 0;
  v2  = 0;
// 1. Compute for each sample:
// vk(n) = (2*cos(2*PI*f0/fs)) * vk(n-1) - vk(n-2) + x(n)
  x = DTMFin;
  for (i = 0; i < N; i++) {
    v0 = ((cos_fact*v1)>>14)-v2+*x;
    x++;
    v2 = v1;
    v1 = v0;
  }
// 2. Compute once every N samples:
// |X(k)|2 = vk(N)2 + vk(N-1)2 - (2*cos(2*PI*f0/fs)) * vk(N) * vk(N-1))
  p1  = v1*v1;
  p2  = v2*v2;
  p01 = v1*v2;
  pwr = p1 - (cos_fact*(p01>>14)) + p2;
  if (pwr < 0)  return (0);
  return ((pwr>>16));          //  make sure that -1 is not returned
}

//  Check Input Signal and Copy it to the DTMF Input Buffer
static void dtmf_GainControl (DTMF *t)  {
  unsigned int  v;
  unsigned int  avg;
  unsigned int  min, max;
  unsigned int  idx;
  short *d;

  min = 0xFFFF;
  max = 0;

  avg  = 0x10000L / N;         // normalize factor for average calculation
  d = &DTMFin[N]; //the last one
  idx = t->AIindex;
  do  {
    v =  t->ADInput[idx & (DTMFBUFsz-1)];
    if (v < min)  min = v;
    if (v > max)  max = v;
    avg += (0x10000L / N) * v;
    idx--;
    *--d = v;
  }  while (d != &DTMFin[0]);


  //dtmf_dump();

  avg >>= 16;                  // avarage value
  min = max - min;


  printf("min = %u, avg=%u\r\n", min, avg);

// calculate prior value in 'v'
  for (v = 0; v < 15 && (min & 0x8000)==0; v++)  {
    min <<= 1;
  }
  if (v < 7)  {
    v = 7 - v;
    for (d = &DTMFin[0]; d != &DTMFin[N]; )  {
      *d++ = ((short) (*d - avg)) >> v;
    }
    return;
  }

  v -= 7;
  for (d = &DTMFin[0]; d != &DTMFin[N]; )  {
    *d++ = ((int) (*d - avg)) << v;
  }
}

//  Check if remaining powers are outside
//  return 0 if invalid power values detected
static int chk_valid (unsigned int p[4],     // power results
                      unsigned int d,        // maximum power
                      unsigned int pref)  {  // power reference

  if (d == 0)  return 0;                     // no digit
  pref /= 8;
  if (d != 1 && p[0] > pref) return (0);
  if (d != 2 && p[1] > pref) return (0);
  if (d != 3 && p[2] > pref) return (0);
  if (d != 4 && p[3] > pref) return (0);
  return (1);
}


/*------------------------------------------------------------------------------
  DTMF Digit:  Checks for valid DTMF digit
      return  digit+0x10  or 0 for invalid digit
 *------------------------------------------------------------------------------*/
static unsigned char DTMF_digit (void)  {
  unsigned int f, rampl, campl;
  unsigned int row, col;
  unsigned int validrow, validcol;
  unsigned int p[4];

//--- Check Row Frequency -------------------------------------
  p[0] = Goertzel (DTMF_697Hz);
  p[1] = Goertzel (DTMF_770Hz);
  p[2] = Goertzel (DTMF_852Hz);
  p[3] = Goertzel (DTMF_941Hz);

  row = 0; rampl = 0x40;  // initial sensivity
  if (p[0] > rampl)  { row = 1;  rampl = p[0]; }
  if (p[1] > rampl)  { row = 2;  rampl = p[1]; }
  if (p[2] > rampl)  { row = 3;  rampl = p[2]; }
  if (p[3] > rampl)  { row = 4;  rampl = p[3]; }
  if (!chk_valid (p, row, rampl)){
	  //printf("Invalid Row\r\n");
	  validrow = 0;
	  //goto invalid;
  }
  else{
	  validrow = 1;
	  printf ("\n\r>row=%d(rowPwr=%d %d %d %d)\r\n", row, p[0], p[1],p[2],p[3]);
  }

//--- Check Col Frequency -------------------------------------
  p[0] = Goertzel (DTMF_1209Hz);
  p[1] = Goertzel (DTMF_1336Hz);
  p[2] = Goertzel (DTMF_1477Hz);
  p[3] = Goertzel (DTMF_1633Hz);

  col = 0; campl = 0x50;  // initial sensivity
  if (p[0] > campl)  { col = 1;  campl = p[0]; }
  if (p[1] > campl)  { col = 2;  campl = p[1]; }
  if (p[2] > campl)  { col = 3;  campl = p[2]; }
  if (p[3] > campl)  { col = 4;  campl = p[3]; }
  if (!chk_valid (p, col, campl)){
	  if(validrow )
		  printf("Valid Row, but Invalid Column\r\n");
	  else
		  printf("Invalid both Row and Column\r\n");
	  goto invalid;
  }else{
	  printf ("col=%d(colPwr=%d %d %d %d)\r\n", col, p[0], p[1],p[2],p[3]);
	  if(!validrow ){
		  printf("Valid Column, but Invalid Row\r\n");
		  goto invalid;
	  }else{
		  printf ("Valid both row and col = (%d,%d)\r\n", row, col);
	  }
  }

  if (col && row)  {    // valid digit detected
// Amplitute Check: col must be within -4dB..+8dB of row
    if ((rampl << 4) < campl)  goto invalid;
    if ((campl << 3) < rampl)  goto invalid;

// check 2nd harmonic
    switch (row)  {
      case 1:
        if (col == 2 || col == 3)  break;   // do not check it
        f = Goertzel (DTMF_1394Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 2:
        if (col == 3 || col == 4)  break;   // do not check it
        f = Goertzel (DTMF_1540Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 3:
        if (col == 4)  break;              // do not check it
        f = Goertzel (DTMF_1704Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 4:
        f = Goertzel (DTMF_1882Hz);
        if (f > (campl / 8))   goto invalid;
        break;
    }

    switch (col)  {
      case 1:
        f = Goertzel (DTMF_2418Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 2:
        f = Goertzel (DTMF_2672Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 3:
        if (row == 4)  break;              // do not check it
        f = Goertzel (DTMF_2954Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 4:
        f = Goertzel (DTMF_3266Hz);
        if (f > (rampl / 8))   goto invalid;
        break;
    }

// digit is valid
    return ((row-1) << 2) | (col-1) | 0x10;
  }

invalid:
	return (0);
}


/*------------------------------------------------------------------------------
  DTMF Detect
 *------------------------------------------------------------------------------*/
void DTMF_Detect (DTMF *t)  {
  unsigned char d;
  unsigned int  cnt;

  if (t->AIindex >= t->AIcheck)  {

	  dtmf_GainControl (t);                  // Copy AD Input to DTMF Buffer

	  t->AIindex &= (DTMFBUFsz-1);           // ToDo make atomic
	  t->AIcheck = t->AIindex + ((N*2)/3);   // Increment DTMF Window (Overlapping Input Buffer)

	  d = DTMF_digit();

	  t->early = d;     cnt = 0;
	  if (t->d[0] == d) cnt++;
	  if (t->d[1] == d) cnt++;
	  if (t->d[2] == d) cnt++;
	  if (t->d[3] == d) cnt++;
	  t->d[(t->d_i++ & 3)] = d;
	  if (cnt >= 2)  {
		  if (t->digit != d)  {
			  t->digit = d;
			  if (d)  t->new   = 1;
		  }
	  }
  }
}

////////////////////////////////////////////////////////////
void actuateOutput(char dtmfcode)
{
	switch(dtmfcode){
		case '1' :
			GPIO_ResetBits(OUT1_GPIO_Port, OUT1_Pin);
			break;

		case '2' :
			GPIO_SetBits(OUT1_GPIO_Port, OUT1_Pin);
			break;

		case '4' :
			GPIO_ResetBits(OUT2_GPIO_Port, OUT2_Pin);
			break;

		case '5' :
			GPIO_SetBits(OUT2_GPIO_Port, OUT2_Pin);
			break;

		default:
			break;
	}
}

void dtmf_loop(void)
{
	char dtmfcode;

	printf("Welcome to DTMF decoder \r\n");

	dtmp_GPIO_Init();

	dtmf_Adc_Config(); //(with DMA)

	dtmf_TIM2_Init();

	//enable ADC to work
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //

  while (1)
  {
		if (dtmf_Input_Info.AIindex >= dtmf_Input_Info.AIcheck)  {

			DTMF_Detect (&dtmf_Input_Info);
			//printf("Detect1:val=%x, Index=%u, Check=%u\r\n", dtmf_Input_Info.ADInput[dtmf_Input_Info.AIindex & (DTMFBUFsz-1)],dtmf_Input_Info.AIindex, dtmf_Input_Info.AIcheck);
		}


		//if (dtmf_Input_Info.early)		GPIO_SetBits(LD2_GPIO_Port, LD2_Pin);
		//else		GPIO_ResetBits(LD2_GPIO_Port, LD2_Pin);

		if (dtmf_Input_Info.new){
			dtmfcode = DTMFchar[dtmf_Input_Info.digit & 0x0F];
			printf ("%c ", dtmfcode);
			//actuateOutput(dtmfcode);
			dtmf_Input_Info.new = 0;
		}
	}

}

