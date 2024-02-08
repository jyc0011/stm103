/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.2
* Date               : 07/11/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
/*
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
//For test, using CAN_Mode_LoopBack;
//Otherwise, use CAN_Mode_Normal;

//+--------+-----------+-----------+-----------+---------+---------+--------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PG7     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BEEP   |           |PB13       |PD14       | <==     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| CAN    |           |           |           |         |         |PB8/9(RX/TX)|
//+--------+-----------+-----------+-----------+---------+---------+--------+

/*
 *   *          ===================================================================
  *                                   How to use this driver
  *          ===================================================================

  *          1.  Enable the CAN controller interface clock using
  *                  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); for CAN1
  *              and RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); for CAN2
  *  @note   In case you are using CAN2 only, you have to enable the CAN1 clock.
  *
  *          2. CAN pins configuration
  *               - Enable the clock for the CAN GPIOs using the following function:
  *                   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
  *               - Connect the involved CAN pins to AF9 using the following function
  *                   GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_CANx);
  *                - Configure these CAN pins in alternate function mode by calling
  *                  the function  GPIO_Init();
  *
  *          3.  Initialise and configure the CAN using CAN_Init() and
  *               CAN_FilterInit() functions.
  *
  *          4.  Transmit the desired CAN frame using CAN_Transmit() function. The Mailbox is used to transmit.
  *
  *          5.  Check the transmission of a CAN frame using CAN_TransmitStatus()
  *              function.
  *
  *          6.  Cancel the transmission of a CAN frame using CAN_CancelTransmit()
  *              function.
  *
  *          7.  Receive a CAN frame using CAN_Recieve() function.
  *
  *          8.  Release the receive FIFOs using CAN_FIFORelease() function. : FIFO is the RXFIFO
  *
  *          9. Return the number of pending received frames using
  *              CAN_MessagePending() function.
  *
  *          10. To control CAN events you can use one of the following two methods:
  *               - Check on CAN flags using the CAN_GetFlagStatus() function.
  *               - Use CAN interrupts through the function CAN_ITConfig() at
  *                 initialization phase and CAN_GetITStatus() function into
  *                 interrupt routines to check if the event has occurred or not.
  *             After checking on a flag you should clear it using CAN_ClearFlag()
  *             function. And after checking on an interrupt event you should
  *             clear it using CAN_ClearITPendingBit() function.
 */
/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
//typedef enum {FAILED = 0, PASSED = !FAILED} eResultStatus;
//typedef enum {CAN_ROLE_SLAVE=0, CAN_ROLE_MASTER=1} CAN_ROLE;


		  /*
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00)
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10)
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20)
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30)
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40)
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50)
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60)
#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70)
		  */


#define ISTX 1
#define ISRX 0

#define CAN_MSG_ID0555 0x0555
#define CAN_MSG_ID0666 0x0666
#define CAN_MSG_ID071C 0x071C
#define CAN_MSG_ID071D 0x071D

#define CANTXMASK (0xffffffff)
#define CANRXMASK (0xffffffff)

struct _yCAN_module{
	//u8 g_can_role;
	unsigned long g_ulMsg1Count;
	unsigned long g_ulMsg2Count;
	unsigned long g_ulMsg3Count;
	unsigned long g_bMsgObj3Sent;
	unsigned long g_bErrFlag;
	unsigned long g_bRxErrFlag;
	unsigned long g_ulRxMsgCount;
	unsigned char g_bRXFlags[33];
	unsigned long g_bRXErrFlag;

	//We use 3 MsgObjects.
	CanTxMsg canMsgObject1;
	CanTxMsg canMsgObject2;
	CanTxMsg canMsgObject3;
};
struct _yCAN_module yCAN_module;

//We provide 4 Message Data.
unsigned char g_ucMsg1_4[4] = {0x00, 0xaa, 0xaa, 0xaa}; //the data of MsgObject1; msgid=0x0555
unsigned char g_ucMsgAutoResponseData2_8[8]={0x55,0x55,0xFF,0xFF,0x00,0x00,0x55,0x55}; //the data of MsgObject2; msgid=0x0666. automatic response value. You may use this value for using the value of temperature, etc.
unsigned char g_ucMsg3_6[6] = {0x31, 0x31, 0x31, 0x31, 0x31, 0x31 }; //the data of MsgObject3; msgid=0x071c
unsigned char g_ucMsg4_8[8] = {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32};//the data of MsgObject3; msgid=0x071d

/* Private variables ---------------------------------------------------------*/
vu32 gfCANrcv = 0; /* for return of the interrupt handling */
volatile unsigned char TestRx;
ErrorStatus HSEStartUpStatus;

/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
eResultStatus CAN_Polling(void);
eResultStatus CAN_Interrupt(void);

void stmCAN_Config(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	unsigned int brp;

#if(PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)  //PB8/9-CANRX/TX



	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Configure CAN1 pins
	//RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// CAN1 Periph clock enable

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE); //FOR STM32F103's PB8/9 ----***
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);// CAN Periph clock enable APB1=36MHz

	// Enable internal CAN1 RX1 interrupt IRQ channel
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn; //USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#elif (PROCESSOR == PROCESSOR_STM32F107VCT6) //PB8/9-CANRX/TX

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

	// Configure CAN1 pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_Out_PP;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE); //NEW ADDED

	// CAN1 Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);/* CAN Periph clock enable APB1=48MHz*/

	// Enable internal CAN1 RX1 interrupt IRQ channel
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn; //USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#else
	// CAN1 Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);/* CAN Periph clock enable APB1=48MHz*/

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);/* GPIO for CAN Periph clock enable -- We need ?*/
	//Connect
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

	  /* Configure CAN1 pins: RX : PD0; TX :PD1*/ //We should Pulled Up.
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //RX
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//add
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //GPIO_AF_CAN1;//GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	  // Enable CAN RX0 interrupt IRQ channel
	  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

#endif

	  /* CAN register init */
	  // use www.bittiming.can-wiki.info
	  // PHB1 Clock = 48MHz
	  // 125Kbps : Prescaler=24; No. of TQ=16; Seg1(Prop_seg+Phase_seg1)=11; Seg2=4; SamplePoint=75% OR
	  // 125Kbps : Prescaler=16; No. of TQ=24; Seg1(Prop_seg+Phase_seg1)=17; Seg2=6; SamplePoint=75%
	  CAN_DeInit(CAN1);
	  CAN_StructInit(&CAN_InitStructure); //Fills each CAN_InitStruct member with its default value.

	  CAN_InitStructure.CAN_TTCM=DISABLE; //time triggered communication mode
	  CAN_InitStructure.CAN_ABOM=DISABLE; //automatic bus-off management
	  CAN_InitStructure.CAN_AWUM=DISABLE; //automatic wake-up mode
	  CAN_InitStructure.CAN_NART=DISABLE; //non-automatic retransmission mode
	  CAN_InitStructure.CAN_RFLM=DISABLE; //Receive FIFO Locked mode
	  CAN_InitStructure.CAN_TXFP=DISABLE; //transmit FIFO priority

	  //On Loopback Mode, Tx enabled, RX is internal loopbacked. Ignore ACKs
	  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;//CAN_Mode_LoopBack;//CAN_Mode_Normal;//CAN_Mode_LoopBack;//CAN_Mode_Normal;//CAN_Mode_LoopBack;//In Loopback Mode,signals on pins still.

	  RCC_GetClocksFreq(&RCC_Clocks);
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT6))

	  //CAN-BitTimingRegister.
	  printf("PCLK1 = %uHz\r\n", RCC_Clocks.PCLK1_Frequency); //May be 72Mhz/2 = 36MHz (APB1 CLK)
	  brp = RCC_Clocks.PCLK1_Frequency;
	  brp = (brp /18) / 100000; //18TQ @ 100Kbps
#if 1
	  //SJW=4, TSEG1=12, TSEG2=5 --> BT = 18TQ, Sampled at 72%
	  //Prescaler = CAN_Clock(36MHz)/(bitRate*numTQ) = 36MHz/(100Kbps*18)
	  CAN_InitStructure.CAN_Prescaler= 20;//100Kbps..... Num tq = 18
	  CAN_InitStructure.CAN_SJW=CAN_SJW_4tq; //maximum number of time quanta of synchronisation_jump_width
	  CAN_InitStructure.CAN_BS1=CAN_BS1_12tq;//CAN_BS1_11tq; bit_segment_1
	  CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;//CAN_BS2_1tq;//bit_segment_2
#else
	  //numTQ=8, //SJW=1, TSEG1=5, TSEG2=2 --> BT = 8TQ, Prescaler=> 45
	  CAN_InitStructure.CAN_Prescaler= 45;//100Kbps..... Num tq = 8 -> 1 + 6 + 1
	  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq; //maximum number of time quanta of synchronisation_jump_width
	  CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;//CAN_BS1_11tq; bit_segment_1
	  CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;//CAN_BS2_1tq;//bit_segment_2
#endif
	  //CAN_InitStructure.CAN_Prescaler= 32;//125Kbps //	RCC_Clocks.PCLK1_Frequency/(15*500000); //=5.6 --> was 6 : PCLK1=42MHz
	  //CAN_InitStructure.CAN_Prescaler= 16;//250Kbps
	  //CAN_InitStructure.CAN_Prescaler= 4;//1Mbps
#else
	  CAN_InitStructure.CAN_Prescaler= 24;//125Kbps..... was 6;//	RCC_Clocks.PCLK1_Frequency/(15*500000); //=5.6 --> was 6 : PCLK1=42MHz
#endif

	  CAN_Init(CAN1,&CAN_InitStructure); //Apply this one.

	  printf("stmCAN> Config Done.\r\n");
}
  //Set Filter
void stmCAN_SetFilter(void){
	  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	  CAN_FilterInitStructure.CAN_FilterNumber		=0; //0..13 for CAN1; 14..27 for CAN2
	  CAN_FilterInitStructure.CAN_FilterMode		=CAN_FilterMode_IdMask; //identifier mask based filtering
	  CAN_FilterInitStructure.CAN_FilterScale		=CAN_FilterScale_32bit;
#if 1
	  CAN_FilterInitStructure.CAN_FilterIdHigh		=0x0000; //MSB 0 == accept all
	  CAN_FilterInitStructure.CAN_FilterIdLow		=0x0000; //LSB
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh	=0x0000;
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow	=0x0000;
#else
	  CAN_FilterInitStructure.CAN_FilterIdHigh		= (CAN_MSG_ID0555 & 0xffff0000) >> 16; //accept only 0x0555
	  CAN_FilterInitStructure.CAN_FilterIdLow		= CAN_MSG_ID0555 & 0x0000ffff;
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow	= 0xfffff;//all 20 bits must match
#endif

	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_FIFO0;//0;
	  CAN_FilterInitStructure.CAN_FilterActivation	=ENABLE;
	  CAN_FilterInit(&CAN_FilterInitStructure);

	  printf("stmCAN> SetFilter Done.\r\n");
}

int stmCAN_SetAdvancedFilter(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	// (1: MsgObject #1) Initialize a message object to receive CAN messages with ID 0x0555 only.
    // The expected ID must be set along with the mask to indicate that all bits in the ID must match.
	  CAN_FilterInitStructure.CAN_FilterNumber		= 0; //0..13 for CAN1; 14..27 for CAN2
	  CAN_FilterInitStructure.CAN_FilterMode		= CAN_FilterMode_IdMask; //identifier mask based filtering
	  CAN_FilterInitStructure.CAN_FilterScale		= CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh		= (CAN_MSG_ID0555 & 0xffff0000) >> 16; //accept only 0x0555
	  CAN_FilterInitStructure.CAN_FilterIdLow		= CAN_MSG_ID0555 & 0x0000ffff;
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_FIFO0;//0;
	  CAN_FilterInitStructure.CAN_FilterActivation	= ENABLE;
	  CAN_FilterInit(&CAN_FilterInitStructure);

	// (2: MsgObject #2) For the ID of CAN_MSG_ID0666, and load into message object 2 which will be
	// used for automatic response any CAN messages with this ID.
	  CAN_FilterInitStructure.CAN_FilterNumber		= 1; //0..13 for CAN1; 14..27 for CAN2
	  CAN_FilterInitStructure.CAN_FilterMode		= CAN_FilterMode_IdMask; //identifier mask based filtering
	  CAN_FilterInitStructure.CAN_FilterScale		= CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh		= (CAN_MSG_ID0666 & 0xffff0000) >> 16; //accept only 0x0666
	  CAN_FilterInitStructure.CAN_FilterIdLow		= CAN_MSG_ID0666 & 0x0000ffff;
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_FIFO0;//0;
	  CAN_FilterInitStructure.CAN_FilterActivation	= ENABLE;
	  CAN_FilterInit(&CAN_FilterInitStructure);

	// (3: MsgObject #3) For the ID of CAN_MSG_ID071C, and load into message object 3 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.

	  CAN_FilterInitStructure.CAN_FilterNumber		= 2; //0..13 for CAN1; 14..27 for CAN2
	  CAN_FilterInitStructure.CAN_FilterMode		= CAN_FilterMode_IdMask; //identifier mask based filtering
	  CAN_FilterInitStructure.CAN_FilterScale		= CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh		= (CAN_MSG_ID071C & 0xffff0000) >> 16; //accept only 071C
	  CAN_FilterInitStructure.CAN_FilterIdLow		= CAN_MSG_ID071C & 0x0000ffff;
	  CAN_FilterInitStructure.CAN_FilterMaskIdHigh	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterMaskIdLow	= 0xfffff;//all 20 bits must match
	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_FIFO0;//0;
	  CAN_FilterInitStructure.CAN_FilterActivation	= ENABLE;
	  CAN_FilterInit(&CAN_FilterInitStructure);

    //(4) For the ID of CAN_MSG_ID071D, the slave will filter it.
	  //-- NO FILTER
}

// StdID = 0x11; DLC=2; Data= 0xca,0xfe
eResultStatus stmCAN_Send_Std(void)
{
  volatile CanTxMsg TxMessage;
  u32 i = 0;
  u8 UsedTransmitMailboxNum = 0;
  u8 ecode;

  // transmit
  yCAN_module.canMsgObject1.StdId   = CAN_MSG_ID0555;//0666;//0x05555;//0x11; //msg id.
  yCAN_module.canMsgObject1.RTR		= CAN_RTR_DATA;  //NOT remote_transmission_request (CAN_RTR_Remote if use RemoteFrame)
  yCAN_module.canMsgObject1.IDE		= CAN_ID_STD; 	 //type of identifier
  yCAN_module.canMsgObject1.DLC		= 4;			 //Data Length of 4
  memcpy(yCAN_module.canMsgObject1.Data, g_ucMsg1_4, yCAN_module.canMsgObject1.DLC); //Data

  UsedTransmitMailboxNum = CAN_Transmit(CAN1,&yCAN_module.canMsgObject1);
  //printf("Tx:UsedMboxNum=%d\r\n",UsedTransmitMailboxNum);
  //Initiates and transmits a CAN frame message. ret = The number of the mailbox that is used for transmission
  while((CAN_TransmitStatus(CAN1,UsedTransmitMailboxNum) != CANTXOK) && (i < 0xFFF))  {    i++;  } //Checks the transmission status of a CAN Frame.
  if(i >= 0xFFF){
	  ecode = CAN_GetLastErrorCode(CAN1);
	  if(ecode == CAN_ErrorCode_BitDominantErr)//0x50
		  printf("CAN> Tx Fail(errCode=0x50: BitDominantErr)\r\n");
	  /*
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00)
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10)
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20)
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30)
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40)
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50)
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60)
#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70)
	  */
	  else
		  printf("CAN> Tx Fail(errCode=0x%02x)\r\n",ecode);
  }else
	  printf("CAN> TxDone.\r\n");

  CAN_CancelTransmit(CAN1,UsedTransmitMailboxNum);

}

//Extended ID = 0x1234; DLC=2; Data= 0xde,0xca
eResultStatus stmCAN_Send_Ext(void)
{
  CanTxMsg TxMessage;
  u32 i = 0;
  printf("CAN> Send_Ext.\r\n");
  // transmit 1 message
  TxMessage.StdId=0x00;
  TxMessage.ExtId=0x1234;
  TxMessage.IDE=CAN_ID_EXT;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=2;
  TxMessage.Data[0]=0xDE;
  TxMessage.Data[1]=0xCA;
  CAN_Transmit(CAN1,&TxMessage);
}

/*******************************************************************************
* Function Name  : CAN_Polling
* Description    : Configures the CAN, transmit and receive by polling
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
unsigned char stmCAN_Receive_by_Polling(void)
{
  CanRxMsg RxMessage;
  u32 i = 0;
  unsigned char ecode = 0;
  unsigned char numRxMsg, err_rxcnt;


  i = 0;
  //while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
  //{   i++;  } //Returns the number of pending received messages.

  while(i != 0xFF){
	  numRxMsg = CAN_MessagePending(CAN1,CAN_FIFO0); //Returns the number of pending received messages.
	   if(numRxMsg > 0) break;
	   else  i++;
  }
  if(i >= 0xFF ){
	  ecode = CAN_GetLastErrorCode(CAN1);
	  if(ecode != CAN_ErrorCode_NoErr){
		  err_rxcnt = CAN_GetReceiveErrorCounter(CAN1);
		  printf("errCode=0x%02x(err_rxcnt=%u)\r\n", ecode, err_rxcnt);
		  CAN_FIFORelease(CAN1,CAN_FIFO0);
		  return 0;
	  }
	  else{
		  if(numRxMsg == 0)
			  return 1;
		  else{
			  CAN_FIFORelease(CAN1,CAN_FIFO0);
			  printf("??\r\n");
		  }
	  }

  }
  printf("CAN> Rx %u Msg\r\n", numRxMsg);

  // receive
  RxMessage.StdId	= 0x00;
  RxMessage.IDE		= CAN_ID_STD;
  RxMessage.DLC		= 0;

  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //Receives a correct CAN frame.RxMessage=return pointer to a structure receive frame which contains CAN Id, CAN DLC, CAN data and FMI number

  if (RxMessage.StdId == CAN_MSG_ID0555){
	  if (RxMessage.IDE != CAN_ID_STD)  {
		  return 0;
	  }
	  //if (RxMessage.DLC != 4)  {
		//  return 0;
	  //}
	  printf("CAN> RxPassed: ");
	  return 2;
  }else{

  }

	if(RxMessage.IDE != CAN_ID_STD){
		printf("RX>EXT msgID=0x%04x len=%u ", RxMessage.ExtId, RxMessage.DLC);
	}else{
		printf("RX>STD msgID=0x%04x len=%u ",  RxMessage.StdId, RxMessage.DLC);
	}

	if (RxMessage.DLC){
		printf("Data=");
		for(i=0; i< RxMessage.DLC; i++)
			printf("%02x ", RxMessage.Data[i]);
	}

	if(RxMessage.RTR)
		printf(" : RETMOTE FREAME \r\n");
	else
		printf(" : NOT RETMOTE FREAME \r\n");

  CAN_FIFORelease(CAN1,CAN_FIFO0);
  return 2;
}

/*******************************************************************************
* Function Name  : CAN1_RX0_IRQHandler
* Description    : This function handles CAN1 RX0 interrupts
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT6))
void CAN1_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)){ //FIFO 0 Message Pending Flag

		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

		RxMessage.StdId=0x00;
		RxMessage.ExtId=0x00;
		RxMessage.IDE=0;
		RxMessage.DLC=0;
		RxMessage.FMI=0;
		RxMessage.Data[0]=0x00;
		RxMessage.Data[1]=0x00;

		CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //Move the msg in the FIFO to RxMessage.

		//Check it.
		//if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
    	//	&& (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
		{
			gfCANrcv = 2;
		}
		//else  {
		//	gfCANrcv = FAILED;
		//}
	}
	else if (CAN_GetITStatus(CAN1,CAN_IT_BOF)){
		gfCANrcv = 0;
	}
	else
		gfCANrcv = 0;
}
#else
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)){ //FIFO 0 Message Pending Flag
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

		RxMessage.StdId=0x00;
		RxMessage.ExtId=0x00;
		RxMessage.IDE=0;
		RxMessage.DLC=0;
		RxMessage.FMI=0;
		RxMessage.Data[0]=0x00;
		RxMessage.Data[1]=0x00;

		CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //Move the msg in the FIFO to RxMessage.

		//Check it.
		if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
    		&& (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
		{
			gfCANrcv = 2;
		}
		else  {
			gfCANrcv = 0;
		}
	}else
		gfCANrcv = 0;
}
#endif

int stmCanLoop(void)
{
	int i=0;
	unsigned char ecode, err_rxcnt;

	stmUser_LED_GPIO_setup(); //PC14

	stmCAN_Config(); //Pin Remap

	stmCAN_SetFilter();
	//stmCAN_SetAdvancedFilter();
#if 1
	//(1) Polling method -- OK
	while(1){
#if 0
	  // 1st example : CAN transmit at 100Kb/s and receive by polling in loopback mode.
	  stmCAN_Send_Std();

	  delayms(1000);
#else
	  TestRx =  stmCAN_Receive_by_Polling();

	  if (TestRx == 0)
	  {   // Turn off user led.
/*		  //stmUserLED_OFF();
		  ecode = 0;
		  ecode = CAN_GetLastErrorCode(CAN1);

		  if(ecode != CAN_ErrorCode_NoErr){
			  err_rxcnt = CAN_GetReceiveErrorCounter(CAN1);
			  printf("CAN> Rx Failed(err cnt = %u, err_code = 0x%02x\r\n", err_rxcnt,ecode);
		  }else
		  {
			  //No rx frame
		  }
*/
	  }
	  else if(TestRx == 1)
		  continue;
	  else if(TestRx == 2) {
		  printf("CAN> Rx Succ(%d)\r\n",i);
		  stmLedToggle();
		  delayx(50000);
	  }
	  i++;
	  //delayms(10);
#endif
	}
#else
	//(2)============= 2nd Example (using Interrupt) -- OK
   // Enable CAN FIFO0 message pending interrupt.
	//CAN_ITConfig(CAN1,CAN_IT_ERR, ENABLE);//Enables the specified CANx interrupts.
	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_BOF, ENABLE);//Enables the specified CANx interrupts.
	                                                    //CAN_IT_FMP0= FIFO 0 message pending Interrupt
														//CAN_IT_BOF = BUS OFF Interrupt

	gfCANrcv = FAILED;//initialize the value that will be returned from CAN1_IRQ

//	stmCAN_Send_Ext();

	while(1){
	 	  if (gfCANrcv == PASSED)  {
	 		  gfCANrcv = FAILED;
	 		  stmUserLED_ON();
	 		  printf("CAN> Rx Succ(%d)\r\n",i);
		 	  //delayms(100);
//		 	  stmCAN_Send_Ext();
		 	  i++;
	 	  }
	}

	//disable interrupt handling
	CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
#endif
}
