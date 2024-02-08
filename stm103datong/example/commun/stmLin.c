// UART1 ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©
/*
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include <stdio.h>
#include <stdint.h>
#include "misc.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "lwip/include/stm32f4x7_eth.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "lwipopts.h"
#include "lwip/include/mem.h"
#include "yInc.h"
*/

#if 1

#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
//#include "stm32f10x_flash.h"
#include "yInc.h"

//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |  STM103M35|103-M37/M39|M70        |401
//+-----------------+-----------+-----------+-----------+---------------+------------
//| USART           | USART2    |    <--    |           |USART1
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| TXD(Pin12)      | PA2       |    <--    |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| RXD(Pin13)      | PA3       |    <--    |           |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| LINEN(Pin3)     | PB10      |    <--    |           |PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
/*
  USART LIN Master transmitter communication is possible through the following procedure:
     1. Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity,
        Mode transmitter or Mode receiver and hardware flow control values using
        the USART_Init() function.
     2. Enable the USART using the USART_Cmd() function.
     3. Enable the LIN mode using the USART_LINCmd() function.
     4. Send the break character using USART_SendBreak() function.

  USART LIN Master receiver communication is possible through the following procedure:
     1. Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity,
        Mode transmitter or Mode receiver and hardware flow control values using
        the USART_Init() function.
     2. Enable the USART using the USART_Cmd() function.
     3. Configures the break detection length using the USART_LINBreakDetectLengthConfig()
        function.
     4. Enable the LIN mode using the USART_LINCmd() function.


@note In LIN mode, the following bits must be kept cleared:
        - CLKEN in the USART_CR2 register,
        - STOP[1:0], SCEN, HDSEL and IREN in the USART_CR3 register.
*/

//Using MC33662 LIN Transceiver


// We employ 5 different LIN msgs.
// MsgID_0_2A 0b00000000 = Write for controlling Motor direction and speed
// MsgID_0_2B 0b01010000 = for reading 2 bytes
// MsgID_0_4  0b10100000 = for sending 4 bytes
// MsgID_0_8  0b11110000 = for reading 8 bytes
// MsgID_3c_8 0x3c       = Broadcast with 8 bytes data.

// Master Schedule
// MsgID_0_2A -> MsgID_0_2A -> MsgID_0_2A -> MsgID_0_2A -> MsgID_0_4 -> MsgID_0_8
//

//LIN
#define LIN_ROLE_MASTER 1
#define LIN_ROLE_SLAVE 0
#define LINMAXTIMEOUT 1000 //1sec

#define MAX_MSG_NUM (5)

struct LinMsgInfo{
	u8 msgIndex;
	u8 msgid;
	u8 needResp;
	char dleng;
};

struct LinTxBuf{
	struct LinMsgInfo msgInfo;
	u8 data[8]; //excluding crc
};
struct LinRxBuf{
	struct LinMsgInfo msgInfo;
	u8 data[8];//excluding crc
	u8 sync_rcvd;
	u8 msgid_rcvd;
  	u8 break_rcvd;
  	u8 fe_rcvd; //not used
  	u8 rcvd;
	u8 needRespBySlave;
};

//For LIN msg parsing
#define getMsgId(x) 	(x & 0x0f)
#define getParity(x) 	((x & 0xc0)>>6)
#define getMsgId54(x) 	((x & 0x30)>>4)


//LIN MSG FORMAT
//(MSB) /P1-P0-ID5-ID4-ID3-ID2-ID1-ID0(LSB); LSB first transmitted.
//P0 = ID4^ID2^ID1^ID0
// ~P1=ID5^ID4^ID3^ID1

//#define MsgID_0 0x11 //0b00-01-0001 Read for requesting status --> expect Response transaction (e.g., Button ON/OFF status)
					 //for reading : MsgId = 0001, leng=01 (msgLen=2), P0=0,P1=0
//#define MsgID_1 0x12 //0b00-01-0010 Write for controling Motor (Master: transmiting Header and Data)
					 //for writing : MsgId = 0010, leng=00 (msgLen=2), P0=0,P1=0

//parity has been calcuated.
#define MsgID_0_2A 0b00000000 //for sending 2 bytes : MsgId = 0000, leng=00 (msgLen=2), P0=0,P1=0
#define MsgID_0_2B 0b01010000 //for reading 2 bytes : Master=Header; Slave=Data[2]
							  //MsgId=0000, leng=01 (=2byte), P0=1 P1=0
#define MsgID_0_4  0b10100000 //for sending 4 bytes : Master=Header+Data[4]
							  //MsgId=0000, leng=10 (=4byte), P0=0 P1=1
#define MsgID_0_8  0b11110000 //for reading 8 bytes : Master=Header; Slave=Data[8]
							  //MsgId=0000, leng=11 (=8byte), P0=1 P1=1
#define MsgID_3c_8 0x3c       //Broadcast with 8 bytes data.

typedef enum {LinMsg0=0, LinMsg1=1, LinMsg2=2,LinMsg3=3, LinMsg4=4} LinMsgIndex;
volatile struct LinMsgInfo g_LinMsgInfoPool[MAX_MSG_NUM]={
		{0, MsgID_0_2A, 0, 2}, //for motor control
		{1, MsgID_0_2B, 1, 2},
		{2, MsgID_0_4,  0, 4},
		{3, MsgID_0_8,  1, 8},
		{4, MsgID_3c_8, 0, 8}
};
//Motor Control data
#define MotorDirectionCW 	0x01
#define MotorDirectionCCW 	0x02
#define MotorSpeedUP 		0x03
#define MotorSpeedDOWN 		0x04

struct LinTxBuf g_LinTxBuf;
struct LinRxBuf g_LinRxBuf;
unsigned char g_role;

//#define LIN_BAUDRATE 20000 //20Kbps
#define LIN_BAUDRATE 19200 //19.2Kbps
/* USART configured as follow:
	        - BaudRate = 20Kbps
	        - Word Length = 8 Bits
	        - One Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
*/

#if (USART2_FOR == USART2_FOR_LIN)
	extern void USART2_IntHandlerForLIN(void);//for LIN void USART2_IRQHandler(void);//for LIN
	#define LIN_USART USART2
#elif (USART1_FOR == USART1_FOR_LIN)
	extern void USART1_IntHandlerForLIN(void);//for LIN
	#define LIN_USART USART1
#endif

void stmLinConf(unsigned char Role)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	  //(a) Config USART2 @ PA2(TXD)/PA3(RXD)

	  /* Enable USART2 and GPIOA clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//USART(1,6)=>APB2, Others=>APB1
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	 // Configure USART2 Tx (PA2) as alternate function push-pull
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);
	    //Configure USART2 Rx (PA3) as input floating
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);

	    //USART2 configuration
		USART_InitStructure.USART_BaudRate = 19200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART2, &USART_InitStructure);/* USART configuration */

		USART_Cmd(USART2, ENABLE);// Enable USART2

		//(b) LIN CONFIG for this USART2
		//Enable LIN
		USART_LINCmd(USART2,ENABLE);
		//For Receiver
		USART_LINBreakDetectLengthConfig(USART2,USART_LINBreakDetectLength_11b);

		//(c) Interrupt Config
	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
//#ifdef LIN_ROLE_MASTER
	  USART_ITConfig(USART2, USART_IT_RXNE | USART_IT_LBD, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  //USART_ITConfig(USART2,USART_IT_LBD, ENABLE); 	//Link Break Detect
//#else
	//  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	//  USART_ITConfig(USART1,USART_IT_LBD, DISABLE); //
//#endif

#else
	  //STM407 and STM401 - USART1
	  //USART1,6=>APB2, Others=>APB1
	  //U1TX -PA9; U1RX-PA10
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Rx*/

	  //Configure USART1 for LIN : Word length = 8bits, Stop bits = 1bit, Parity,
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = LIN_BAUDRATE; //20Kbps
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART1, ENABLE);

	  //For Receiver
	  USART_LINBreakDetectLengthConfig(USART1,USART_LINBreakDetectLength_11b);
	  //Enable LIN
	  USART_LINCmd(USART1,ENABLE);//USART_Cmd(USART1, ENABLE);

	  //Enable its Rx Interrupt mode
#ifdef LIN_ROLE_MASTER
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  USART_ITConfig(USART1,USART_IT_LBD, ENABLE); //Link Break Detect
#else
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  USART_ITConfig(USART1,USART_IT_LBD, DISABLE); //
#endif

	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

#endif //PROCESSOR
}

void stmLinNodeInit(unsigned char Role){

	GPIO_InitTypeDef GPIO_InitStruct;

	stmLinConf(Role); //20Kbps UART1

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	//PB10 = LIN_EN : STM103
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
	  GPIO_SetBits(GPIOB, GPIO_Pin_10); //issue EN

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//USART_LINCmd(USART1,ENABLE);

	//PB10 = LIN_EN : STM401RET
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_SetBits(GPIOB, GPIO_Pin_10);
#else
	/*
		//PC8 = LIN_EN : STM407
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);// Enable GPIO clock
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
		  GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_SetBits(GPIOC, GPIO_Pin_8);
	*/
#endif
	printf("Make LIN enable\r\n");
}

unsigned char stmLinCksum(unsigned char lindatabyte, unsigned char stmLinCksum){
	unsigned short sum;
	sum = stmLinCksum + lindatabyte;
	sum = (sum & 0xff) + ((sum & 0xff00)>>8);
	sum = (sum & 0xff) + ((sum & 0xff00)>>8);
	return (unsigned char)(sum & 0xff);
}

unsigned char stmGetLinCksum(unsigned char lindatabyte[]){
	unsigned short cksum = 0;
	int i;
	for(i=0;i<8;i++){
		cksum += lindatabyte[i];
		if(cksum >= 256)
			cksum -= 255;
	}
	return (0xff &(~cksum));
	//return (unsigned char)(sum & 0xff);
}
void stmLinSendHeaderByMaster (unsigned char linMsgId){ //By Master

	USART_SendBreak(LIN_USART);//Send Break Signal
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
	//delayms(1); //Needed?
	//UARTBreakCtl(UART1_BASE,false); //Stop Break Signal -- NOT NEED

	//Send Sync Byte (0x55)
	g_LinRxBuf.sync_rcvd = 0;
	USART_SendData(LIN_USART, 0x55);
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}

	//Send Msg id
	USART_SendData(LIN_USART, linMsgId);
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
}

void stmLinSendData(unsigned char *p_lindata, unsigned char leng){ //By Master and Slave
	int i=0;
	unsigned char cksum = 0;

	for(i=0;i<leng;i++){
		stmLinCksum(p_lindata[i], cksum); //calc during response data transmission.
		while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
		USART_SendData(LIN_USART, p_lindata[i]);
	}
	//After sending the whole data, and finally append Checksum
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
	USART_SendData(LIN_USART, (uint8_t) cksum);


}

//By Master for reading or writing
void stmLinSendMsgByMaster(
		struct LinMsgInfo *plinMsgInfo,
		unsigned char msgindex,
		u8 *txD)
{


	plinMsgInfo->msgIndex = msgindex;
	stmLinSendHeaderByMaster(plinMsgInfo->msgid); //send header and control data by this master
	if((txD != NULL) && (!plinMsgInfo->needResp)) //for writing by master or responding by slave.
		stmLinSendData(txD, plinMsgInfo->dleng);

	if(g_role == LIN_ROLE_MASTER)
		printf("MASTER>> Sent Msg(id=0x%02x)\r\n", plinMsgInfo->msgid);
	else
		printf("SLAVE>> Sent Msg(id=0x%02x)\r\n", plinMsgInfo->msgid);


}

u8 stmLinWaitForReponse(){
	static u32 timeout;
	//printf("\r\nLinWaitForReponse()..\r\n");
	timeout = 0;
	while((!g_LinRxBuf.rcvd) && (timeout < LINMAXTIMEOUT)){
		delayms(1);
		timeout++;
	}

	if  (timeout < LINMAXTIMEOUT){
		printf("MASTER>Got a response\r\n");
		timeout = 0;
		return 1;
	}else{
		//printf("\r\n===============No response\r\n");
		timeout = 0;
		return 0;
	}
}

void stmLinMaster_MsgAppHandler(){
	u8 i;
	printf("MASTER>RxDone. Need more detailed parsing...\r\n");
	for(i=0;i<g_LinRxBuf.msgInfo.dleng;i++){
			printf("[%d]=0x%02x ", i, g_LinRxBuf.data[i]);
	}
	printf("\r\n");
	g_LinRxBuf.rcvd = 0;
}

//MASTER TASK
int stmLIN_MasterTask (void){
	unsigned char msgid, controldata;
	u8 txD[8];
	u8 i;
	u32 cnt;
	struct LinMsgInfo *plinMsgInfo;

	while(1){
		//Schedule timeslot 0 : rotate motor CW
		txD[0]=MotorDirectionCW;
		txD[1]=0x01; //any data..
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg0]; //no response, datalen =2
		stmLinSendMsgByMaster(plinMsgInfo, LinMsg0,txD);
		delayms(1000);

		//Schedule timeslot 1 : rotate motor CCW
		txD[0]=MotorDirectionCCW;
		txD[1]=0x02; //any data..
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster (plinMsgInfo,LinMsg0, txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 2 ; speedup
		txD[0]=MotorSpeedUP;
		txD[1]=0x03;
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster (plinMsgInfo,LinMsg0, txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 3 : slow down
		txD[0]=MotorSpeedDOWN;
		txD[1]=0x04;
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster(plinMsgInfo,LinMsg0,txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 4 -- Requst and Handle Response
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg1];
		stmLinSendMsgByMaster (plinMsgInfo,LinMsg1, 0); //need response, datalen =2
		if(!stmLinWaitForReponse()){//Wait for Response from the slave.
			printf("No Response\r\n");
		}else{
			stmLinMaster_MsgAppHandler();
		}
		delayms(1000);

		//Schedule timeslot 5 -- Requst and Handle Response
		plinMsgInfo = &g_LinMsgInfoPool[LinMsg3];     //need response, datalen =8
		stmLinSendMsgByMaster(plinMsgInfo,LinMsg3,0); //
		if(!stmLinWaitForReponse()){//Wait for Response from the slave.
			printf("No Response\r\n");
		}else{
			stmLinMaster_MsgAppHandler();
		}
		delayms(1000);

		//goto sleep
		stmLinGotoSleep();
		delayms(20);
		//goto Wakeup
		stmLinGotoWakeup();
		delayms(1);

	}
}

void stmLinGotoSleep(){ //Make EN Off. You will see the transition of INH Output.
#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
#else
	GPIO_ResetBits(GPIOC, GPIO_Pin_8); //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5); //EN=0:
#endif
	printf("Goto Sleep\r\n");
}

void stmLinGotoWakeup(){ //Make EN On. You will see the transition of INH Output.
#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
#else
	GPIO_SetBits(GPIOC, GPIO_Pin_8); //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5); //EN=1:
#endif
	printf("Wakeup\r\n");
}

//========== for SLAVE ==============================

char stmLinFindMsgIndexFromMsgInfoPool(u8 msgid,struct LinMsgInfo *linMsgInfoPool){
	char i;
	struct LinMsgInfo *linMsgInfo;
	linMsgInfo = linMsgInfoPool;
	for(i=0;i<MAX_MSG_NUM;i++){
		if(linMsgInfo->msgid == msgid){
			printf("Found Index = %d\r\n",i);
			return i;
		}else
			linMsgInfo++;
	}
	return -1;
}

char getDataLengFromMsgId(u8 msgid, u8 msgIndex){
	char retv;

	retv = (msgid & 0x30) >> 4;

	if(retv <= 1) 		retv = 2;
	else if(retv == 2) 	retv = 4;
	else  				retv = 8;

	if(g_LinMsgInfoPool[msgIndex].dleng == retv )
		return retv;
	else
		return -1;
}

//need some more...
void stmLinSlaveResponse(struct LinMsgInfo *plinMsgInfo){
	unsigned char linRespData[8]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8};
	if(plinMsgInfo->needResp){
		switch(plinMsgInfo->msgid){
		case MsgID_0_2B:
			stmLinSendData(linRespData, plinMsgInfo->dleng);
			break;
		case MsgID_0_8:
			stmLinSendData(linRespData, plinMsgInfo->dleng);
			break;
		default:
			printf("Slave> RX\r\n");
			break;
		}//switch
		printf("Slave>Respond(for MsgID=%x):",plinMsgInfo->msgid);
	}
}
//Parsing Data Field. (If this field is readbacked data, it will be disregarded.)
void stmLinParser(struct LinMsgInfo *plinMsgInfo){
	u8 i;

	if(g_role == LIN_ROLE_MASTER){
		if(plinMsgInfo->needResp){
			printf("MASTER>Got a response from the Slave = ");
			if(plinMsgInfo->dleng != g_LinRxBuf.msgInfo.dleng){ //discard
				g_LinRxBuf.rcvd = 0;
			}else{
				for(i=0; i<plinMsgInfo->dleng;i++)	printf("%02x ", g_LinRxBuf.data[i]);
				printf("\r\n");
				g_LinRxBuf.rcvd = 1;
			}
		}else{//Readbacked data-- Discard
			g_LinRxBuf.rcvd = 0;
		}
	}else{ //as a slave -- need to response by sending data.
		if(plinMsgInfo->needResp){
			g_LinRxBuf.rcvd = 0; //Readbacked data -- discard
		}else{
			if(plinMsgInfo->dleng != g_LinRxBuf.msgInfo.dleng){ //discard
				g_LinRxBuf.rcvd = 0;
				printf("SLAVE>Wrong Length Discard\r\n");
			}else{
				g_LinRxBuf.rcvd = 1;
				printf("SLAVE>Got a request from the Master = ");
				for(i=0; i<plinMsgInfo->dleng;i++)	printf("%02x ", g_LinRxBuf.data[i]);
				printf("\r\n");
			}
		}
	}
}
#if (USART2_FOR == USART2_FOR_LIN)
//master or slave
void USART2_IRQHandler(void) //void USART2_IntHandlerForLIN(void) //
{
    unsigned long ulIntStatus;
    unsigned char val;
    u8 dlen;
#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
    // (LIN_USART == USART2)
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(LIN_USART, USART_IT_LBD) ){
   		val = USART2->DR; 	// the character from the USART2 data register is saved in t
    	g_LinRxBuf.break_rcvd =1;
    	g_LinRxBuf.sync_rcvd = 0;
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);//clear IRQ //| USART_IT_ORE_RX
	}

	if( USART_GetITStatus(LIN_USART, USART_IT_RXNE) ){
		val = USART2->DR; 	// the character from the USART2 data register is saved in t
		printf("%02x ", val);
   		if(g_LinRxBuf.break_rcvd){ //expect Sync...
   			g_LinRxBuf.break_rcvd=0;
   			if(val == 0x55 ){ //is_Sync?
   				g_LinRxBuf.sync_rcvd=1;
   				//printf("Sync Rcvd\n");
   			}else{
   				printf("Get Char but no Sync Code\n");
   				g_LinRxBuf.sync_rcvd=0;
   			}
   		}else if(g_LinRxBuf.sync_rcvd){//already got sync code. Expect Msg ID...
   			g_LinRxBuf.sync_rcvd=0;
   			g_LinRxBuf.msgid_rcvd=1;
			g_LinRxBuf.msgInfo.msgIndex = stmLinFindMsgIndexFromMsgInfoPool(val,&g_LinMsgInfoPool[0]);
			g_LinRxBuf.msgInfo.dleng = 0;
	  		g_LinRxBuf.msgInfo.msgid = val;
	      	if(g_role == LIN_ROLE_MASTER){ //Readbacked --> will be Discarded
	      		//printf("Get Msg ID(0x%02x):ID[3:0]=0x%02x,ID[5:4]=0x%02x, P[1:0]=0x%02x\r\n",val, getMsgId(val),getMsgId54(val), getParity(val)); //Useless. Thus you should drop it.
	      	}else{
	      		if(g_LinMsgInfoPool[g_LinRxBuf.msgInfo.msgIndex].needResp){
	      			g_LinRxBuf.needRespBySlave = 1;
	      		}else
	      			g_LinRxBuf.needRespBySlave = 0;
	      	}
	    }
	    else if (g_LinRxBuf.msgid_rcvd) { //Previous Event was MsgId_received.
	    	 if(g_LinRxBuf.msgInfo.dleng < g_LinMsgInfoPool[g_LinRxBuf.msgInfo.msgIndex].dleng){
	    		g_LinRxBuf.data[g_LinRxBuf.msgInfo.dleng]=val;
	    		g_LinRxBuf.msgInfo.dleng++;
	    		if(g_LinRxBuf.msgInfo.dleng == g_LinMsgInfoPool[g_LinRxBuf.msgInfo.msgIndex].dleng){
					stmLinParser(&g_LinMsgInfoPool[g_LinRxBuf.msgInfo.msgIndex]);
	    			g_LinRxBuf.sync_rcvd=0;
	    			g_LinRxBuf.msgid_rcvd=0;
	    			g_LinRxBuf.break_rcvd = 0;
	    		}
	    	 }else{
	    		printf("Error : Too long data...Reset\r\n");
	    		g_LinRxBuf.msgInfo.dleng = 0; //used as rx byte count
				g_LinRxBuf.sync_rcvd=0;
				g_LinRxBuf.msgid_rcvd=0;
				g_LinRxBuf.break_rcvd = 0;
	    	 }
	    }
   		USART_ClearITPendingBit(LIN_USART,USART_IT_RXNE);//clear IRQ
    }
#else
#endif
}
#endif

void doMotorControl(unsigned char controldata){
	if(controldata == MotorDirectionCW)
		printf("Do Motor Control : Forward\r\n");
	else if(controldata == MotorDirectionCCW)
		printf("Do Motor Control : Backward\r\n");
	else if(controldata == MotorSpeedUP)
		printf("Do Motor Control : SpeedUp\r\n");
	else if(controldata == MotorSpeedDOWN)
		printf("Do Motor Control : SpeedDown\r\n");
	else{
		printf("Do Motor Control : ???\r\n");
	}
}

int LIN_SlaveTask(void){

	unsigned char msgid;
	u8 i;

	g_LinRxBuf.rcvd = 0;

	while(1){
		if(g_LinRxBuf.needRespBySlave){
			stmLinSlaveResponse(&g_LinMsgInfoPool[g_LinRxBuf.msgInfo.msgIndex]);
  			g_LinRxBuf.needRespBySlave = 0;
		}
		else if(g_LinRxBuf.rcvd) //set by ISR
		{
			printf("Slave>Rx(%x):", g_LinRxBuf.msgInfo.msgid);
			for(i=0; i<g_LinRxBuf.msgInfo.dleng;i++)	printf("%02x ", g_LinRxBuf.data[i]);
			printf("\r\n");

			switch(g_LinRxBuf.msgInfo.msgid){
			case MsgID_0_2A:
				if(g_LinRxBuf.data[0]== MotorDirectionCW)
				{
					doMotorControl(MotorDirectionCW);
				}
				else if(g_LinRxBuf.data[0]== MotorDirectionCCW)
				{
					doMotorControl(MotorDirectionCCW);
				}
				else if(g_LinRxBuf.data[0]== MotorSpeedUP)
				{
					doMotorControl(MotorSpeedUP);
				}
				else if(g_LinRxBuf.data[0]== MotorSpeedDOWN)
				{
					doMotorControl(MotorSpeedDOWN);
				}
				else{
					printf("Invalid\r\n");
				}

			default:
				break;
			}

			g_LinRxBuf.rcvd = 0;
		}
	}
}

//==========================================
int stmLinLoop(void)
{

#ifdef LIN_ROLE_MASTER
	printf("LIN Master Test.");
	g_role = LIN_ROLE_MASTER;
	stmLinNodeInit(LIN_ROLE_MASTER); //Master mode Init

	stmLIN_MasterTask ();
#else
	GuiInit("LIN Slave Test.");
	g_role = LIN_ROLE_SLAVE;
	g_LinRxBuf.rcvd = 0;
	stmLinNodeInit(LIN_ROLE_SLAVE); //Slave mode Init
	LIN_SlaveTask ();
#endif

}

#endif
