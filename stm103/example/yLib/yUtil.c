

#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#else
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
#endif

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)|| (PROCESSOR == PROCESSOR_STM32F107VCT))
#define STM32_CLOCK_HZ 72000000UL
#define STM32_CYCLES_PER_LOOP 6 // This will need tweaking or calculating
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
#define STM32_CLOCK_HZ 84000000UL
#define STM32_CYCLES_PER_LOOP 6 // This will need tweaking or calculating
#else
#define STM32_CLOCK_HZ 168000000UL
#define STM32_CYCLES_PER_LOOP 3 // This will need tweaking or calculating
#endif

extern uint32_t g_tickCnt;
extern uint32_t g_tickEvent;

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        21. September 2015
* $Revision:    V.1.4.5 a
*
* Project:      CMSIS DSP Library
* Title:        arm_cos_f32.c
*
* Description:  Fast cosine calculation for floating-point values.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

//#include "arm_math.h"
//#include "arm_common_tables.h"
/**
 * @ingroup groupFastMath
 */

/**
 * @defgroup cos Cosine
 *
 * Computes the trigonometric cosine function using a combination of table lookup
 * and linear interpolation.  There are separate functions for
 * Q15, Q31, and floating-point data types.
 * The input to the floating-point version is in radians while the
 * fixed-point Q15 and Q31 have a scaled input with the range
 * [0 +0.9999] mapping to [0 2*pi).  The fixed-point range is chosen so that a
 * value of 2*pi wraps around to 0.
 *
 * The implementation is based on table lookup using 256 values together with linear interpolation.
 * The steps used are:
 *  -# Calculation of the nearest integer table index
 *  -# Compute the fractional portion (fract) of the table index.
 *  -# The final result equals <code>(1.0f-fract)*a + fract*b;</code>
 *
 * where
 * <pre>
 *    b=Table[index+0];
 *    c=Table[index+1];
 * </pre>
 */

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        12. March 2014
* $Revision:    V1.4.4
*
* Project:      CMSIS DSP Library
* Title:        arm_sin_f32.c
*
* Description:  Fast sine calculation for floating-point values.
*               Fast cosine calculation for floating-point values.
*
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */
#include <stdint.h>
#if USE_SINE_TABLE
//#include <nanohub_math.h>
#define FAST_MATH_TABLE_SIZE  512


typedef float float32_t;
/**
 * \par
 * Example code for the generation of the floating-point sine table:
 * <pre>
 * tableSize = 512;
 * for(n = 0; n < (tableSize + 1); n++)
 * {
 *	sinTable[n]=sin(2*pi*n/tableSize);
 * }</pre>
 * \par
 * where pi value is  3.14159265358979
 */
static const float32_t sinTable_f32[FAST_MATH_TABLE_SIZE + 1] = {
   0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
   0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
   0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
   0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
   0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
   0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
   0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
   0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
   0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
   0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
   0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
   0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
   0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
   0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
   0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
   0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
   0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
   0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
   0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
   0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
   0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
   0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
   0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
   0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
   0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
   0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
   0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
   0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
   0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
   0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
   0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
   0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
   0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
   0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
   0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
   0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
   0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
   0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
   0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
   0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
   0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
   0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
   0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
   -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
   -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
   -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f,
   -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f,
   -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f,
   -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f,
   -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f,
   -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f,
   -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f,
   -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f,
   -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f,
   -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f,
   -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f,
   -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f,
   -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f,
   -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f,
   -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f,
   -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f,
   -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f,
   -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f,
   -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f,
   -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f,
   -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f,
   -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f,
   -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f,
   -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f,
   -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f,
   -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f,
   -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f,
   -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f,
   -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f,
   -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f,
   -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f,
   -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f,
   -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f,
   -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f,
   -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f,
   -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f,
   -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f,
   -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f,
   -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f,
   -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f,
   -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f,
   -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f,
   -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f,
   -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f,
   -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f,
   -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f,
   -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f,
   -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f,
   -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

 /**
 * @addtogroup cos
 * @{
 */

/**
 * @brief  Fast approximation to the trigonometric cosine function for floating-point data.
 * @param[in] x input value in radians.
 * @return cos(x).
 */

float32_t arm_cos_f32(
  float32_t xRad)// input x is in radians
{
  float32_t cosVal, fract, in;                   /* Temporary variables for input, output */
  uint16_t index;                                /* Index variable */
  float32_t a, b;                                /* Two nearest output values */
  int32_t n;
  float32_t findex;


  // Scale the input to [0..1] range from [0..2*PI] , divide input x by 2*pi, add 0.25 (pi/2) to read sine table
  in = xRad * 0.159154943092f + 0.25f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if(in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0..1] */
  in = in - (float32_t) n;

  /* Calculation of index of the table */
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  index = ((uint16_t)findex) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float32_t) index;

  /* Read two nearest values of input value from the cos table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  cosVal = (1.0f-fract)*a + fract*b;

  /* Return the output value */
  return (cosVal);
}
#else
#endif
/**
 * @} end of cos group
 */

void delayx(unsigned int ms) {
	//4694 = 1 ms
	while (ms > 1) {
		ms--;
		asm("nop");
	}
}
/*
void delayms(u32 msec){ // Wait about 1mS.
	SysCtlDelay(msec*(SYSCTLCLK50MHz/3000)); //each loop takes 3 cycles/loop
}
*/
/*
void delayms(uint32_t ms){
	int x;
	x = x+1;
}
*/

void somedelay(volatile uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
//---- Too fast ??? -------------------
/*
#if (PROCESSOR == PROCESSOR_STM32F103C8 ) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
#define DELAY_TIM_FREQUENCY_MS 1000 //1KHz
void _init_ms(){
	//enable clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM;
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_MS)-1;
	TIM.TIM_Period = 0xFFFF;
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM);

	TIM_Cmd(TIM2,ENABLE);
}
//void _stop_timer(){
//	TIM_Cmd(TIM2,DISABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
//}
void delayms(uint32_t ms){
	_init_ms();

	volatile unsigned int start = TIM2->CNT;
	while((TIM2->CNT - start) <= ms);

	//stop timer
	TIM_Cmd(TIM2,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}
#else
*/
void delayms(uint32_t ms)
{
    ms *= STM32_CLOCK_HZ / 1000 / STM32_CYCLES_PER_LOOP;

    asm volatile(" mov r0, %[ms] \n\t"
             "1: subs r0, #1 \n\t"
             " bhi 1b \n\t"
             :
             : [ms] "r" (ms)
             : "r0");
}

void delayus(uint32_t us)
{
    us *= STM32_CLOCK_HZ / 1000000 / STM32_CYCLES_PER_LOOP;

    asm volatile(" mov r0, %[us] \n\t"
             "1: subs r0, #1 \n\t"
             " bhi 1b \n\t"
             :
             : [us] "r" (us)
             : "r0");
}
//#endif

/*
static void _delay_tick(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   _delay_tick\n"
          "    bx      lr");
}
// micro second ë‹¨ìœ„ë¡œ delay
void delay_us(unsigned long us)
{
    _delay_tick(us * (STM32_CLOCK_HZ / STM32_CYCLES_PER_LOOP / 1000000));
}
// mili second ë‹¨ìœ„ë¡œ delay
void delayms(unsigned long ms)
{
	_delay_tick(ms * (STM32_CLOCK_HZ / STM32_CYCLES_PER_LOOP / 1000));
}
*/

/*-----WORKING-------
//Using Data Watchpoint and Trace Registers(DWT) - 32 bit CYCCNT register couns process cycles.
void _init_ms(){
	if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){ //core_cm3.h
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}
void delayms(uint32_t ms){
	_init_ms();
	int tp = DWT->CYCCNT + ms * (SystemCoreClock / 1000);
	while(((int)DWT->CYCCNT - tp) < 0);
}
---------------*/
unsigned long micros(){
	return g_tickCnt;
}
unsigned long millis(){
	return g_tickCnt * 10;
}
//===================================
//===================================
char *float2str(float x){
	volatile char str[100];
	char *tmpSign = (x < 0) ? "-" : "";
	float tmpVal = (x < 0) ? -x : x;

	int tmpInt1 = tmpVal;              		// Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;  		// Get fraction (0.0123).
	int tmpInt2 = trunc(tmpFrac * 10000.0);  	// Turn into integer (123).
	sprintf (str, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	return str;
}

void testFloat(){
	//test float
	int i;
	while(1){

		float f;

		f = (float)i/100.0;
		printf("%s\r\n", float2str(f));
		delayms(100);
		i++;
	}
}

void y_itoa (uint8_t* str, uint8_t len, uint32_t val)
{
  uint8_t i;
  for(i=0; i<=len; i++)
  {
    str[len-i] = (uint8_t) ((val % 10UL) + '0');
    val/=10;
  }
}
//================================= TBD ================================================
//PB5 = PPS each 1sec.
void PB5Init(){
	  GPIO_InitTypeDef GPIO_InitStruct;
	  /* Configure PB5*/
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#else
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 	// this sets the GPIO modules clock speed
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
	  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	  delayms(1);
	  GPIO_SetBits(GPIOB, GPIO_Pin_5); // PB5 ON
	  delayms(1);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	  delayms(1);

}
void PB15Init(){
	  GPIO_InitTypeDef GPIO_InitStruct;
	  /* Configure PB5*/
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#else
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 	// this sets the GPIO modules clock speed
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
	  GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	  delayms(1);
	  GPIO_SetBits(GPIOB, GPIO_Pin_15); // PB15 ON
	  delayms(1);
	  GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	  delayms(1);
}
void sysTickTestLoop(){
	GPIO_InitTypeDef GPIO_InitStruct;

	PB5Init();

	Init_SysTick(1000);

	  while(1){
		  if(g_tickEvent){
			  GPIO_SetBits(GPIOB, GPIO_Pin_15); // PB15 ON
			  delayms(100);
			  GPIO_ResetBits(GPIOB,  GPIO_Pin_15 ); //PB15 OFF
			  g_tickEvent = 0;
			  //delayms(900);
		  }
	  }
}


void stmSyncProjLoop(unsigned char uartId)
{
	int i;
	short len, rxchar;
	char nmeastr[256];
	int numLEDsegments = 1;//2
	int nCS;
    unsigned freq;// = 25000000;//25MHz
    //unsigned waveForm = SINE;

	printf(">GPS Test with Usart%u<\r\n", uartId);
	if(uartId==3)
		stmUSART3ShimConf(9600); //Interrupt Mode. Rx Only
	else
		stmUSART1ShimConf(9600); //Interrupt Mode. Rx Only

	printf("GPS>GpsConf(9600bps).\r\n");

	//7-segment LED Driver Config
	nCS=1;

	stmMax7219_Init(2, stmSPIMODE3, stmSPIDATA8, numLEDsegments, nCS);
	stmMax7219_config();
/*
	//Function Generator Config (max12.5MHz)
	nCS=0;
    stmAD9833_Init(nCS);
    freq = 400000;//400KHz
	printf("Freq=%d\r\n", freq); //TBD
	stmAD9833_GenFreq(freq, 0x2028);//0x2000);//0x2000);//#define SINE 0x2000 SQUARE = 0x2028
*/

	while(1) //main loop
    {
		memset(nmeastr,0x00,256);
		if(uartId==3)
			len = stmUSART3_GetNMEAString(nmeastr);
		else
			len = stmUSART1_GetNMEAString(nmeastr);
		if(len){
			printf("%s",nmeastr);
			stmGpsGetTimeOfDayAndDisplay(uartId);
		}


/*
		rxchar = USART3_GetChar();
		if(rxchar >=0){
			printf("%c",(char)rxchar);
*/
	}
}
