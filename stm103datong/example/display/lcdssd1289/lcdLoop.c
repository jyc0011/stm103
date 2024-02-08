/**
 *
 */
/* Includes *******************************************************************/
#include "lcdLoop.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 300;
uint16_t CCR2_Val = 100;

int lcdLoop (void)
{
  /* Initialization */

  //Init_SysTick();
  Init_GPIO();
  Init_FSMC();
  Init_LCD();
  //pwm_init();
  touch_init();

  /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f4xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f4xx.c file
       */

  /* Demo */
  //TIM4->CCR1 = CCR1_Val;  //TIM4->CCR2 = CCR2_Val;

  Clear_Screen(0x0000);
	Demo_MMIA();
  while(1)
  {
	  delayms(100);
	  Convert_Pos();

    //Draw_Image(0, 319, 240, 320, img03);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img04);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img05);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img06);
    //delayms(3000);
  }

  //return 0;
}

/*
 * Demonstration project designed for MMIA.
 */

void Demo_MMIA(void)
{
  uint16_t Number=0;
  //int CharCount;
  char StrNumber[10];

  Clear_Screen(0x0000);

  // delayms(3000);

  Set_Font(&Font16x24);
  Display_String(14, 295, "EtherCrafts", LCD_WHITE);
  uint16tostr(StrNumber, Number, 10);
  Display_String(43, 295, StrNumber, LCD_WHITE);

  //CharCount = sprintf(StrNumber,"%d", Number);
  //Display_String(43, 295, StrNumber, LCD_WHITE);

  Display_String(72, 287, "KISADO", LCD_WHITE);
  Set_Font(&Font12x12);
  Display_String(97, 285, "STM32F407", LCD_WHITE);

  Draw_Image(120, 195, 70, 70, img00);

  Set_Font(&Font8x8);
  Display_String(220, 259, "Compiled by YOON", LCD_WHITE);
  Display_String(230, 259, "SSD1289 + XPT2046", LCD_WHITE);
  delayms(5000);
  Number = 70;
  Set_Font(&Font16x24);
  while (Number != 0)
  {
	  Draw_Full_Rect(43, 295 ,61 ,25 , LCD_BLACK);
	  uint16tostr(StrNumber, Number, 10);
	  Display_String(43, 295, StrNumber, LCD_WHITE);
	  //CharCount = sprintf(StrNumber,"%d", Number);
	  //Display_String(43, 295, StrNumber, LCD_WHITE);
	    //TIM_OCInitStructure.TIM_Pulse = Number;
	    //TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	    //TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	  delayms(20);
	  Number--;
  }

  GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

//	Clear_Screen(0x0000);
//	Draw_Image(0, 319, 240, 320, img02);
//	delayms(3000);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 255, "Random Lines", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Lines();
  delayms(500);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 295, "Random Rectangles", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Rect();
  delayms(500);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 271, "Random Circles", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Circle();
  delayms(500);

/*  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 199, "Images", LCD_WHITE);

  delayms(2000);
  Draw_Image(0, 319, 240, 320, img02);
*/
}


/*
 * Draw random lines.
 */

void Random_Lines(void)
{
  uint16_t x1,y1,x2,y2;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<100;i++)
  {
    x1=rand() % 240;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y1=rand() % 320;
    x2=rand() % 240;
    y2=rand() % 320;

    cr=rand();

    Draw_Line(x1, y1 ,x2 ,y2 , cr << 3);
    delayms(100);
  }
}

/*
 * Draw random rectangles.
 */

void Random_Rect(void)
{
  uint16_t x1,y1,x2,y2,z;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<25;i++)
  {
    x1=rand() % 240;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y1=rand() % 320;
    x2=rand() % 240;
    y2=rand() % 320;

    cr=rand();

    z=rand() % 10;

    if (z >= 5) Draw_Rect(x1, y1 ,x2 ,y2 , cr << 3);
    else Draw_Full_Rect(x1, y1 ,x2 ,y2 , cr << 3);
    delayms(100);
  }
}

/*
 * Draw random circles.
 */

void Random_Circle(void)
{
  uint16_t x, y, r, z;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<25;i++)
  {
    x=rand() % 140;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y=rand() % 220;
    r=(rand() % 50) + 1;

    cr=rand() << 3;

    z=rand() % 10;

    if (z >= 5) Draw_Circle(x+50, y+50, r, cr);
    else Draw_Full_Circle(x+50, y+50, r, cr);
    delayms(100);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
