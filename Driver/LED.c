#include "LED.h"


void LED_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; 
	 RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE);
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;//RL1,RL2,RL3 À¶¡¢»Æ¡¢ºì
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

//À¶É«

  
void LED_BLUE_ON(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
}


//ÂÌÉ«
void LED_GREEN_ON(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
}


//ºìÉ«
void LED_RED_ON(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}

//ºìÀ¶È«ÁÁ
void LED_REDBLUE_ON(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}

//È«Ãð
void LED_OFF(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
}