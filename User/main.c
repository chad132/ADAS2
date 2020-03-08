#include "stm32f10x.h"
#include <stdio.h>
#include "USART.h"
#include "delay.h"
#include "CAN.h"
#include "LED.h"
#include "Timer.h"
uint8_t Flag_dw3 = 0;

RCC_ClocksTypeDef RCC_Clocks;
int main(void)
{
	
	 Delay_Init(72);//�δ�ʱ�ӳ�ʼ��
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж�Ϊ����2
	 LED_Init();
	 CANInit();
	 TIM2_Init();
	 TIM4_Init();
	 USART1_Init();
	 RCC_GetClocksFreq(&RCC_Clocks);//debugģʽ�¶�ȡ����ģ���ʱ��
	 Flag_dw3 =5;
	 
   while(1)
   {
//   	LED_RED_ON();
//		Delay_Ms(500);
//		LED_GREEN_ON();
//		Delay_Ms(500); 
//    LED_BLUE_ON();		 
//		Delay_Ms(500); 
		 

		 
		 
//		 if(Flag_Alarm == 0)//�ޱ������̵���
//		 {
//		   LED_GREEN_ON();
//		 }
//		 else if(Flag_Alarm == 1)//FCW�����������
//		 {
//		   LED_RED_ON();
//		 }
//		 else if(Flag_Alarm == 2)//LDW������������
//		 {
//		   LED_BLUE_ON();
//		 }
//		 else if(Flag_Alarm == 3)//ȫ������
//		 {
//		   LED_REDBLUE_ON();
//		 }
   }
	 
	 
}